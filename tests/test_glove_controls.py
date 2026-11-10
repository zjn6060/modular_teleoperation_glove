"""Check sidebar order, raising direction and bidirectional motor conversion."""

import mujoco
import numpy as np
import pytest

from driver import MotorConfig, Register, SimulatedBus, XL330Driver
from driver.errors import SafetyError
from simulation.glove.reference_pose import SCENE_PATH, reference_angles, reset_reference_pose
from simulation.glove.servo_bridge import SYNC_FRAMES, make_bridge

FINGERS = (
    ("index", 2, "j2", "short_ef_28_045", "short_fgh"),
    ("middle", 3, "j10", "lang_ef_36_549", "lang_fgh"),
    ("ring", 4, "j26", "part_1_16", "part_1_17"),
    ("little", 5, "j18", "part_1_6", "part_1_7"),
)


def make_setup():
    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    data = mujoco.MjData(model)
    reset_reference_pose(model, data)
    offsets = reference_angles()
    config = {"servos": [
        {"finger": finger, "motor_id": motor_id, "joint": joint, "offset": offsets[joint]}
        for finger, motor_id, joint, _, _ in FINGERS
    ]}
    motors = [MotorConfig(motor_id=i, name=f, zero_position=2048, direction=-1,
                          min_position=1800, max_position=2300) for f, i, *_ in FINGERS]
    bus = SimulatedBus([m.motor_id for m in motors])
    return model, data, config, motors, bus


def test_sidebar_pairs_follow_finger_order():
    model, *_ = make_setup()
    assert [model.actuator(i).name for i in range(model.nu)] == [
        "index_mcp_abduction", "j2", "middle_mcp_abduction", "j10",
        "ring_mcp_abduction", "j26", "little_mcp_abduction", "j18",
    ]


@pytest.mark.parametrize("finger,motor_id,joint,ef,f", FINGERS)
def test_left_slider_stops_at_horizontal_and_right_slider_flexes(finger, motor_id, joint, ef, f):
    model, data, *_ = make_setup()
    a = model.actuator(joint).id
    horizontal = data.ctrl[a]
    assert model.actuator_ctrlrange[a, 0] == horizontal
    data.ctrl[a] -= 0.5
    for _ in range(round(2 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    assert abs(data.xpos[model.body(f).id, 2] - data.xpos[model.body(ef).id, 2]) < 1e-6
    data.ctrl[a] = horizontal + 0.02
    for _ in range(round(2 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    assert data.xpos[model.body(f).id, 2] - data.xpos[model.body(ef).id, 2] < -0.0005
    data.ctrl[a] = horizontal - 0.5
    for _ in range(round(2 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    assert abs(data.xpos[model.body(f).id, 2] - data.xpos[model.body(ef).id, 2]) < 1e-6
    assert not any(w.number for w in data.warning)


@pytest.mark.parametrize("finger,motor_id,joint,ef,f", FINGERS)
def test_sim2real_reverses_slider_gear_and_keeps_other_motors_still(finger, motor_id, joint, ef, f):
    model, data, config, motors, bus = make_setup()
    by_id = {m.motor_id: m for m in motors}
    with XL330Driver(bus, motors) as driver:
        driver.enable_torque()
        tick = make_bridge(driver, config, motors, "sim2real")(model, data)
        for _ in range(SYNC_FRAMES + 1):
            tick()
        assert all(bus.read(m.motor_id, int(Register.GOAL_POSITION), 4) == m.zero_position
                   for m in motors)
        actuator = model.actuator(joint).id
        gear = model.actuator_gear[actuator, 0]
        data.ctrl[actuator] += 0.02
        tick()
        assert bus.read(motor_id, int(Register.GOAL_POSITION), 4) == (
            by_id[motor_id].radians_to_position(0.02 / gear))
        assert all(bus.read(m.motor_id, int(Register.GOAL_POSITION), 4) == m.zero_position
                   for m in motors if m.motor_id != motor_id)
        offset = reference_angles()[joint]
        for target in (-10.0, 10.0):
            data.ctrl[actuator] = target
            for _ in range(30):
                tick()
            control = np.clip(target, *model.actuator_ctrlrange[actuator])
            assert bus.read(motor_id, int(Register.GOAL_POSITION), 4) == (
                by_id[motor_id].radians_to_position(control / gear - offset, clamp=True))
            assert (bus.read(motor_id, int(Register.GOAL_POSITION), 4)
                    - by_id[motor_id].zero_position) * by_id[motor_id].direction * gear >= 0


def test_real2sim_applies_gear_without_changing_calibration_angles():
    model, data, config, motors, bus = make_setup()
    with XL330Driver(bus, motors) as driver:
        for m in motors:
            bus.write(m.motor_id, int(Register.PRESENT_POSITION), 4, m.zero_position + 27)
        tick = make_bridge(driver, config, motors, "real2sim")(model, data)
        tick()
        for servo, motor in zip(config["servos"], motors):
            a = model.actuator(servo["joint"]).id
            expected_joint = motor.position_to_radians(motor.zero_position + 27) + servo["offset"]
            np.testing.assert_allclose(data.ctrl[a], np.clip(
                model.actuator_gear[a, 0] * expected_joint, *model.actuator_ctrlrange[a]))
            assert bus.read(motor.motor_id, int(Register.PRESENT_POSITION), 4) == motor.zero_position + 27


@pytest.mark.parametrize("finger,motor_id,joint,ef,f", FINGERS)
def test_sim2real_does_not_command_startup_from_above_horizontal(finger, motor_id, joint, ef, f):
    model, data, config, motors, bus = make_setup()
    with XL330Driver(bus, motors) as driver:
        gear = int(model.actuator_gear[model.actuator(joint).id, 0])
        current = 2048 + gear * 10
        bus.write(motor_id, int(Register.PRESENT_POSITION), 4, current)
        driver.enable_torque()
        tick = make_bridge(driver, config, motors, "sim2real")(model, data)
        with pytest.raises(SafetyError, match=f"ID {motor_id}: current pose"):
            tick()
        assert bus.read(motor_id, int(Register.GOAL_POSITION), 4) == current
