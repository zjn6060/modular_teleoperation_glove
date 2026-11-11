"""Check sidebar order, raising direction and bidirectional motor conversion."""

import mujoco
import numpy as np
import pytest

from driver import MotorConfig, Register, SimulatedBus, XL330Driver
from driver.errors import SafetyError
from simulation.glove.reference_pose import SCENE_PATH, reference_angles, reset_reference_pose
from simulation.glove.servo_bridge import HOME_SPEED, MAX_DT, make_bridge, temporary_current_test

FINGERS = (
    ("index", 2, "index_motor", "short_ef_28_045", "short_fgh"),
    ("middle", 3, "middle_motor", "lang_ef_36_549", "lang_fgh"),
    ("ring", 4, "ring_motor", "part_1_16", "part_1_17"),
    ("little", 5, "little_motor", "part_1_6", "part_1_7"),
)


def make_setup():
    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    data = mujoco.MjData(model)
    reset_reference_pose(model, data)
    offsets = reference_angles()
    config = {"reference_pose": "ef_horizontal", "servos": [
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
        "index_mcp_abduction", "index_motor", "middle_mcp_abduction", "middle_motor",
        "ring_mcp_abduction", "ring_motor", "little_mcp_abduction", "little_motor",
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
        tick = timed_bridge(driver, config, motors, model, data)
        for _ in range(20):
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


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def advance(self, seconds=1 / 60):
        self.now += seconds


def timed_bridge(driver, config, motors, model, data, **kwargs):
    clock = Clock()
    callback = make_bridge(driver, config, motors, "sim2real", clock=clock, **kwargs)(model, data)

    def tick(seconds=1 / 60):
        clock.advance(seconds)
        callback()

    return tick


@pytest.mark.parametrize("finger,motor_id,joint,ef,f", FINGERS)
@pytest.mark.parametrize("side", (-1, 1))
def test_sim2real_homes_real_pose_without_moving_sim_reference(finger, motor_id, joint, ef, f, side):
    model, data, config, motors, bus = make_setup()
    gear = int(model.actuator_gear[model.actuator(joint).id, 0])
    current = 2048 + side * gear * 30
    bus.registers[motor_id][Register.PRESENT_POSITION] = current
    with XL330Driver(bus, motors) as driver:
        tick = timed_bridge(driver, config, motors, model, data)
        assert bus.registers[motor_id][Register.GOAL_POSITION] == current
        reference = data.ctrl.copy()
        data.ctrl[model.actuator(joint).id] += 0.2  # ignored during homing
        tick()
        np.testing.assert_allclose(data.ctrl, reference)
        first = bus.registers[motor_id][Register.GOAL_POSITION]
        assert abs(first - current) <= HOME_SPEED / 60 * 4096 / (2 * np.pi) + 1
        for _ in range(60):
            tick()
        assert bus.registers[motor_id][Register.GOAL_POSITION] == 2048
        assert bus.registers[motor_id][Register.PRESENT_POSITION] == 2048
        np.testing.assert_allclose(data.ctrl, reference)
    assert all(bus.registers[m.motor_id][Register.TORQUE_ENABLE] == 0 for m in motors)


def test_homing_waits_for_feedback_and_times_out_with_torque_cleanup(monkeypatch, capsys):
    model, data, config, motors, bus = make_setup()
    motor_id = motors[0].motor_id
    current = 2100
    bus.registers[motor_id][Register.PRESENT_POSITION] = current
    write = bus.write

    def stalled_write(i, address, size, value):
        present = bus.registers[i][Register.PRESENT_POSITION]
        write(i, address, size, value)
        if address == Register.GOAL_POSITION:
            bus.registers[i][Register.PRESENT_POSITION] = present

    monkeypatch.setattr(bus, "write", stalled_write)
    with pytest.raises(SafetyError, match="homing timed out"):
        with XL330Driver(bus, motors) as driver:
            tick = timed_bridge(driver, config, motors, model, data, home_timeout=1.0)
            reference = data.ctrl.copy()
            for _ in range(70):
                data.ctrl[model.actuator('index_motor').id] += 0.2
                tick()
                np.testing.assert_allclose(data.ctrl, reference)
    assert bus.registers[motor_id][Register.GOAL_POSITION] == 2048  # commanded home, not actually there
    assert bus.registers[motor_id][Register.PRESENT_POSITION] == current
    assert 'Sliders ready' not in capsys.readouterr().out
    assert all(bus.registers[m.motor_id][Register.TORQUE_ENABLE] == 0 for m in motors)


@pytest.mark.parametrize("finger,motor_id,joint,ef,f", FINGERS)
@pytest.mark.parametrize("hz", (30, 60, 120))
def test_flexion_and_extension_have_same_wall_clock_target_speed(finger, motor_id, joint, ef, f, hz):
    model, data, config, motors, bus = make_setup()
    motor = next(m for m in motors if m.motor_id == motor_id)
    with XL330Driver(bus, motors) as driver:
        tick = timed_bridge(driver, config, motors, model, data, max_speed=0.4)
        for _ in range(20):  # already horizontal; wait for the feedback settling interval
            tick()
        actuator = model.actuator(joint).id
        reference = data.ctrl[actuator]
        data.ctrl[actuator] = reference + 0.3
        for _ in range(hz // 5):
            tick(1 / hz)
        angle = motor.position_to_radians(bus.registers[motor_id][Register.GOAL_POSITION])
        assert abs(angle) == pytest.approx(0.4 * 0.2, abs=2 * np.pi / 4096)
        data.ctrl[actuator] = reference
        for _ in range(hz // 5):
            tick(1 / hz)
        assert bus.registers[motor_id][Register.GOAL_POSITION] == 2048


def test_delayed_viewer_frame_cannot_create_large_homing_jump():
    model, data, config, motors, bus = make_setup()
    bus.registers[2][Register.PRESENT_POSITION] = 2140
    with XL330Driver(bus, motors) as driver:
        tick = timed_bridge(driver, config, motors, model, data)
        tick(3.0)
        delta = 2140 - bus.registers[2][Register.GOAL_POSITION]
        assert 0 < delta <= HOME_SPEED * MAX_DT * 4096 / (2 * np.pi) + 1


@pytest.mark.parametrize("fault", ('mode', 'reference', 'limits'))
def test_invalid_startup_is_rejected_before_torque_enable(fault):
    model, data, config, motors, bus = make_setup()
    if fault == 'mode':
        bus.registers[2][Register.OPERATING_MODE] = 0
    elif fault == 'reference':
        config['servos'][0]['offset'] += 0.1
    else:
        bus.registers[2][Register.PRESENT_POSITION] = 2400
    with XL330Driver(bus, motors) as driver:
        with pytest.raises(SafetyError):
            timed_bridge(driver, config, motors, model, data)
        assert not any(address == Register.TORQUE_ENABLE and value == 1
                       for _, address, _, value in bus.writes)


def test_bridge_keeps_existing_current_and_profile_settings():
    model, data, config, motors, bus = make_setup()
    with XL330Driver(bus, motors) as driver:
        for motor in motors:
            bus.registers[motor.motor_id][Register.OPERATING_MODE] = 5
            bus.registers[motor.motor_id][Register.CURRENT_LIMIT] = 20
            bus.registers[motor.motor_id][Register.GOAL_CURRENT] = 15
        tick = timed_bridge(driver, config, motors, model, data, diagnostics=True)
        tick()
        assert not any(address in (Register.CURRENT_LIMIT, Register.GOAL_CURRENT,
                                   Register.PROFILE_VELOCITY, Register.PROFILE_ACCELERATION,
                                   Register.OPERATING_MODE, Register.DRIVE_MODE)
                       for _, address, _, value in bus.writes)


@pytest.mark.parametrize('test_current', (False, True))
def test_recorded_low_current_tracking_errors_unlock_sliders_and_send_targets(monkeypatch, capsys, test_current):
    """The real 20-mA log settled at about +4, -4, -10, +1 angle ticks.

    A perfect simulated motor hid the old all-fingers / two-tick lockout.
    Keep these residual errors even after a position command has been sent.
    """
    model, data, config, motors, bus = make_setup()
    biases = {2: -4, 3: 4, 4: 10, 5: -1}  # raw encoder sign; configured direction is -1
    write = bus.write

    def imperfect_write(i, address, size, value):
        write(i, address, size, value)
        if address == Register.GOAL_POSITION:
            bus.registers[i][Register.PRESENT_POSITION] = value + biases[i]

    monkeypatch.setattr(bus, 'write', imperfect_write)
    for motor in motors:
        bus.registers[motor.motor_id].update({Register.OPERATING_MODE: 5,
                                             Register.CURRENT_LIMIT: 20, Register.GOAL_CURRENT: 20,
                                             Register.POSITION_P_GAIN: 250, Register.POSITION_D_GAIN: 600})
    with XL330Driver(bus, motors) as driver:
        with temporary_current_test(driver, 2 if test_current else None, 25 if test_current else None):
            tick = timed_bridge(driver, config, motors, model, data)
            for _ in range(40):
                tick()
            assert 'Sliders ready' in capsys.readouterr().out
            actuator = model.actuator('index_motor').id
            data.ctrl[actuator] += 0.05
            tick()
            assert bus.registers[2][Register.GOAL_POSITION] == motors[0].radians_to_position(-0.05)
            assert all(bus.registers[m.motor_id][Register.GOAL_POSITION] == m.zero_position
                       for m in motors if m.motor_id != 2)
            assert bus.registers[2][Register.GOAL_CURRENT] == (25 if test_current else 20)
            assert all(bus.registers[m.motor_id][Register.POSITION_P_GAIN] == 250
                       and bus.registers[m.motor_id][Register.POSITION_D_GAIN] == 600 for m in motors)
    assert bus.registers[2][Register.CURRENT_LIMIT] == 20
    assert bus.registers[2][Register.GOAL_CURRENT] == 20


def test_strict_arrival_tolerance_reports_lock_and_still_times_out(monkeypatch, capsys):
    model, data, config, motors, bus = make_setup()
    write = bus.write

    def residual_write(i, address, size, value):
        write(i, address, size, value)
        if i == 5 and address == Register.GOAL_POSITION:
            bus.registers[i][Register.PRESENT_POSITION] = value + 3

    monkeypatch.setattr(bus, 'write', residual_write)
    with pytest.raises(SafetyError, match='sliders stayed locked'):
        with XL330Driver(bus, motors) as driver:
            tick = timed_bridge(driver, config, motors, model, data,
                                home_tolerance_ticks=2, home_timeout=0.5)
            for _ in range(40):
                tick()
    output = capsys.readouterr().out
    assert 'HOMING: sliders locked' in output
    assert 'ID 5:' in output
    assert 'Sliders ready' not in output
    assert all(bus.registers[m.motor_id][Register.TORQUE_ENABLE] == 0 for m in motors)


def test_a_brief_arrival_sample_does_not_unlock_sliders(monkeypatch, capsys):
    model, data, config, motors, bus = make_setup()
    actual = {m.motor_id: 0.0 for m in motors}
    with XL330Driver(bus, motors) as driver:
        tick = timed_bridge(driver, config, motors, model, data)
        monkeypatch.setattr(driver, 'read_angles', lambda: actual.copy())
        tick(0.05)
        tick(0.05)
        actual[3] = 0.03  # leaves the arrival tolerance; the settling interval must restart
        tick(0.05)
        actual[3] = 0.015
        tick(0.05)
        tick(0.15)
        assert 'Sliders ready' not in capsys.readouterr().out
        tick(0.06)
        assert 'Sliders ready' in capsys.readouterr().out


@pytest.mark.parametrize('tolerance', (0, -1, 1.5))
def test_invalid_home_tolerance_is_rejected_before_torque(tolerance):
    model, data, config, motors, bus = make_setup()
    with XL330Driver(bus, motors) as driver:
        with pytest.raises(ValueError, match='home_tolerance_ticks'):
            timed_bridge(driver, config, motors, model, data, home_tolerance_ticks=tolerance)
        assert bus.writes == []
