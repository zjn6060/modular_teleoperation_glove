"""Offline contact and register-lifecycle regression tests; no hardware required."""
from types import SimpleNamespace

import mujoco
import numpy as np
import pytest

from driver import SimulatedBus, XL330Driver, Register
from driver.errors import SafetyError
from simulation.glove.haptic_demo import (
    RESTORE, VirtualWall, KinematicWall, ramp_current, build_scene, check_state, current_session,
    signed_current,
)
from utils.glove_servos import load_config, motor_configs


@pytest.fixture
def finger():
    servo = next(s for s in load_config()['servos'] if s['finger'] == 'index')
    return servo, motor_configs({'servos': [servo]})[0]


@pytest.mark.parametrize('motion', ['slow', 'jump', 'random'])
def test_real_geometry_blocks_without_penetration_and_releases(finger, motion):
    servo, motor = finger
    model, data, aid, gear, lo, hi = build_scene(servo, motor)
    proxy = KinematicWall(model, data, aid, gear, lo, hi)
    wall = VirtualWall()
    qadr = model.jnt_qposadr[model.actuator_trnid[aid, 0]]
    if motion == 'slow':
        inputs = np.r_[np.linspace(lo, hi, 250), np.linspace(hi, lo, 250)]
    elif motion == 'jump':
        inputs = np.array([lo, hi] * 50 + [lo])
    else:
        inputs = np.r_[hi, np.random.default_rng(42).uniform(lo, hi, 500), lo]
    peak = 0
    for requested in inputs:
        data.ctrl[aid] = requested
        actual = proxy.render(data, requested)
        current = wall.update(requested, proxy.boundary, True)
        peak = max(peak, current)
        assert actual <= proxy.boundary
        assert gear * data.qpos[qadr] == pytest.approx(actual, abs=1e-9)
        assert proxy.gap(data.qpos) >= -1e-7
        assert max(abs(data.efc_pos[:model.neq * 3])) < 1e-8
        assert data.ctrl[aid] == requested  # Input survives the visual pose clamp.
        for direction in (-1, 1):
            assert signed_current(current, gear, direction) * gear * direction <= 0
    assert peak == 70
    assert wall.boundary is None and current == 0
    # Reach full current with >0.1 rad left before the calibrated flexion limit.
    assert proxy.boundary + 70 / wall.stiffness < hi - .1
    assert not any(w.number for w in data.warning)


def test_disabled_feedback_and_retreat_release():
    wall = VirtualWall(stiffness=120, max_current=20)
    assert wall.update(.19, .2, True) == 0
    assert wall.update(.5, .2, True) == 20
    assert wall.update(.19, .2, True) == 0
    assert wall.boundary is None
    assert wall.update(.5, .2, True) == 20
    assert wall.update(.5, .2, False) == 0
    assert wall.boundary is None


@pytest.mark.parametrize('rate', [30, 60, 125])
def test_time_based_current_ramp_and_immediate_release(rate):
    value = 0
    for _ in range(round(rate * .4)):
        value = ramp_current(70, value, 1/rate)
    assert value == 70
    assert ramp_current(0, value, 1/rate) == 0
    assert ramp_current(70, 0, 5) == 10  # Never jump to full force on a delayed sample.


@pytest.mark.parametrize('failure', [None, RuntimeError, KeyboardInterrupt])
@pytest.mark.parametrize('cap', [20, 70])
def test_current_mode_restores_settings_and_leaves_torque_off(finger, failure, cap):
    _, motor = finger
    mid = motor.motor_id
    bus = SimulatedBus([mid])
    with XL330Driver(bus, [motor]) as driver:
        driver.write_register(mid, Register.GOAL_POSITION, motor.zero_position)
        driver.write_register(mid, Register.OPERATING_MODE, 5)
        driver.write_register(mid, Register.CURRENT_LIMIT, 70)
        driver.write_register(mid, Register.GOAL_CURRENT, 70)
        original = {(a, s): bus.read(mid, a, s) for a, s in RESTORE}
        # Emulate real mode changes resetting gains/goals (plain SimulatedBus does not).
        real_write = bus.write
        def reset_on_mode(m, a, s, value):
            real_write(m, a, s, value)
            if a == 11:
                for address, size in RESTORE:
                    if address not in (36, 38):
                        real_write(m, address, size, 0)
        bus.write = reset_on_mode
        before_session = len(bus.writes)
        def exercise():
            with current_session(driver, motor, cap):
                assert bus.read(mid, 11, 1) == 0
                assert bus.read(mid, 38, 2) == cap
                assert bus.read(mid, 102, 2) == 0
                driver.enable_torque()
                driver.command_currents({mid: -15})
                if failure:
                    raise failure('injected')
        if failure:
            with pytest.raises(failure):
                exercise()
        else:
            exercise()
        assert bus.read(mid, 11, 1) == 5
        assert bus.read(mid, 64, 1) == 0
        assert {(a, s): bus.read(mid, a, s) for a, s in RESTORE} == original
        assert all(entry[0] == mid for entry in bus.writes)
        assert not any(address == 116 for _, address, _, _ in bus.writes[before_session:])


def test_outside_limits_and_hardware_errors_stop(finger):
    _, motor = finger
    for raw, error in [(motor.min_position - 1, 0), (motor.max_position + 1, 0), (motor.zero_position, 4)]:
        with pytest.raises(SafetyError):
            check_state(SimpleNamespace(position_raw=raw, hardware_error=error, position_rad=0), motor)


@pytest.mark.parametrize('close_during_sync', [False, True])
def test_preview_viewer_loop_slider_and_close(monkeypatch, finger, capsys, close_during_sync):
    """Exercise the actual run loop with GUI input; no GL or hardware involved."""
    import contextlib
    import mujoco.viewer
    from simulation.glove import haptic_demo as demo
    servo, motor = finger
    model, data, aid, gear, lo, hi = build_scene(servo, motor)

    class Viewer:
        cam = SimpleNamespace(lookat=np.zeros(3), distance=0, azimuth=0, elevation=0)
        frames = 0
        texts = None
        def __enter__(self):
            return self
        def __exit__(self, *exc):
            pass
        def lock(self):
            return contextlib.nullcontext()
        def is_running(self):
            return self.frames < (1 if close_during_sync else 4)
        def sync(self):
            self.frames += 1
            data.ctrl[aid] = hi
        def set_texts(self, texts):
            self.texts = texts

    viewer = Viewer()
    monkeypatch.setattr(mujoco.viewer, 'launch_passive', lambda *a, **kw: viewer)
    monkeypatch.setattr(demo.time, 'sleep', lambda _: None)
    initial_time = data.time
    demo.run(SimpleNamespace(stiffness=120, max_current_ma=20), servo, motor,
             model, data, aid, gear, lo, hi)
    if close_during_sync:
        assert data.time == initial_time  # No physics after GUI reports closure.
    else:
        assert data.time > initial_time
        assert data.ctrl[aid] == hi  # GUI input survives frame processing.
        assert 'NO HARDWARE' in viewer.texts[2]
    assert 'PREVIEW ONLY' in capsys.readouterr().out


@pytest.mark.parametrize('position', [1290, 2048])
def test_preflight_outside_travel_can_open_but_stays_torque_off(finger, capsys, position):
    _, motor = finger
    bus = SimulatedBus([motor.motor_id])
    with XL330Driver(bus, [motor]) as driver:
        bus.write(motor.motor_id, 132, 4, position)
        with current_session(driver, motor, 70):
            assert bus.read(motor.motor_id, 64, 1) == 0
            assert bus.read(motor.motor_id, 102, 2) == 0
        assert not any(a == 64 and v == 1 for _, a, _, v in bus.writes)
    assert 'feedback stays OFF' in capsys.readouterr().out


def test_preflight_hardware_error_rejects_before_any_write(finger):
    _, motor = finger
    bus = SimulatedBus([motor.motor_id])
    with XL330Driver(bus, [motor]) as driver:
        bus.write(motor.motor_id, 70, 1, 4)
        before = list(bus.writes)
        with pytest.raises(SafetyError, match='hardware error=0x04'):
            with current_session(driver, motor, 70):
                pytest.fail('Hardware fault must prevent setup')
        assert bus.writes == before


def test_hardware_fault_is_distinguished_from_travel(finger):
    _, motor = finger
    state = SimpleNamespace(position_raw=motor.zero_position, hardware_error=4, position_rad=0)
    with pytest.raises(SafetyError, match='hardware error=0x04') as error:
        check_state(state, motor)
    assert 'outside calibrated' not in str(error.value)


@pytest.mark.parametrize('active_error', [False, True])
def test_restore_mismatch_preserves_original_and_restores_watchdog(finger, active_error):
    _, motor = finger
    mid = motor.motor_id
    bus = SimulatedBus([mid])
    with XL330Driver(bus, [motor]) as driver:
        driver.write_register(mid, Register.GOAL_POSITION, motor.zero_position)
        driver.write_register(mid, Register.OPERATING_MODE, 5)
        driver.write_register(mid, Register.BUS_WATCHDOG, 7)
        original_write = bus.write
        def mismatch_gain(m, a, s, value):
            original_write(m, a, s, value + 1 if a == 84 else value)
        with pytest.raises(SafetyError) as error:
            with current_session(driver, motor, 20):
                bus.write = mismatch_gain
                if active_error:
                    raise SafetyError('original travel violation')
        assert bus.read(mid, 64, 1) == 0
        assert bus.read(mid, 98, 1) == 7
        if active_error:
            assert str(error.value) == 'original travel violation'
            diagnostic = ' '.join(error.value.__notes__)
        else:
            diagnostic = str(error.value)
        assert 'register 84 readback mismatch' in diagnostic
        assert 'expected=800, actual=801' in diagnostic


@pytest.mark.parametrize('arm_before_violation', [False, True])
@pytest.mark.parametrize('retreat_raw', [1314, 1395, 1450, 1451, 1600, 1805])
def test_hardware_loop_travel_off_blocks_arming_on_keeps_torque(monkeypatch, finger, arm_before_violation, retreat_raw):
    import contextlib
    import mujoco.viewer
    from simulation.glove import haptic_demo as demo
    servo, motor = finger
    model, data, aid, gear, lo, hi = build_scene(servo, motor)
    mid = motor.motor_id
    bus = SimulatedBus([mid])
    return_inside_limits = motor.min_position <= retreat_raw <= motor.max_position
    callback = None
    class Viewer:
        cam = SimpleNamespace(lookat=np.zeros(3), distance=0, azimuth=0, elevation=0)
        frames = 0
        def __enter__(self): return self
        def __exit__(self, *exc): pass
        def lock(self): return contextlib.nullcontext()
        def is_running(self): return self.frames < 4
        def sync(self):
            self.frames += 1
            retreat = retreat_raw
            raw = motor.zero_position if self.frames == 1 or (self.frames == 3 and not return_inside_limits) else retreat
            bus.write(mid, 132, 4, raw)
            if (arm_before_violation and self.frames == 1) or (not arm_before_violation and self.frames == 2):
                callback(32)
        def set_texts(self, texts): pass
    viewer = Viewer()
    def launch(*args, **kwargs):
        nonlocal callback
        callback = kwargs['key_callback']
        return viewer
    monkeypatch.setattr(mujoco.viewer, 'launch_passive', launch)
    monkeypatch.setattr(demo.time, 'sleep', lambda _: None)
    with XL330Driver(bus, [motor]) as driver:
        bus.write(mid, 116, 4, motor.zero_position)
        with current_session(driver, motor, 20):
            start = len(bus.writes)
            def exercise():
                demo.run(SimpleNamespace(stiffness=120, max_current_ma=20), servo, motor,
                         model, data, aid, gear, lo, hi, driver)
            exercise()
            if not return_inside_limits:
                assert bus.read(mid, 64, 1) == int(arm_before_violation)
                assert bus.read(mid, 102, 2) == 0
                enables = [v for _, a, _, v in bus.writes[start:] if a == 64 and v == 1]
                assert len(enables) == int(arm_before_violation)  # No torque cycling on return.
            else:
                if not arm_before_violation and (not return_inside_limits or retreat_raw == 1600):
                    assert not any(a == 64 and v == 1 for _, a, _, v in bus.writes[start:])
                else:
                    assert bus.read(mid, 102, 2) == 0
                    assert bus.read(mid, 64, 1) == 1
                    if retreat_raw < motor.zero_position:
                        assert data.ctrl[aid] == lo
        assert bus.read(mid, 64, 1) == 0


def test_default_seventy_ma_keeps_proportional_feedback_and_release():
    wall = VirtualWall()
    assert wall.update(.3, .2, True) == pytest.approx(50)
    assert wall.update(.8, .2, True) == 70
    assert wall.update(.19, .2, True) == 0


@pytest.mark.parametrize('outside_raw', [1290, 1822])
def test_hardware_loop_reaches_cap_resumes_after_outside_and_releases(monkeypatch, finger, outside_raw):
    import contextlib
    import mujoco.viewer
    from simulation.glove import haptic_demo as demo
    servo, motor = finger
    model, data, aid, gear, lo, hi = build_scene(servo, motor)
    proxy = KinematicWall(model, data, aid, gear, lo, hi)
    # Well inside the physical limit, yet enough overtravel for the 70 mA cap.
    pressed = motor.radians_to_position((proxy.boundary + .15) / gear - servo['offset'])
    assert pressed < motor.max_position - 50
    mid = motor.motor_id
    bus = SimulatedBus([mid])
    clock = SimpleNamespace(now=0)
    monkeypatch.setattr(demo, 'time', SimpleNamespace(monotonic=lambda: clock.now, sleep=lambda _: None))
    callback = None
    class Viewer:
        cam = SimpleNamespace(lookat=np.zeros(3), distance=0, azimuth=0, elevation=0)
        frame = 0
        def __enter__(self): return self
        def __exit__(self, *exc): pass
        def lock(self): return contextlib.nullcontext()
        def is_running(self): return self.frame < 54
        def sync(self):
            self.frame += 1
            clock.now += .02
            if self.frame in (26, 27):
                raw = outside_raw
            else:
                raw = pressed if 2 <= self.frame <= 51 else motor.zero_position
            bus.write(mid, 132, 4, raw)
            if self.frame == 1:
                callback(32)
        def set_texts(self, texts): pass
    def launch(*args, **kwargs):
        nonlocal callback
        callback = kwargs['key_callback']
        return Viewer()
    monkeypatch.setattr(mujoco.viewer, 'launch_passive', launch)
    with XL330Driver(bus, [motor]) as driver:
        bus.write(mid, 132, 4, motor.zero_position)
        with current_session(driver, motor, 70):
            before = len(bus.writes)
            demo.run(SimpleNamespace(stiffness=500, max_current_ma=70), servo, motor,
                     model, data, aid, gear, lo, hi, driver, proxy)
            commands = [v - 65536 if v >= 32768 else v
                        for _, address, _, v in bus.writes[before:] if address == 102]
            assert min(commands) == -70
            assert max(commands) == 0
            assert commands[-1] == 0
            first_cap = commands.index(-70)
            zero_after_cap = commands.index(0, first_cap)
            assert -70 in commands[zero_after_cap:]  # Force returns without another Space.
            torque_writes = [v for _, address, _, v in bus.writes[before:] if address == 64]
            assert torque_writes == [1]  # No disarm/re-arm during either excursion.
            assert data.qpos[proxy.qadr] * gear == pytest.approx(lo)
            assert proxy.gap(data.qpos) > 0
        assert bus.read(mid, 64, 1) == 0
