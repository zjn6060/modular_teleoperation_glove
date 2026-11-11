"""Read-only baseline reports must not actuate or change existing motor settings."""

import pytest

from driver import MotorConfig, Register, SafetyError, SimulatedBus, XL330Driver
from simulation.glove import servo_bridge
from utils.glove_servos import load_config, motor_configs


@pytest.mark.parametrize('mode', (0, 5))
@pytest.mark.parametrize('torque', (0, 1))
def test_settings_only_reads_actual_values_without_viewer_or_register_writes(monkeypatch, capsys, mode, torque):
    motors = motor_configs(load_config())
    bus = SimulatedBus([m.motor_id for m in motors])
    driver = XL330Driver(bus, motors)
    for motor in motors:
        bus.registers[motor.motor_id].update({
            Register.OPERATING_MODE: mode,
            Register.TORQUE_ENABLE: torque,
            Register.CURRENT_LIMIT: 20,
            Register.GOAL_CURRENT: 15,
            Register.POSITION_P_GAIN: 456,
            Register.POSITION_I_GAIN: 7,
            Register.POSITION_D_GAIN: 23,
        })
    monkeypatch.setattr(servo_bridge, 'open_driver', lambda *args: driver)

    def forbidden_viewer(*args, **kwargs):
        pytest.fail('settings-only must not start a viewer or enable torque')

    monkeypatch.setattr(servo_bridge, 'run', forbidden_viewer)
    servo_bridge.main(['sim2real', '--settings-only'])
    lines = capsys.readouterr().out.splitlines()
    header = next(line for line in lines if line.startswith('ID |'))
    columns = [item.strip() for item in header.split('|')]
    rows = [line for line in lines if line.split('|')[0].strip().isdigit()]
    assert [int(row.split('|')[0]) for row in rows] == [2, 3, 4, 5]
    for row in rows:
        values = dict(zip(columns, [item.strip() for item in row.split('|')]))
        assert values['GoalCurrent(mA)'] == '15'
        assert values['CurrentLimit(mA)'] == '20'
        assert values['Kp(raw)'] == '456'
        assert values['Ki(raw)'] == '7'
        assert values['Kd(raw)'] == '23'
        assert values['Torque(0/1)'] == str(torque)
    assert bus.writes == []
    assert not bus.is_open
    assert all(bus.registers[m.motor_id][Register.TORQUE_ENABLE] == torque for m in motors)


def test_settings_report_decodes_negative_goal_current_without_writes(capsys):
    motors = motor_configs(load_config())[:1]
    bus = SimulatedBus([motors[0].motor_id])
    bus.registers[motors[0].motor_id][Register.GOAL_CURRENT] = 65536 - 10
    with XL330Driver(bus, motors) as driver:
        servo_bridge.report_settings(driver)
    assert '| -10 |' in capsys.readouterr().out
    assert bus.writes == []


def current_test_driver():
    motors = [MotorConfig(i) for i in (2, 3)]
    bus = SimulatedBus((2, 3))
    for i in (2, 3):
        bus.registers[i].update({Register.OPERATING_MODE: 5, Register.CURRENT_LIMIT: 20,
                                 Register.GOAL_CURRENT: 20, Register.POSITION_P_GAIN: 250,
                                 Register.POSITION_D_GAIN: 600})
    return XL330Driver(bus, motors), bus


@pytest.mark.parametrize('stop', ('close', 'exception', 'ctrl_c'))
def test_current_test_restores_both_limits_after_torque_off_and_changes_one_id(stop):
    driver, bus = current_test_driver()

    def session():
        with driver:
            with servo_bridge.temporary_current_test(driver, 2, 25):
                assert bus.registers[2][Register.CURRENT_LIMIT] == 25
                assert bus.registers[2][Register.GOAL_CURRENT] == 25
                driver.enable_torque()
                if stop == 'exception':
                    raise RuntimeError('viewer failed')
                if stop == 'ctrl_c':
                    raise KeyboardInterrupt()

    if stop == 'exception':
        with pytest.raises(RuntimeError, match='viewer failed'):
            session()
    elif stop == 'ctrl_c':
        with pytest.raises(KeyboardInterrupt):
            session()
    else:
        session()
    current_writes = [(i, address, value) for i, address, _, value in bus.writes
                      if address in (Register.CURRENT_LIMIT, Register.GOAL_CURRENT)]
    assert current_writes == [(2, 38, 25), (2, 102, 25), (2, 102, 20), (2, 38, 20)]
    restore_index = next(i for i, row in enumerate(bus.writes) if row[:2] == (2, 102) and row[3] == 20)
    for motor_id in (2, 3):
        assert (motor_id, Register.TORQUE_ENABLE, 1, 0) in bus.writes[:restore_index]
        assert bus.registers[motor_id][Register.GOAL_CURRENT] == 20
        assert bus.registers[motor_id][Register.CURRENT_LIMIT] == 20
        assert bus.registers[motor_id][Register.POSITION_P_GAIN] == 250
        assert bus.registers[motor_id][Register.POSITION_D_GAIN] == 600
        assert bus.registers[motor_id][Register.TORQUE_ENABLE] == 0


def test_current_test_does_not_rewrite_eeprom_when_existing_ceiling_is_sufficient():
    driver, bus = current_test_driver()
    bus.registers[2][Register.CURRENT_LIMIT] = 100
    with driver:
        with servo_bridge.temporary_current_test(driver, 2, 25):
            assert bus.registers[2][Register.CURRENT_LIMIT] == 100
        assert bus.registers[2][Register.CURRENT_LIMIT] == 100
        assert not any(address == Register.CURRENT_LIMIT for _, address, _, _ in bus.writes)


@pytest.mark.parametrize('fault', ('mode', 'torque', 'id', 'range'))
def test_invalid_current_test_does_not_write_any_register(fault):
    driver, bus = current_test_driver()
    motor_id, current = 2, 25
    if fault == 'mode':
        bus.registers[2][Register.OPERATING_MODE] = 3
    elif fault == 'torque':
        bus.registers[2][Register.TORQUE_ENABLE] = 1
    elif fault == 'id':
        motor_id = 99
    else:
        current = 1751
    with driver:
        with pytest.raises((SafetyError, ValueError)):
            with servo_bridge.temporary_current_test(driver, motor_id, current):
                pytest.fail('Invalid settings must be rejected before entering the test.')
        assert bus.writes == []


def test_partial_current_write_failure_restores_original_eeprom_limit(monkeypatch):
    driver, bus = current_test_driver()
    write = bus.write

    def failing_write(i, address, size, value):
        if address == Register.GOAL_CURRENT and value == 25:
            raise RuntimeError('goal write failed')
        write(i, address, size, value)

    monkeypatch.setattr(bus, 'write', failing_write)
    with driver:
        with pytest.raises(RuntimeError, match='goal write failed'):
            with servo_bridge.temporary_current_test(driver, 2, 25):
                pytest.fail('The current write should fail before starting the viewer.')
        assert bus.registers[2][Register.CURRENT_LIMIT] == 20
        assert bus.registers[2][Register.GOAL_CURRENT] == 20


@pytest.mark.parametrize('options', (
    ['--test-id', '2'],
    ['--test-current-ma', '25'],
    ['--settings-only', '--test-id', '2', '--test-current-ma', '25'],
    ['--test-id', '2', '--test-current-ma', '0'],
))
def test_invalid_current_cli_options_are_rejected_before_opening_serial(monkeypatch, options):
    def forbidden_open(*args, **kwargs):
        pytest.fail('Invalid CLI options must be rejected before opening serial.')
    monkeypatch.setattr(servo_bridge, 'open_driver', forbidden_open)
    with pytest.raises(SystemExit) as exc:
        servo_bridge.main(['sim2real', *options])
    assert exc.value.code == 2


@pytest.mark.parametrize('stop', ('close', 'exception', 'ctrl_c'))
def test_uniform_current_sets_all_motors_and_restores_after_torque_off(stop):
    driver, bus = current_test_driver()
    bus.registers[3][Register.TORQUE_ENABLE] = 1  # another app left torque on

    def session():
        with driver:
            with servo_bridge.uniform_current_limit(driver, 30):
                assert all(bus.registers[i][Register.GOAL_CURRENT] == 30
                           and bus.registers[i][Register.CURRENT_LIMIT] == 30 for i in (2, 3))
                assert all(bus.registers[i][Register.POSITION_P_GAIN] == 250
                           and bus.registers[i][Register.POSITION_D_GAIN] == 600 for i in (2, 3))
                driver.enable_torque()
                if stop == 'exception':
                    raise RuntimeError('viewer failed')
                if stop == 'ctrl_c':
                    raise KeyboardInterrupt()

    if stop == 'exception':
        with pytest.raises(RuntimeError, match='viewer failed'):
            session()
    elif stop == 'ctrl_c':
        with pytest.raises(KeyboardInterrupt):
            session()
    else:
        session()
    first_setting = next(n for n, (_, address, _, _) in enumerate(bus.writes)
                         if address == Register.CURRENT_LIMIT)
    for i in (2, 3):
        assert (i, Register.TORQUE_ENABLE, 1, 0) in bus.writes[:first_setting]
        assert bus.registers[i][Register.CURRENT_LIMIT] == 20
        assert bus.registers[i][Register.GOAL_CURRENT] == 20
        assert bus.registers[i][Register.TORQUE_ENABLE] == 0


def test_uniform_current_preflights_all_modes_before_writing():
    driver, bus = current_test_driver()
    bus.registers[3][Register.OPERATING_MODE] = 3
    with driver:
        with pytest.raises(SafetyError, match='ID 3: uniform current limiting requires mode 5'):
            with servo_bridge.uniform_current_limit(driver, 30):
                pytest.fail('A mode mismatch must abort before any settings are applied.')
        assert bus.writes == []


def test_uniform_partial_write_failure_rolls_back_previously_configured_motors(monkeypatch):
    driver, bus = current_test_driver()
    write = bus.write

    def failing_write(i, address, size, value):
        if i == 3 and address == Register.GOAL_CURRENT and value == 30:
            raise RuntimeError('second motor write failed')
        write(i, address, size, value)

    monkeypatch.setattr(bus, 'write', failing_write)
    with driver:
        with pytest.raises(RuntimeError, match='second motor write failed'):
            with servo_bridge.uniform_current_limit(driver, 30):
                pytest.fail('The failed setup must not start the viewer.')
        assert all(bus.registers[i][Register.CURRENT_LIMIT] == 20
                   and bus.registers[i][Register.GOAL_CURRENT] == 20 for i in (2, 3))


@pytest.mark.parametrize('target', (None, 20))
def test_sim2real_main_applies_uniform_current_without_changing_gains(monkeypatch, target):
    motors = motor_configs(load_config())
    bus = SimulatedBus([m.motor_id for m in motors])
    driver = XL330Driver(bus, motors)
    for motor in motors:
        bus.registers[motor.motor_id].update({Register.OPERATING_MODE: 5,
                                            Register.CURRENT_LIMIT: 20, Register.GOAL_CURRENT: 20,
                                            Register.POSITION_P_GAIN: 250, Register.POSITION_D_GAIN: 600,
                                            Register.PRESENT_POSITION: motor.zero_position})
    monkeypatch.setattr(servo_bridge, 'open_driver', lambda *args: driver)
    expected = 70 if target is None else target
    seen = []

    def fake_viewer(hook):
        import mujoco
        from simulation.glove.reference_pose import SCENE_PATH, reset_reference_pose
        model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
        data = mujoco.MjData(model)
        reset_reference_pose(model, data)
        hook(model, data)
        for motor in motors:
            registers = bus.registers[motor.motor_id]
            assert registers[Register.GOAL_CURRENT] == expected
            assert registers[Register.CURRENT_LIMIT] == expected
            assert registers[Register.TORQUE_ENABLE] == 1
            assert registers[Register.POSITION_P_GAIN] == 250
            assert registers[Register.POSITION_D_GAIN] == 600
        seen.append(True)

    monkeypatch.setattr(servo_bridge, 'run', fake_viewer)
    options = [] if target is None else ['--current-ma', str(target)]
    servo_bridge.main(['sim2real', *options])
    assert seen == [True]
    assert all(bus.registers[m.motor_id][Register.GOAL_CURRENT] == 20
               and bus.registers[m.motor_id][Register.CURRENT_LIMIT] == 20
               and bus.registers[m.motor_id][Register.TORQUE_ENABLE] == 0 for m in motors)


@pytest.mark.parametrize('options', (
    ['--current-ma', '0'],
    ['--current-ma', '1751'],
    ['--settings-only', '--current-ma', '30'],
    ['--current-ma', '30', '--test-id', '2', '--test-current-ma', '25'],
))
def test_invalid_uniform_current_cli_options_are_rejected_before_serial(monkeypatch, options):
    def forbidden_open(*args, **kwargs):
        pytest.fail('Invalid options must not open the serial port.')
    monkeypatch.setattr(servo_bridge, 'open_driver', forbidden_open)
    with pytest.raises(SystemExit) as exc:
        servo_bridge.main(['sim2real', *options])
    assert exc.value.code == 2
