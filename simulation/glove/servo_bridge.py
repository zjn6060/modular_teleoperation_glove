#!/usr/bin/env python3
"""Bridge the real glove servos and the MuJoCo preview.

real2sim: torque off; servo angles drive the simulation.
sim2real: slowly home the real glove to EF-horizontal, then follow the sliders.
Abduction joints have no hardware and stay on the sidebar in both modes.
"""
from pathlib import Path
from contextlib import ExitStack, contextmanager
import argparse
import math
import sys
import time
import numpy as np
import mujoco

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from utils.glove_servos import CONFIG_PATH, load_config, motor_configs, open_driver
from driver import OperatingMode, Register
from driver.errors import SafetyError
if __package__:
    from .reference_pose import REFERENCE_POSE, reset_reference_pose
    from .view_glove import run
else:
    from reference_pose import REFERENCE_POSE, reset_reference_pose
    from view_glove import run


MAX_SPEED = 6.0  # rad/s, identical for flexion and extension
SIM2REAL_CURRENT_MA = 70  # one mode-5 current cap for both motion directions
HOME_SPEED = 0.3  # rad/s; move the physical glove gradually on startup
HOME_TIMEOUT = 15.0  # wall-clock seconds, including any mechanical lag
MAX_DT = 0.05  # do not catch up with a large jump after a delayed viewer frame
POSITION_TOLERANCE = 2 * 2 * np.pi / 4096  # calibration-reference consistency only
HOME_TOLERANCE_TICKS = 13  # about 0.020 rad / 1.14 degrees of physical tracking error
HOME_STABLE_TIME = 0.2  # all fingers must stay within the arrival tolerance


def check_position_modes(driver):
    for motor_id in driver.motor_ids:
        mode = driver.read_register(motor_id, Register.OPERATING_MODE)
        if mode not in (OperatingMode.POSITION, OperatingMode.CURRENT_BASED_POSITION):
            raise SafetyError(f"ID {motor_id}: sim2real requires position mode 3 or 5 (got {mode}).")


def report_settings(driver):
    """Read existing settings; never increase the available current implicitly."""
    print('ID | MODE | GoalCurrent(mA) | CurrentLimit(mA) | Kp(raw) | Ki(raw) | Kd(raw) | Torque(0/1) | DriveMode | ProfileVel(raw) | ProfileAccel(raw)')
    for motor_id in driver.motor_ids:
        values = [driver.read_register(motor_id, reg) for reg in (
            Register.OPERATING_MODE, Register.GOAL_CURRENT, Register.CURRENT_LIMIT,
            Register.POSITION_P_GAIN, Register.POSITION_I_GAIN, Register.POSITION_D_GAIN,
            Register.TORQUE_ENABLE,
            Register.DRIVE_MODE, Register.PROFILE_VELOCITY, Register.PROFILE_ACCELERATION,
        )]
        if values[1] >= 32768:
            values[1] -= 65536
        print(f"{motor_id} | " + ' | '.join(map(str, values)))


@contextmanager
def temporary_current_test(driver, motor_id=None, current_ma=None, *, label='CURRENT TEST'):
    """Temporarily change one mode-5 motor's current cap, restoring it torque-off.

    Only raise Current Limit when necessary to allow the requested Goal Current.
    Other motors and all PID gains keep their existing settings.
    """
    if current_ma is None:
        yield
        return
    if motor_id not in driver.motor_ids:
        raise ValueError(f'ID {motor_id} is not a configured glove motor.')
    if not isinstance(current_ma, int) or not 1 <= current_ma <= 1750:
        raise ValueError('Test current must be an integer in 1..1750 mA.')
    if driver.read_register(motor_id, Register.OPERATING_MODE) != OperatingMode.CURRENT_BASED_POSITION:
        raise SafetyError(f'ID {motor_id}: a current-limit test requires mode 5.')
    if driver.read_register(motor_id, Register.TORQUE_ENABLE):
        raise SafetyError(f'ID {motor_id}: disable torque before testing a different current setting.')
    old_limit = driver.read_register(motor_id, Register.CURRENT_LIMIT)
    old_goal = driver.read_register(motor_id, Register.GOAL_CURRENT)
    test_limit = max(old_limit, current_ma)

    def write_verified(register, value):
        driver.write_register(motor_id, register, value)
        if driver.read_register(motor_id, register) != value:
            raise SafetyError(f'ID {motor_id}: {register.name} readback does not match {value}.')

    try:
        # The EEPROM limit must permit the new RAM goal before it is written.
        if test_limit != old_limit:
            write_verified(Register.CURRENT_LIMIT, test_limit)
        if current_ma != old_goal:
            write_verified(Register.GOAL_CURRENT, current_ma)
        print(f'{label} ID {motor_id}: Goal Current {old_goal} -> {current_ma} mA; '
              f'Current Limit {old_limit} -> {test_limit} mA. PID gains unchanged. '
              'Settings will be restored on normal exit or Ctrl+C.')
        yield
    finally:
        active_error = sys.exc_info()[1]
        try:
            failures = driver.disable_torque()
            if failures:
                raise SafetyError('Cannot restore current settings until torque is off: ' + '; '.join(failures))
            # Restore the RAM goal first, before lowering the EEPROM ceiling.
            if driver.read_register(motor_id, Register.GOAL_CURRENT) != old_goal:
                write_verified(Register.GOAL_CURRENT, old_goal)
            if driver.read_register(motor_id, Register.CURRENT_LIMIT) != old_limit:
                write_verified(Register.CURRENT_LIMIT, old_limit)
            print(f'{label} ID {motor_id}: restored Goal Current={old_goal} mA, '
                  f'Current Limit={old_limit} mA; torque OFF.')
        except Exception as cleanup_error:
            if active_error is None:
                raise
            active_error.add_note(f'Current-test restoration failed: {cleanup_error}')
            print(f'Current-test restoration failed: {cleanup_error}', file=sys.stderr)


@contextmanager
def uniform_current_limit(driver, current_ma):
    """Apply one temporary current cap to every configured mode-5 motor.

    Preflight all modes, release torque before EEPROM writes, and roll back
    already-configured motors if any later setup or viewer operation fails.
    """
    if not isinstance(current_ma, int) or not 1 <= current_ma <= 1750:
        raise ValueError('Uniform current must be an integer in 1..1750 mA.')
    for motor_id in driver.motor_ids:
        if driver.read_register(motor_id, Register.OPERATING_MODE) != OperatingMode.CURRENT_BASED_POSITION:
            raise SafetyError(f'ID {motor_id}: uniform current limiting requires mode 5.')
    failures = driver.disable_torque()
    if failures:
        raise SafetyError('Could not release torque before applying current limits: ' + '; '.join(failures))
    with ExitStack() as stack:
        for motor_id in driver.motor_ids:
            stack.enter_context(temporary_current_test(driver, motor_id, current_ma, label='UNIFORM CURRENT'))
        print(f'sim2real: all configured motors use {current_ma} mA in both directions. PID gains unchanged.')
        yield


def make_bridge(driver, config, motors, mode, *, max_speed=MAX_SPEED,
                home_speed=HOME_SPEED, home_timeout=HOME_TIMEOUT,
                home_tolerance_ticks=HOME_TOLERANCE_TICKS, diagnostics=False, clock=None):
    """Build a bridge with timed, symmetric target slew and feedback-checked homing."""
    if mode not in ('real2sim', 'sim2real'):
        raise ValueError(f'Unknown bridge mode: {mode}')
    for name, value in (('max_speed', max_speed), ('home_speed', home_speed),
                        ('home_timeout', home_timeout)):
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f'{name} must be finite and positive.')
    if not isinstance(home_tolerance_ticks, int) or home_tolerance_ticks < 1:
        raise ValueError('home_tolerance_ticks must be a positive integer.')
    home_tolerance = home_tolerance_ticks * 2 * np.pi / 4096
    clock = clock or time.monotonic

    def setup(model, data):
        if len(config['servos']) != len(motors):
            raise ValueError('Each configured servo must have one motor configuration.')
        if mode == 'sim2real':
            if config.get('reference_pose') != REFERENCE_POSE:
                raise SafetyError('Recalibrate with the EF-horizontal reference before sim2real.')
            reset_reference_pose(model, data)
        servos = {}
        home = {}
        home_controls = {}
        for servo, motor in zip(config['servos'], motors):
            actuator = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, servo['joint'])
            if actuator < 0:
                raise ValueError(f"Actuator {servo['joint']} is not in the model.")
            gear = float(model.actuator_gear[actuator, 0])
            if gear not in (-1.0, 1.0):
                raise ValueError(f"Actuator {servo['joint']} must have joint gear +1 or -1.")
            lo, hi = sorted(motor.position_to_radians(p)
                            for p in (motor.min_position, motor.max_position))
            offset = servo.get('offset', 0.0)
            calibrated_controls = sorted(gear * (angle + offset) for angle in (lo, hi))
            control_lo = max(model.actuator_ctrlrange[actuator, 0], calibrated_controls[0])
            control_hi = min(model.actuator_ctrlrange[actuator, 1], calibrated_controls[1])
            if control_lo >= control_hi:
                raise SafetyError(f"ID {motor.motor_id}: calibration and model have no usable motion range.")
            model.actuator_ctrlrange[actuator] = (control_lo, control_hi)
            lo, hi = sorted(control / gear - offset for control in (control_lo, control_hi))
            servos[motor.motor_id] = (actuator, offset, lo, hi, gear)
            if mode == 'sim2real':
                reference = float(data.ctrl[actuator])
                angle = reference / gear - offset
                if (abs(angle) > POSITION_TOLERANCE
                        or not control_lo - 1e-9 <= reference <= control_hi + 1e-9):
                    raise SafetyError(f"ID {motor.motor_id}: EF-horizontal reference does not match calibration; recalibrate.")
                home[motor.motor_id] = angle
                home_controls[actuator] = reference

        last = {}
        homing = mode == 'sim2real'
        if homing:
            # Validate everything before torque-on. The driver first holds the real
            # positions; only subsequent frames ramp those holds toward horizontal.
            check_position_modes(driver)
            holds = driver.enable_torque()
            last = {motor.motor_id: motor.position_to_radians(holds[motor.motor_id])
                    for motor in motors}
            print(f'sim2real: torque ON; returning real fingers to EF-horizontal at {home_speed:g} rad/s. '
                  f'Sliders unlock after all fingers stay within +/-{home_tolerance_ticks} ticks '
                  f'({home_tolerance:.3f} rad) for {HOME_STABLE_TIME:g} s; close the window to release.')
        started = previous = clock()
        next_report = started
        next_home_report = started
        stable_since = None

        def on_frame():
            nonlocal previous, homing, next_report, next_home_report, stable_since
            now = clock()
            dt = max(0.0, min(now - previous, MAX_DT))
            previous = now
            if mode == 'real2sim':
                for motor_id, angle in driver.read_angles().items():
                    actuator, offset, lo, hi, gear = servos[motor_id]
                    data.ctrl[actuator] = np.clip(gear * (angle + offset),
                                                  *model.actuator_ctrlrange[actuator])
                return

            if homing:
                # Keep the model at its reference, including when a slider is
                # dragged or reset while the real fingers are still homing.
                for actuator, control in home_controls.items():
                    data.ctrl[actuator] = control
                actual = driver.read_angles()
                arrived = all(abs(actual[i] - home[i]) <= home_tolerance + 1e-12 for i in home)
                commanded_home = all(abs(last[i] - home[i]) < 1e-9 for i in home)
                if arrived and commanded_home:
                    if stable_since is None:
                        stable_since = now
                    if now - stable_since >= HOME_STABLE_TIME:
                        homing = False
                        print(f'sim2real: EF-horizontal reached within +/-{home_tolerance_ticks} ticks. Sliders ready.')
                else:
                    stable_since = None
                if homing:
                    errors = ', '.join(f'ID {i}: {actual[i] - home[i]:+.3f} rad '
                                       f'({round((actual[i] - home[i]) * 4096 / (2 * np.pi)):+d} ticks)'
                                       for i in home)
                    if now - started >= home_timeout:
                        raise SafetyError(f'EF-horizontal homing timed out ({errors}); sliders stayed locked. '
                                          f'Arrival tolerance is +/-{home_tolerance_ticks} ticks. '
                                          'Check current settings and linkage resistance.')
                    if now >= next_home_report:
                        print(f'HOMING: sliders locked; arrival tolerance +/-{home_tolerance_ticks} ticks; {errors}')
                        next_home_report = now + 1.0
                targets = home
                speed = min(home_speed, max_speed)
            else:
                targets = {}
                for motor_id, (actuator, offset, lo, hi, gear) in servos.items():
                    data.ctrl[actuator] = np.clip(data.ctrl[actuator],
                                                  *model.actuator_ctrlrange[actuator])
                    targets[motor_id] = float(np.clip(data.ctrl[actuator] / gear - offset, lo, hi))
                speed = max_speed

            commands = {i: last[i] + float(np.clip(targets[i] - last[i], -speed * dt, speed * dt))
                        for i in targets}
            # During homing the starting pose may be above the new sidebar floor.
            # Ramp from that pose; the driver's calibrated raw limits still apply.
            driver.command_angles(commands, clamp=False)
            last.update(commands)
            if diagnostics and now >= next_report:
                for i, state in driver.read_telemetry().items():
                    if state.hardware_error:
                        raise SafetyError(f'ID {i}: hardware error {state.hardware_error:#04x}.')
                    phase = 'HOMING' if homing else 'READY'
                    torque = 'ON' if state.torque_enabled else 'OFF'
                    print(f'{phase} ID {i}: target={commands[i]:+.3f} rad, actual={state.position_rad:+.3f} rad, '
                          f'current={state.current_ma:+.0f} mA, voltage={state.input_voltage_v:.1f} V, torque={torque}')
                next_report = now + 1.0

        return on_frame
    return setup


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('mode', choices=('real2sim', 'sim2real'))
    parser.add_argument('--config', type=Path, default=CONFIG_PATH)
    parser.add_argument('--device', help='default: value in the config')
    parser.add_argument('--baud', type=int, help='default: value in the config')
    parser.add_argument('--simulate', action='store_true')
    parser.add_argument('--settings-only', action='store_true',
                        help='read current/gain settings and exit; no register writes, torque enable, or viewer')
    parser.add_argument('--max-speed', type=float, default=MAX_SPEED, help='symmetric target slew limit in rad/s')
    parser.add_argument('--home-speed', type=float, default=HOME_SPEED, help='startup homing target slew in rad/s')
    parser.add_argument('--home-timeout', type=float, default=HOME_TIMEOUT, help='homing timeout in seconds')
    parser.add_argument('--home-tolerance-ticks', type=int, default=HOME_TOLERANCE_TICKS,
                        help='physical homing arrival tolerance in encoder ticks (default: 13, about 1.14 degrees)')
    parser.add_argument('--diagnostics', action='store_true', help='print position/current/voltage feedback once per second')
    parser.add_argument('--current-ma', type=int,
                        help=f'uniform mode-5 Goal Current for all motors (default: {SIM2REAL_CURRENT_MA} mA); restores on exit')
    parser.add_argument('--test-id', type=int, help='single motor ID for an explicit temporary current test')
    parser.add_argument('--test-current-ma', type=int,
                        help='temporary mode-5 Goal Current in mA; restores original goal/limit on exit')
    args = parser.parse_args(argv)
    if args.current_ma is not None:
        if args.mode != 'sim2real' or args.settings_only or args.test_current_ma is not None:
            parser.error('--current-ma requires sim2real and cannot be combined with --settings-only or a single-ID test.')
        if not 1 <= args.current_ma <= 1750:
            parser.error('--current-ma must be in 1..1750.')
    if (args.test_id is None) != (args.test_current_ma is None):
        parser.error('--test-id and --test-current-ma must be supplied together.')
    if args.test_current_ma is not None:
        if args.mode != 'sim2real' or args.settings_only:
            parser.error('A current test requires sim2real and cannot be combined with --settings-only.')
        if not 1 <= args.test_current_ma <= 1750:
            parser.error('--test-current-ma must be in 1..1750.')

    config = load_config(args.config)
    motors = motor_configs(config)
    driver = open_driver(motors, args.device or config['device'],
                         args.baud or config['baudrate'], args.simulate)
    hook = make_bridge(driver, config, motors, args.mode, max_speed=args.max_speed,
                       home_speed=args.home_speed, home_timeout=args.home_timeout,
                       home_tolerance_ticks=args.home_tolerance_ticks,
                       diagnostics=args.diagnostics)
    with driver:
        if args.settings_only:
            print('Read-only settings report; no register writes or torque changes.')
            report_settings(driver)
            return
        if args.mode == 'sim2real':
            check_position_modes(driver)
        else:
            failures = driver.disable_torque()
            if failures:
                raise SystemExit('Could not disable torque: ' + '; '.join(failures))
            print('real2sim: torque OFF. Move the real fingers; abduction stays on the sidebar.')
        if args.mode == 'sim2real' and args.test_current_ma is None:
            current_context = uniform_current_limit(
                driver, args.current_ma if args.current_ma is not None else SIM2REAL_CURRENT_MA)
        else:
            current_context = temporary_current_test(driver, args.test_id, args.test_current_ma)
        with current_context:
            if args.mode == 'sim2real':
                report_settings(driver)
            run(hook)


if __name__ == '__main__':
    main()
