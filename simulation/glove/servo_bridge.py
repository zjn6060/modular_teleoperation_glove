#!/usr/bin/env python3
"""Bridge the real glove servos and the MuJoCo preview.

real2sim: torque off; each servo angle drives its horn joint and the linkage follows.
sim2real: torque on; each horn-joint slider target is sent to its servo directly
          (clamped to the calibrated range), so the servo does not wait on the sim.
Abduction joints have no hardware and stay on the sidebar in both modes.
"""
from pathlib import Path
import argparse
import sys
import numpy as np
import mujoco

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from utils.glove_servos import CONFIG_PATH, load_config, motor_configs, open_driver
from driver.errors import SafetyError
if __package__:
    from .view_glove import run
else:
    from view_glove import run


SYNC_FRAMES = 60      # sim2real: follow the real glove for 1 s before commanding it
MAX_STEP = 6.0 / 60   # sim2real: servo target rate limit, rad per frame
STARTUP_TOLERANCE = 2 * 2 * np.pi / 4096  # two encoder ticks of measurement tolerance


def make_bridge(driver, config, motors, mode):
    """Return a view_glove hook that exchanges horn-joint angles with the servos."""
    def setup(model, data):
        servos = {}
        for servo, motor in zip(config['servos'], motors):
            actuator = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, servo['joint'])
            if actuator < 0:
                raise ValueError(f"Actuator {servo['joint']} is not in the model.")
            gear = float(model.actuator_gear[actuator, 0])
            if gear not in (-1.0, 1.0):
                raise ValueError(f"Actuator {servo['joint']} must have joint gear +1 or -1.")
            lo, hi = sorted(motor.position_to_radians(p)
                            for p in (motor.min_position, motor.max_position))
            offset = servo.get('offset', 0.0)  # sim joint angle at the servo's zero
            calibrated_controls = sorted(gear * (angle + offset) for angle in (lo, hi))
            control_lo = max(model.actuator_ctrlrange[actuator, 0], calibrated_controls[0])
            control_hi = min(model.actuator_ctrlrange[actuator, 1], calibrated_controls[1])
            if control_lo >= control_hi:
                raise SafetyError(f"ID {motor.motor_id}: calibration and model have no usable motion range.")
            # Construct the sidebar with the same bounds used for real commands.
            model.actuator_ctrlrange[actuator] = (control_lo, control_hi)
            lo, hi = sorted(control / gear - offset for control in (control_lo, control_hi))
            servos[motor.motor_id] = (actuator, offset, lo, hi, gear)
        frame = 0
        last = {}

        def on_frame():
            nonlocal frame
            if mode == 'real2sim' or frame < SYNC_FRAMES:
                for motor_id, angle in driver.read_angles().items():
                    actuator, offset, lo, hi, gear = servos[motor_id]
                    if mode == 'sim2real' and not lo - STARTUP_TOLERANCE <= angle <= hi + STARTUP_TOLERANCE:
                        raise SafetyError(
                            f"ID {motor_id}: current pose is outside the permitted horizontal-to-flexion range; "
                            "close the viewer, return the finger to the horizontal calibration pose and retry."
                        )
                    data.ctrl[actuator] = np.clip(gear * (angle + offset),
                                                  *model.actuator_ctrlrange[actuator])
                    last[motor_id] = angle
            else:
                commands = {}
                for motor_id, (actuator, offset, lo, hi, gear) in servos.items():
                    data.ctrl[actuator] = np.clip(data.ctrl[actuator],
                                                  *model.actuator_ctrlrange[actuator])
                    joint_target = data.ctrl[actuator] / gear
                    step = np.clip(joint_target - offset - last[motor_id], -MAX_STEP, MAX_STEP)
                    commands[motor_id] = float(np.clip(last[motor_id] + step, lo, hi))
                driver.command_angles(commands, clamp=True)
                last.update(commands)
            frame += 1

        return on_frame
    return setup


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('mode', choices=('real2sim', 'sim2real'))
    parser.add_argument('--config', type=Path, default=CONFIG_PATH)
    parser.add_argument('--device', help='default: value in the config')
    parser.add_argument('--baud', type=int, help='default: value in the config')
    parser.add_argument('--simulate', action='store_true')
    args = parser.parse_args()

    config = load_config(args.config)
    motors = motor_configs(config)
    driver = open_driver(motors, args.device or config['device'],
                         args.baud or config['baudrate'], args.simulate)
    with driver:
        if args.mode == 'sim2real':
            driver.enable_torque()
            print('sim2real: torque ON. Drag the horn-joint sliders '
                  f"({', '.join(s['joint'] for s in config['servos'])}); close the window to release.")
        else:
            failures = driver.disable_torque()
            if failures:
                raise SystemExit('Could not disable torque: ' + '; '.join(failures))
            print('real2sim: torque OFF. Move the real fingers; abduction stays on the sidebar.')
        run(make_bridge(driver, config, motors, args.mode))


if __name__ == '__main__':
    main()
