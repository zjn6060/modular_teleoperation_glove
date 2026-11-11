#!/usr/bin/env python3
"""Single-index-finger virtual wall. Default is a hardware-free preview."""
from contextlib import contextmanager
from pathlib import Path
import argparse
import faulthandler
import math
import os
import sys
import time
import traceback
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from driver import OperatingMode, Register
from driver.errors import SafetyError
from utils.glove_servos import CONFIG_PATH, load_config, motor_configs, open_driver
from simulation.glove.reference_pose import reset_reference_pose

HERE = Path(__file__).resolve().parent
DEFAULT_CURRENT_MA = 70
DEFAULT_STIFFNESS = 500.0
# Persistent tuning and current settings reset/owned by this demo. Goal Position
# (116) is a stale motion command, not a tuning parameter: never replay it on exit.
# The position bridge initializes its own hold target before enabling torque.
RESTORE = ((36, 2), (38, 2), (76, 2), (78, 2), (80, 2), (82, 2),
           (84, 2), (88, 2), (90, 2), (100, 2), (102, 2), (104, 4),
           (108, 4), (112, 4))


def build_scene(servo, motor, wall_fraction=0.65):
    """Build an isolated scene; only the green proxy and orange box collide."""
    root = ET.parse(HERE / 'robot.xml').getroot()
    root.find('compiler').set('meshdir', str(HERE / 'assets'))
    for geom in root.iter('geom'):
        geom.set('contype', '0')
        geom.set('conaffinity', '0')
    root.find('option/flag').set('contact', 'enable')
    scene = ET.parse(HERE / 'scene.xml').getroot()
    root.append(scene.find('keyframe'))
    root.append(scene.find('visual'))
    body = root.find(".//body[@name='short_fgh']")
    ET.SubElement(body, 'geom', name='haptic_tip', type='sphere', size='0.004',
                  pos='0.018271096 0.016378935 -0.00125', mass='0',
                  contype='1', conaffinity='2', rgba='0.1 0.9 0.3 1', group='0')
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
    data = mujoco.MjData(model)
    reset_reference_pose(model, data)
    aid = model.actuator(servo['joint']).id
    gear = float(model.actuator_gear[aid, 0])
    if gear not in (-1, 1):
        raise SafetyError('Expected a unit signed actuator gear.')
    physical = sorted(gear * (motor.position_to_radians(p) + servo['offset'])
                      for p in (motor.min_position, motor.max_position))
    lo = max(float(model.actuator_ctrlrange[aid, 0]), physical[0])
    hi = min(float(model.actuator_ctrlrange[aid, 1]), physical[1])
    if hi - lo < 0.1:
        raise SafetyError('Insufficient calibrated flexion range.')
    data.ctrl[aid] = lo + wall_fraction * (hi - lo)
    for _ in range(2500):
        mujoco.mj_step(model, data)
    center = data.geom('haptic_tip').xpos.copy()
    ET.SubElement(root.find('worldbody'), 'geom', name='haptic_box', type='box',
                  pos=' '.join(map(str, center)), size='0.006 0.006 0.006',
                  contype='2', conaffinity='1', rgba='1 0.55 0.1 0.7', group='0')
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
    model.actuator_ctrlrange[aid] = [lo, hi]
    data = mujoco.MjData(model)
    reset_reference_pose(model, data)
    if data.ncon:
        raise SafetyError('Box overlaps horizontal reference; increase --wall-fraction.')
    return model, data, aid, gear, lo, hi


class KinematicWall:
    """One-DOF geometric contact proxy for a fixed box, not force simulation.

    Solve the four passive index joints from the closed-link constraints. Locate
    the FIRST sphere/box surface intersection along flexion before hardware opens.
    Real input and preview ctrl remain unrestricted by this virtual wall; only the
    rendered pose is clamped. This avoids solver softness and discrete tunnelling.
    """
    def __init__(self, model, data, aid, gear, lo, hi):
        self.model, self.aid, self.gear = model, aid, gear
        self.lo, self.hi = lo, hi
        self.qadr = model.jnt_qposadr[model.actuator_trnid[aid, 0]]
        self.passive = [model.jnt_qposadr[model.joint(name).id]
                        for name in ('j3', 'j5', 'j6', 'j7')]
        self.work = mujoco.MjData(model)
        reset_reference_pose(model, self.work)
        self.reference = self.work.qpos.copy()
        self.tip, self.box = model.geom('haptic_tip').id, model.geom('haptic_box').id
        self.samples = [lo]
        self.poses = [self.pose(lo, self.reference)]
        if self.gap(self.poses[0]) <= 0:
            raise SafetyError('Virtual wall overlaps horizontal reference.')
        # A dense monotonic sweep brackets the first contact, not a far-side exit.
        for target in np.linspace(lo, hi, 161)[1:]:
            pose = self.pose(target, self.poses[-1])
            if self.gap(pose) <= 0:
                low, high = self.samples[-1], target
                safe_pose = self.poses[-1]
                for _ in range(18):
                    middle = (low + high) / 2
                    candidate = self.pose(middle, safe_pose)
                    if self.gap(candidate) > 1e-7:
                        low, safe_pose = middle, candidate
                    else:
                        high = middle
                self.boundary = low
                self.samples.append(low)
                self.poses.append(safe_pose)
                break
            self.samples.append(target)
            self.poses.append(pose)
        else:
            raise SafetyError('No geometric contact within calibrated travel; move the virtual box.')
        self.samples = np.asarray(self.samples)
        self.poses = np.asarray(self.poses)
        reset_reference_pose(model, data)

    def pose(self, target, seed):
        d, m = self.work, self.model
        d.qpos[:] = self.reference
        d.qpos[self.passive] = seed[self.passive]
        d.qpos[self.qadr] = target / self.gear
        d.qvel[:] = 0
        # All connect equality residuals come first in this model; contact and
        # limit residuals are deliberately excluded from the kinematic solve.
        count = m.neq * 3
        for _ in range(20):
            mujoco.mj_forward(m, d)
            error = d.efc_pos[:count].copy()
            if np.max(np.abs(error)) < 1e-9:
                return d.qpos.copy()
            jac = np.empty((count, len(self.passive)))
            for column, index in enumerate(self.passive):
                original = d.qpos[index]
                d.qpos[index] += 1e-6
                mujoco.mj_forward(m, d)
                jac[:, column] = (d.efc_pos[:count] - error) / 1e-6
                d.qpos[index] = original
            step = np.linalg.lstsq(jac, -error, rcond=None)[0]
            d.qpos[self.passive] += np.clip(step, -0.15, 0.15)
        raise SafetyError('Closed linkage kinematic solve failed; feedback unavailable.')

    def gap(self, pose):
        d, m = self.work, self.model
        d.qpos[:] = pose
        mujoco.mj_forward(m, d)
        local = d.geom_xmat[self.box].reshape(3, 3).T @ (d.geom_xpos[self.tip] - d.geom_xpos[self.box])
        delta = np.abs(local) - m.geom_size[self.box]
        return float(np.linalg.norm(np.maximum(delta, 0)) + min(np.max(delta), 0)
                     - m.geom_size[self.tip, 0])

    def render(self, data, requested):
        target = float(np.clip(requested, self.lo, self.boundary))
        if target >= self.boundary:
            pose = self.poses[-1]
        else:
            i = min(np.searchsorted(self.samples, target, side='right') - 1, len(self.samples) - 2)
            fraction = (target - self.samples[i]) / (self.samples[i + 1] - self.samples[i])
            seed = self.poses[i] + fraction * (self.poses[i + 1] - self.poses[i])
            pose = self.pose(target, seed)
        if self.gap(pose) < -1e-7:
            raise SafetyError('Virtual wall geometric penetration; releasing.')
        data.qpos[:] = pose
        data.qvel[:] = 0
        data.qacc_warmstart[:] = 0
        data.qfrc_applied[:] = 0
        data.xfrc_applied[:] = 0
        data.time += .008
        mujoco.mj_forward(self.model, data)
        return target


def ramp_current(target, previous, dt):
    """200 mA/s increase, immediate decrease/release; cap gaps at 50 ms."""
    return min(target, previous + 200.0 * min(max(dt, 0), .05))


class VirtualWall:
    def __init__(self, stiffness=DEFAULT_STIFFNESS, max_current=DEFAULT_CURRENT_MA):
        self.stiffness = stiffness
        self.max_current = max_current
        self.boundary = None

    def update(self, requested, boundary, enabled):
        self.boundary = boundary if enabled and requested > boundary else None
        return (min(self.max_current, self.stiffness * (requested - boundary))
                if self.boundary is not None else 0.0)


def signed_current(magnitude, gear, direction):
    # Flexion grows with ctrl = gear * (direction * raw_angle + offset).
    return int(round(-gear * direction * magnitude))


def check_state(state, motor, *, require_travel=True):
    problems = []
    if state.hardware_error:
        problems.append(f'hardware error=0x{state.hardware_error:02X}')
    if require_travel and not motor.min_position <= state.position_raw <= motor.max_position:
        problems.append(f'position={state.position_raw} outside calibrated '
                        f'[{motor.min_position}, {motor.max_position}] '
                        f'(EF-horizontal reference={motor.zero_position})')
    if problems:
        raise SafetyError(f'ID {motor.motor_id}: ' + '; '.join(problems) +
                          '. Feedback cannot continue; do not bypass calibration limits.')
    if not math.isfinite(state.position_rad):
        raise SafetyError('Invalid motor position.')


@contextmanager
def current_session(driver, motor, cap):
    """Only the selected motor is opened. Restore settings, never restore torque ON."""
    mid = motor.motor_id
    read = lambda a, s: driver.bus.read(mid, a, s)
    def write(a, s, value):
        driver.bus.write(mid, a, s, value)
        actual = read(a, s)
        if actual != value:
            raise SafetyError(f'ID {mid}: register {a} readback mismatch '
                              f'(expected={value}, actual={actual}).')
    mode = read(11, 1)
    if mode not in (0, 3, 5) or read(10, 1) != 0:
        raise SafetyError('Demo requires mode 0/3/5 and Drive Mode 0; no settings changed.')
    watchdog = read(98, 1)
    if watchdog == 255:
        raise SafetyError('Clear the existing Bus Watchdog fault before starting.')
    saved = {(a, s): read(a, s) for a, s in RESTORE}
    state = driver.read_telemetry()[mid]
    print(f'Preflight ID {mid}: mode={mode}, torque={int(state.torque_enabled)}, '
          f'position={state.position_raw}, limits=[{motor.min_position}, {motor.max_position}], '
          f'EF-horizontal={motor.zero_position}, hardware_error=0x{state.hardware_error:02X}, '
          f'voltage={state.input_voltage_v:.1f}V, temperature={state.temperature_c:.0f}C. '
          'No register writes yet.', flush=True)
    check_state(state, motor, require_travel=False)
    if not motor.min_position <= state.position_raw <= motor.max_position:
        print('Starting outside calibrated travel: feedback stays OFF; return manually before arming.', flush=True)
    try:
        failures = driver.disable_torque()
        if failures or read(64, 1):
            raise SafetyError(f'Cannot disable torque: {failures}')
        write(98, 1, 0)
        driver.set_operating_mode(OperatingMode.CURRENT)
        write(38, 2, cap)
        write(102, 2, 0)
        yield
    finally:
        original_error = sys.exc_info()[1]
        restore_errors = []
        torque_off = False
        try:
            # No mode/EEPROM writes unless torque-off is confirmed.
            failures = driver.disable_torque()
            if failures or read(64, 1):
                raise SafetyError(f'Torque-off failed; manual power removal required: {failures}')
            torque_off = True
            write(98, 1, 0)
            write(102, 2, 0)
            write(11, 1, mode)
            # A failed individual restore must not skip later settings/watchdog.
            for (address, size), value in saved.items():
                try:
                    write(address, size, value)
                except Exception as exc:
                    restore_errors.append(str(exc))
            write(98, 1, watchdog)
        except Exception as exc:
            restore_errors.append(str(exc))
        if restore_errors:
            message = ('Restore incomplete; ' + ('torque OFF confirmed. ' if torque_off else
                       'torque OFF NOT confirmed. ') + ' | '.join(restore_errors))
            print(message, file=sys.stderr, flush=True)
            if original_error is not None:
                original_error.add_note(message)
            else:
                raise SafetyError(message)
        else:
            print('Original mode/current/gains/profiles restored; torque OFF. '
                  'Previous Goal Position was not replayed; initialize a fresh position target before torque ON.', flush=True)


def run(args, servo, motor, model, data, aid, gear, lo, hi, driver=None, proxy=None):
    import mujoco.viewer
    proxy = proxy or KinematicWall(model, data, aid, gear, lo, hi)
    wall = VirtualWall(args.stiffness, args.max_current_ma)
    flags = {'enabled': False, 'quit': False}
    def key_callback(key):
        if key == 32:
            flags['enabled'] = not flags['enabled']
        elif key in (81, 256):
            flags['quit'] = True
    tip, box = model.geom('haptic_tip').id, model.geom('haptic_box').id
    base_ctrl = data.ctrl.copy()
    last = None
    last_print = 0.0
    armed = False
    outside_travel = False
    previous_magnitude = 0.0
    print(f'Fixed contact boundary: {proxy.boundary:.4f} rad; full current at input '
          f'{proxy.boundary + args.max_current_ma / args.stiffness:.4f} rad (travel end {hi:.4f}).', flush=True)
    print('Space: feedback ON/OFF (starts OFF). Q/Esc: exit. Green sphere: contact proxy.', flush=True)
    mode_label = 'HARDWARE' if driver else 'PREVIEW / NO HARDWARE'
    print(f'Feedback current cap: {args.max_current_ma} mA; stiffness: {args.stiffness:g} mA/rad.', flush=True)
    print('HARDWARE: move ONLY index finger by hand; sliders do not command motors.' if driver else
          'PREVIEW ONLY: no real2sim/sim2real connection. Drag ONLY index_motor.\n'
          'For real finger input + contact feedback, restart with --hardware.', flush=True)
    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback,
                                     show_left_ui=True, show_right_ui=True) as viewer:
        with viewer.lock():
            viewer.cam.lookat[:] = model.stat.center
            viewer.cam.distance = 1.05 * model.stat.extent
            viewer.cam.azimuth = model.vis.global_.azimuth
            viewer.cam.elevation = model.vis.global_.elevation
        while viewer.is_running() and not flags['quit']:
            started = time.monotonic()
            if last is not None and started - last > 0.15 and armed:
                raise SafetyError('Feedback loop stalled >150 ms; releasing. Restart demo.')
            dt = .008 if last is None else started - last
            last = started
            viewer.sync()
            if not viewer.is_running() or flags['quit']:
                break
            if driver:
                state = driver.read_telemetry()[motor.motor_id]
                # Hardware faults remain fatal. Outside calibrated travel we
                # keep torque enabled but command zero current until re-entry.
                check_state(state, motor, require_travel=False)
                within_travel = motor.min_position <= state.position_raw <= motor.max_position
                if armed and not within_travel:
                    driver.command_currents({motor.motor_id: 0})
                    wall.boundary = None
                    previous_magnitude = 0.0
                    if not outside_travel:
                        print(f'OUTSIDE TRAVEL: ID {motor.motor_id} raw={state.position_raw}; '
                              'torque remains ON, feedback current=0. Feedback resumes gradually '
                              'on return to calibrated contact region.', flush=True)
                outside_travel = not within_travel
                if armed and (not state.torque_enabled or driver.read_register(motor.motor_id, Register.BUS_WATCHDOG) == 255):
                    raise SafetyError('Torque lost or watchdog expired; releasing.')
                requested = gear * (state.position_rad + servo['offset'])
                if flags['enabled'] and not armed:
                    # Calibrated extension beyond the horizontal display stop
                    # is also a free-space start; feedback stays zero there.
                    if not within_travel or requested > lo + 0.05:
                        flags['enabled'] = False
                        print(f'Cannot arm at raw={state.position_raw}: stay inside '
                              f'[{motor.min_position}, {motor.max_position}] and return '
                              'near EF-horizontal or its extension side, then press Space.')
                        continue
                    driver.enable_torque()
                    driver.write_register(motor.motor_id, Register.BUS_WATCHDOG, 10)
                    armed = True
                elif not flags['enabled'] and armed:
                    driver.command_currents({motor.motor_id: 0})
                    failures = driver.disable_torque()
                    if failures:
                        raise SafetyError(f'Cannot release torque: {failures}')
                    armed = False
            else:
                requested = float(data.ctrl[aid])
            # The preview has a horizontal extension stop; physical calibrated
            # travel extends past it. Retreat there is free, not a hardware fault.
            feedback_enabled = (flags['enabled'] and lo <= requested <= hi
                                and (driver is None or within_travel))
            requested = float(np.clip(requested, lo, hi))
            with viewer.lock():
                data.ctrl[:] = base_ctrl
                data.ctrl[aid] = requested
                actual = proxy.render(data, requested)
                contact = requested >= proxy.boundary
                magnitude = wall.update(requested, proxy.boundary, feedback_enabled)
                magnitude = ramp_current(magnitude, previous_magnitude, dt)
                previous_magnitude = magnitude
                command = signed_current(magnitude, gear, motor.direction)
                model.geom_rgba[box] = [1, 0.15 if wall.boundary is not None else 0.55, 0.1, 0.7]
            if driver and armed:
                if time.monotonic() - started > 0.15:
                    raise SafetyError('Slow feedback cycle; releasing.')
                driver.command_currents({motor.motor_id: command})
            if started - last_print >= 0.5:
                measured = (f' measured={state.current_ma:+.0f}mA raw={state.position_raw} '
                            f'travel={"OK" if within_travel else ("OUTSIDE / CURRENT ZERO" if armed else "OUTSIDE / ARMING BLOCKED")}'
                            if driver else '')
                status = f'{"ON" if flags["enabled"] else "OFF"} contact={contact} wall={wall.boundary is not None}'
                print(f'{mode_label} {status} input={requested:.3f} sim={actual:.3f} '
                      f'boundary={proxy.boundary:.3f} penetration={max(0, requested-proxy.boundary):.3f}rad '
                      f'current={command:+d}mA{measured}', flush=True)
                viewer.set_texts((None, None, mode_label + '\nSpace: feedback ON/OFF; Q: exit',
                                  status + f'\nGoal current: {command:+d} mA'))
                last_print = started
            time.sleep(max(0, 0.008 - (time.monotonic() - started)))
        print('Viewer loop exiting; closing window.', flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--hardware', action='store_true', help='Connect ONLY the configured index motor; temporarily use mode 0.')
    parser.add_argument('--config', type=Path, default=CONFIG_PATH)
    parser.add_argument('--device')
    parser.add_argument('--max-current-ma', type=int, default=DEFAULT_CURRENT_MA)
    parser.add_argument('--stiffness', type=float, default=DEFAULT_STIFFNESS, help='Virtual wall stiffness in mA/rad.')
    parser.add_argument('--wall-fraction', type=float, default=0.65)
    args = parser.parse_args(argv)
    if not 1 <= args.max_current_ma <= 70:
        parser.error('--max-current-ma must be 1..70.')
    if not math.isfinite(args.stiffness) or not 0 < args.stiffness <= 500:
        parser.error('--stiffness must be finite and in (0, 500].')
    if not 0.3 <= args.wall_fraction <= 0.9:
        parser.error('--wall-fraction must be in [0.3, 0.9].')
    config = load_config(args.config)
    if config.get('reference_pose') != 'ef_horizontal':
        parser.error('EF-horizontal calibration is required.')
    servo = next(s for s in config['servos'] if s['finger'] == 'index')
    motor = motor_configs({'servos': [servo]})[0]
    model, data, aid, gear, lo, hi = build_scene(servo, motor, args.wall_fraction)
    proxy = KinematicWall(model, data, aid, gear, lo, hi)
    if args.hardware:
        with open_driver([motor], args.device or config['device'], config['baudrate'], False) as driver:
            with current_session(driver, motor, args.max_current_ma):
                run(args, servo, motor, model, data, aid, gear, lo, hi, driver, proxy)
    else:
        run(args, servo, motor, model, data, aid, gear, lo, hi, proxy=proxy)


if __name__ == '__main__':
    # SIGSEGV cannot be caught as a Python exception. Keep thread stacks on disk
    # so a native viewer/GL crash can be distinguished from control-loop errors.
    crash_path = Path('/tmp') / f'glove-haptic-crash-{os.getpid()}.log'
    crash_log = crash_path.open('w')
    crash_log.write(f'MuJoCo {mujoco.__version__}; Python {sys.version}\n')
    crash_log.flush()
    faulthandler.enable(file=crash_log, all_threads=True)
    print(f'Native crash diagnostics: {crash_path}', flush=True)
    try:
        main()
    except BaseException:
        traceback.print_exc(file=crash_log)
        crash_log.flush()
        raise
