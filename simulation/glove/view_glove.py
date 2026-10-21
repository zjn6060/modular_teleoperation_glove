#!/usr/bin/env python3
"""Interactive, contact-free closed-linkage preview with automatic follower targets."""
from pathlib import Path
import time
import numpy as np
import mujoco
import mujoco.viewer


FINGER_CONTROLS = {
    'index': ('index_mcp_abduction', 'j2', 'j3', 'j5', 'j6', 'j7'),
    'middle': ('middle_mcp_abduction', 'j10', 'j12', 'j13', 'j14', 'j15'),
    'little': ('little_mcp_abduction', 'j18', 'j20', 'j21', 'j22', 'j24'),
    'ring': ('ring_mcp_abduction', 'j26', 'j28', 'j29', 'j30', 'j31'),
    'thumb': ('thumb_mcp_abduction', 'thumb_A', 'thumb_C', 'thumb_D', 'thumb_E', 'thumb_F', 'thumb_H'),
}


def control_groups(names):
    """Resolve each finger's abduction control and linkage followers by name."""
    indices = {name: i for i, name in enumerate(names)}
    expected = {name for controls in FINGER_CONTROLS.values() for name in controls}
    if set(indices) != expected or len(indices) != len(names):
        raise ValueError('Actuator names do not match FINGER_CONTROLS.')
    lateral = {indices[controls[0]] for controls in FINGER_CONTROLS.values()}
    lateral.add(indices['thumb_H'])
    groups = {indices[name]: finger for finger, controls in FINGER_CONTROLS.items()
              for name in controls}
    defaults = {finger: indices[controls[1]]
                for finger, controls in FINGER_CONTROLS.items()}
    return lateral, groups, defaults


def main():
    model = mujoco.MjModel.from_xml_path(str(Path(__file__).resolve().parent / 'scene.xml'))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(model.nu)]
    qadr = model.jnt_qposadr[model.actuator_trnid[:, 0]]
    lateral, groups, defaults = control_groups(names)
    drivers = dict(defaults)
    targets = np.zeros(model.nu)
    displayed = data.ctrl.copy()
    events = {'paused': False, 'reset': False}

    def key_callback(key):
        if key == 32:
            events['paused'] = not events['paused']
        elif key == 259:
            events['reset'] = True

    print(f'Loaded: {model.nu} controls; {model.neq // 2} closed revolute connections; all five fingers.')
    print('Starts running. Space: pause/run. Backspace: reset. Shift+Tab: right sidebar.')
    print('Use Control sliders. Within each finger, the most recently edited linkage slider drives; other linkage targets follow.')
    print('thumb_mcp_abduction controls the palm hinge; thumb_H independently controls the fingertip saddle.')
    print('Approximate inertias; gravity and contacts disabled for mechanism preview.')

    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback,
                                     show_left_ui=True, show_right_ui=True) as viewer:
        with viewer.lock():
            viewer.cam.lookat[:] = model.stat.center
            viewer.cam.distance = 1.05 * model.stat.extent
            viewer.cam.azimuth = model.vis.global_.azimuth
            viewer.cam.elevation = model.vis.global_.elevation
        while viewer.is_running():
            started = time.monotonic()
            viewer.sync()
            with viewer.lock():
                if events['reset']:
                    mujoco.mj_resetData(model, data)
                    mujoco.mj_forward(model, data)
                    targets[:] = 0
                    displayed[:] = 0
                    drivers = dict(defaults)
                    events['reset'] = False
                changed = np.flatnonzero(np.abs(data.ctrl - displayed) > 1e-5)
                for i in changed:
                    targets[i] = data.ctrl[i]
                    if i not in lateral:
                        drivers[groups[i]] = int(i)
                if not events['paused']:
                    for _ in range(max(1, round((1 / 60) / model.opt.timestep))):
                        data.ctrl[:] = data.qpos[qadr]
                        for i in lateral:
                            data.ctrl[i] = targets[i]
                        for i in drivers.values():
                            data.ctrl[i] = targets[i]
                        mujoco.mj_step(model, data)
                displayed = data.ctrl.copy()
            time.sleep(max(0, 1 / 60 - (time.monotonic() - started)))


if __name__ == '__main__':
    main()
