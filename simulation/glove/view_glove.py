#!/usr/bin/env python3
"""Interactive, contact-free closed-linkage preview with automatic follower targets."""
from pathlib import Path
import time
import numpy as np
import mujoco
import mujoco.viewer


def main():
    model = mujoco.MjModel.from_xml_path(str(Path(__file__).resolve().parent / 'scene.xml'))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(model.nu)]
    qadr = model.jnt_qposadr[model.actuator_trnid[:, 0]]
    numbers = [int(n[1:]) for n in names]
    lateral = {i for i, n in enumerate(numbers) if n in (1, 9, 17, 25)}
    groups = {i: (n - 1) // 8 for i, n in enumerate(numbers)}
    defaults = {g: names.index('j' + str(2 + 8 * g)) for g in range(4)}
    drivers = dict(defaults)
    targets = np.zeros(model.nu)
    displayed = data.ctrl.copy()
    events = {'paused': True, 'reset': False}

    def key_callback(key):
        if key == 32:
            events['paused'] = not events['paused']
        elif key == 259:
            events['reset'] = True

    print('Loaded: 24 controls; 8 closed revolute connections.')
    print('Starts paused. Space: run/pause. Backspace: reset. Shift+Tab: right sidebar.')
    print('Use Control sliders. Within each finger, the most recently edited linkage slider drives; other linkage targets follow.')
    print('Approximate inertias; gravity and contacts disabled for mechanism preview.')

    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback,
                                     show_left_ui=True, show_right_ui=True) as viewer:
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
