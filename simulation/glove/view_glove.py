#!/usr/bin/env python3
"""Interactive, contact-free closed-linkage preview.

Each finger has two controls: MCP abduction and the servo-horn joint. The other
linkage joints are passive and follow through the closed-loop constraints.
"""
from pathlib import Path
import time
import mujoco
import mujoco.viewer

if __package__:
    from .reference_pose import reset_reference_pose
else:
    from reference_pose import reset_reference_pose


def run(make_hook=None):
    """Run the preview; make_hook(model, data) may return an on_frame() callback."""
    model = mujoco.MjModel.from_xml_path(str(Path(__file__).resolve().parent / 'scene.xml'))
    data = mujoco.MjData(model)
    reset_reference_pose(model, data)
    on_frame = make_hook(model, data) if make_hook else None
    events = {'paused': False, 'reset': False}

    def key_callback(key):
        if key == 32:
            events['paused'] = not events['paused']
        elif key == 259:
            events['reset'] = True

    print(f'Loaded: {model.nu} controls (abduction + servo horn per finger); 8 closed revolute connections.')
    print('Starts with EF rods horizontal. Space: pause/run. Backspace: horizontal reset. Shift+Tab: right sidebar.')
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
                    reset_reference_pose(model, data)
                    events['reset'] = False
                if on_frame:
                    on_frame()
                if not events['paused']:
                    for _ in range(max(1, round((1 / 60) / model.opt.timestep))):
                        mujoco.mj_step(model, data)
            time.sleep(max(0, 1 / 60 - (time.monotonic() - started)))


if __name__ == '__main__':
    run()
