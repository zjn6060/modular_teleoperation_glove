"""Shared EF-horizontal reference for preview and physical zero calibration."""

from pathlib import Path

import mujoco

SCENE_PATH = Path(__file__).resolve().parent / "scene.xml"
REFERENCE_POSE = "ef_horizontal"


def reset_reference_pose(model, data):
    """Reset state and position targets together, without stepping the mechanism."""
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, REFERENCE_POSE)
    if key_id < 0:
        raise ValueError(f"Model is missing reference pose {REFERENCE_POSE!r}.")
    mujoco.mj_resetDataKeyframe(model, data, key_id)
    mujoco.mj_forward(model, data)


def reference_angles():
    """Return the CAD joint angles to associate with EF-horizontal motor zeros."""
    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    key = model.key(REFERENCE_POSE)
    # Calibration stores joint angles, independent of slider direction and order.
    return {
        model.actuator(i).name: float(key.qpos[model.jnt_qposadr[model.actuator_trnid[i, 0]]])
        for i in range(model.nu)
    }
