"""Check the physical pivot lines and closed-chain constraints, without hardware."""

import mujoco
import numpy as np

from simulation.glove.reference_pose import SCENE_PATH, reset_reference_pose


def test_horizontal_reference_is_closed_stable_and_resettable():
    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    data = mujoco.MjData(model)
    reset_reference_pose(model, data)
    initial = data.qpos.copy()
    for ef, f in (("short_ef_28_045", "short_fgh"),
                  ("lang_ef_36_549", "lang_fgh"),
                  ("part_1_6", "part_1_7"),
                  ("part_1_16", "part_1_17")):
        assert abs(data.xpos[model.body(f).id, 2] - data.xpos[model.body(ef).id, 2]) < 1e-8
    assert max(abs(data.efc_pos[:model.neq * 3])) < 1e-8
    for _ in range(round(10 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    assert max(abs(data.qpos - initial)) < 1e-5
    assert not any(w.number for w in data.warning)
    data.qpos[:] = 0.3
    data.ctrl[:] = 0.3
    reset_reference_pose(model, data)
    np.testing.assert_array_equal(data.qpos, initial)
    np.testing.assert_allclose(
        data.ctrl,
        model.actuator_gear[:, 0] * data.qpos[model.jnt_qposadr[model.actuator_trnid[:, 0]]],
    )
