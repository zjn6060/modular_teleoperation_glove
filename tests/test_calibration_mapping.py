"""Verify calibration preserves hardware mapping and per-motor settings offline."""

import importlib.util
import json
from pathlib import Path
from unittest.mock import patch

import pytest

from driver import MotorConfig, SimulatedBus, XL330Driver
from simulation.glove.reference_pose import reference_angles


@pytest.mark.parametrize("ids", [(2, 3, 5, 4), (12, 13, 15, 14)])
def test_recalibration_preserves_configured_mapping(ids, tmp_path):
    script = Path(__file__).resolve().parents[1] / "scripts" / "calibration.py"
    spec = importlib.util.spec_from_file_location("calibration_under_test", script)
    calibration = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(calibration)

    definitions = [
        {"finger": finger, "motor_id": motor_id, "joint": joint,
         "direction": sign, "offset": offset,
         "zero_position": 2000, "min_position": 1800, "max_position": 2200}
        for motor_id, (finger, joint, sign, offset) in zip(ids, (
            ("index", "j2", 1, 0.12), ("middle", "j10", -1, -0.31),
            ("ring", "j26", 1, 0.27), ("little", "j18", -1, -0.51)))
    ]
    driver = XL330Driver(SimulatedBus(ids), [MotorConfig(motor_id=i) for i in ids])
    zeros = {i: 1900 + i * 10 for i in ids}

    def sample(driver, prompt, on_sample):
        if "Step 1" in prompt:
            return zeros
        on_sample({i: p - 80 for i, p in zeros.items()})
        on_sample({i: p + 110 for i, p in zeros.items()})
        return zeros

    output = tmp_path / "servos.json"
    output.write_text(json.dumps({"device": "/dev/ttyUSB0", "baudrate": 1000000,
                                 "servos": definitions}))
    with patch.object(calibration, "open_driver", return_value=driver) as open_driver, \
         patch.object(calibration, "sample_until_enter", side_effect=sample):
        calibration.main(["--output", str(output), "--simulate"])
    assert tuple(m.motor_id for m in open_driver.call_args.args[0]) == ids
    saved = json.loads(output.read_text())
    assert saved["reference_pose"] == "ef_horizontal"
    offsets = reference_angles()
    for before, after in zip(definitions, saved["servos"]):
        for field in ("motor_id", "finger", "joint", "direction"):
            assert after[field] == before[field]
        assert after["offset"] == offsets[after["joint"]]
        assert after["zero_position"] == zeros[after["motor_id"]]
        assert after["min_position"] == zeros[after["motor_id"]] - 80
        assert after["max_position"] == zeros[after["motor_id"]] + 110
