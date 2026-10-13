# Motor driver

This package provides a reusable DYNAMIXEL Protocol 2.0 bus and a class-based
XL330 driver. It is adapted from the DOGlove motor diagnostics, with MuJoCo,
UDP, fixed motor counts, and glove-specific joint mappings removed from the
hardware layer.

## Design

- `DynamixelBus` owns serial communication and register-level operations.
- `XL330Driver` provides motor discovery, state reads, calibration conversion,
  torque lifecycle management, synchronized position commands, and explicit
  position-control configuration.
- `MotorConfig` holds the ID, name, zero, direction, and mechanical limits for
  each motor. Any nonempty set of unique motor IDs is supported.
- `SimulatedBus` implements the same transport interface for offline work and
  tests.

Opening a driver only pings and validates motors. It does not enable torque or
change registers. Before enabling torque, the driver reads every position and
writes those positions as hold targets. Closing the driver attempts to disable
every motor whose torque it enabled.

## Read-only hardware check

Activate the project environment, disconnect DYNAMIXEL Wizard from the serial
port, and run:

```bash
conda activate myglove
python -m driver check --device /dev/ttyUSB0 --baud 1000000 --ids 1 2 3 4
```

Monitor positions until Ctrl+C:

```bash
python -m driver monitor --device /dev/ttyUSB0 --ids 1 2 3 4 --rate 10
```

Exercise both commands without hardware by adding `--simulate`.

## Hardware test programs

The connection report checks every requested motor independently and prints a
per-motor table plus a connected/failed summary:

```bash
python tests/test_motor_connection.py \
  --device /dev/ttyUSB0
```

The default connection scan uses 1,000,000 baud and checks IDs 0 through 10.
Use `--baud` or `--ids` only when you need to override those defaults.

The state-feedback test opens one real-time figure for every motor. Each figure
contains three stacked subplots for position, velocity, and estimated torque,
and also shows current, voltage, temperature, PWM, torque state, motion state,
and hardware errors:

```bash
python tests/test_state_feedback.py \
  --device /dev/ttyUSB0
```

By default it uses 1,000,000 baud, scans IDs 0 through 10, and plots every
healthy XL330 it finds. Explicit `--ids` and `--names` remain available when a
specific subset or custom labels are needed.

Both programs are read-only and accept `--simulate`. The plotting implementation
is reusable from `utils/debugging/realtime_motor_plot.py`. Torque is explicitly
shown as an estimate derived from current and the model's 5 V stall rating;
XL330 input-current feedback is not a calibrated torque measurement.

## Application API

```python
import math

from driver import MotorConfig, OperatingMode, XL330Driver

motors = [
    MotorConfig(
        motor_id=1,
        name="index",
        zero_position=2048,
        direction=1,
        min_position=1900,
        max_position=2300,
    ),
    MotorConfig(
        motor_id=2,
        name="middle",
        zero_position=2048,
        direction=-1,
        min_position=1800,
        max_position=2200,
    ),
]

driver = XL330Driver.from_serial(
    motors,
    device="/dev/ttyUSB0",
    baudrate=1_000_000,
)

with driver:
    print(driver.read_states())

    # Apply only settings explicitly supplied here. Operating mode and current
    # limit are EEPROM-backed on the XL330, so configure them deliberately.
    driver.configure_position_control(
        operating_mode=OperatingMode.CURRENT_BASED_POSITION,
        current_limit=50,
        position_p_gain=100,
        profile_velocity=10,
        profile_acceleration=5,
        watchdog=25,
    )

    driver.enable_torque()
    driver.command_angles({
        1: math.radians(5),
        2: math.radians(-5),
    })
```

Angle commands are relative to each motor's configured zero. Targets outside
the configured tick limits raise `SafetyError`; pass `clamp=True` to
`command_angles` only when clipping is intended.

## Tests

```bash
conda activate myglove
python -m pytest -q driver/tests
```

The tests use `SimulatedBus` and do not access serial hardware.

## Attribution

Portions of the driver design and safety behavior were adapted from the
DOGlove project by Tsinghua Embodied AI Lab. See `LICENSE.DOGlove`.

