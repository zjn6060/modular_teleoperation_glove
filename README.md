# Modular Teleoperation Glove

This repository contains the hardware drivers and supporting software for the
modular teleoperation glove. The first driver targets ROBOTIS DYNAMIXEL XL330
motors and is being adapted from the DOGlove reference implementation under
`reference/DOGlove`.

## Conda environment

The project uses a Conda environment named `myglove`. Create it from the
checked-in environment definition:

```bash
conda env create -f environment.yml
conda activate myglove
```

The definition installs Python 3.11, NumPy, pyserial, the DYNAMIXEL SDK,
MuJoCo, Matplotlib, and pytest. If a fresh Miniconda installation asks you to
accept the Anaconda default-channel terms even though this project uses only
conda-forge, create the same environment with an explicit channel override:

```bash
conda create --name myglove --override-channels --channel conda-forge \
  python=3.11 pip numpy pyserial pytest mujoco matplotlib
conda activate myglove
python -m pip install dynamixel-sdk
```

Confirm that Python and the motor communication library are available:

```bash
python --version
python -c "import dynamixel_sdk, serial, numpy, mujoco, matplotlib; print('myglove dependencies OK')"
```

After `environment.yml` changes, update an existing environment with:

```bash
conda env update --name myglove --file environment.yml --prune
```

To leave the environment:

```bash
conda deactivate
```

## Serial device access

The motor driver communicates with a U2D2 or another compatible serial adapter.
On Linux, identify the adapter after connecting it:

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

If the port exists but cannot be opened, add the current user to the serial-port
group, then log out and back in:

```bash
sudo usermod -aG dialout "$USER"
```

Only one application can own the serial port at a time. Disconnect ROBOTIS
DYNAMIXEL Wizard before running the driver. The adapter and motors must use the
same baud rate, and each motor on the bus must have a unique ID.

## Repository layout

- `driver/` — class-based motor drivers and read-only diagnostics; see
  [`driver/README.md`](driver/README.md).
- `simulation/glove/` — assembled MuJoCo glove model, mesh assets, and sidebar
  control preview; see [`simulation/glove/README.md`](simulation/glove/README.md).
- `reference/DOGlove/` — upstream reference code used during development.

The `myglove` environment includes the core motor-driver dependencies, MuJoCo,
and Matplotlib. ROS and learning dependencies can be added as those parts of the
project are introduced.

## MuJoCo glove preview

From the repository root, use an environment with MuJoCo and NumPy installed:

```bash
conda activate myglove
python3 simulation/glove/view_glove.py
```

The viewer starts running with a closer, centered camera. Press **Space** to pause/resume;
use the right **Control** panel to adjust joint targets in radians.
**Backspace** resets the model, and **Shift+Tab** toggles the right sidebar.
The most recently edited linkage control in each finger drives that mechanism;
other linkage targets follow their current angles to allow closed-loop motion.

The repaired assembly contains 24 hinge joints, 24 position actuators, and
16 equality constraints for eight closed revolute connections. Gravity and
contacts are disabled, and masses and inertias are approximate; this preview
is intended for inspecting assembly and linkage motion. Repair details and
validation results are included in `simulation/glove/`.

The included `config.json` records the source Onshape assembly. Regenerating
the model with onshape-to-robot overwrites the repaired XML; follow the
closed-mate naming instructions in the simulation README before re-exporting.
