# TeleGlove

TeleGlove is a modular teleoperation glove project with hardware drivers and
supporting software. The first driver targets ROBOTIS DYNAMIXEL XL330
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
- `simulation/robots/tiangong/` — Tiangong TG2 URDF models, mesh assets, and a
  MuJoCo target model; see [`simulation/robots/tiangong/README.md`](simulation/robots/tiangong/README.md).
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

The five-finger assembly includes the thumb mounted 60 mm rearward, with 31
hinge joints, 31 position controls, and 20 equality constraints for ten closed
revolute connections. The thumb controls are `thumb_mcp_abduction`,
`thumb_A`, `thumb_C`, `thumb_D`, `thumb_E`, `thumb_F`, and `thumb_H`;
the fingertip saddle (`thumb_H`) can be controlled independently. Gravity and
contacts are disabled, and masses and inertias are approximate; this preview
is intended for inspecting assembly and linkage motion. Repair details and
validation results are included in `simulation/glove/`.

The included `config.json` records the source Onshape assembly. Regenerating
the model with onshape-to-robot overwrites the processed XML; export into a
separate directory and verify the rigid groups and closed-loop constraints
before replacing the current model. The current five-finger validation is in
`validation.json` and `onshape_validation.json`; older four-finger reports are
identified in the simulation README.

![Five-finger MuJoCo glove preview](simulation/glove/preview.png)

## Tiangong robot target

The imported TG2 models are available under `simulation/robots/tiangong/`.
To preview the existing MuJoCo target model from the repository root:

```bash
conda activate myglove
python -m mujoco.viewer --mjcf=simulation/robots/tiangong/tg2_inspirehand.xml
```

This adds the robot assets for future sim2sim work. Glove-to-hand joint mapping,
calibration, and a combined simulation control loop are not implemented by this
asset import. See the model README for provenance and validation details.
