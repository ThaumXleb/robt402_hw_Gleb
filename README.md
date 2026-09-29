# ROBT 402 IMU Quaternion 3D Visualization (Ubuntu WSL)

Real-time IMU orientation visualization for **ROBT 402 – Robot/Mechatronic System Design**.

The program runs inside **Ubuntu on WSL/WSLg** and connects to a **Raspberry Pi over SSH**. The Raspberry Pi remains physically connected to the IMU over I2C. Sensor data is streamed to WSL, processed with a Madgwick orientation filter, converted to a quaternion, and used to rotate a 3D IMU model in a FreeGLUT/OpenGL window.

Main script:

```text
imu_3d_wsl_opengl_fixed.py
```

## System architecture

```text
+-----------------------+        SSH        +---------------------------+
|     Raspberry Pi      | ----------------> |       Ubuntu WSL          |
|                       |                   |                           |
| LSM6DS33 IMU          |                   | Paramiko SSH client       |
|    |                  |                   |        |                  |
|    +-- I2C bus 1      |                   |        v                  |
|                       |                   | raw IMU measurements      |
| i2c-tools             |                   |        |                  |
| - i2cdetect           |                   |        v                  |
| - i2cget / i2cset     |                   | gyro bias calibration     |
| - i2ctransfer         |                   |        |                  |
+-----------------------+                   |        v                  |
                                            | Madgwick filter           |
                                            |        |                  |
                                            |        v                  |
                                            | quaternion (w,x,y,z)      |
                                            |        |                  |
                                            |        v                  |
                                            | PyOpenGL + FreeGLUT       |
                                            +-------------+-------------+
                                                          |
                                                          v
                                                Windows desktop window
                                                live 3D IMU orientation
```

The WSL machine does **not** need direct access to the I2C bus. All I2C commands are executed remotely on the Raspberry Pi through SSH.

---

## Features

- SSH connection from Ubuntu WSL to Raspberry Pi using `paramiko`
- Automatic LSM6DS33 detection at I2C address `0x6B`
- LSM6DS33 configuration and continuous sensor acquisition
- Automatic LIS3MDL magnetometer detection at `0x1E`
- 6-DOF fusion when only gyroscope + accelerometer are available
- 9-DOF fusion when gyroscope + accelerometer + magnetometer are available
- Initial gyroscope bias calibration
- Madgwick AHRS orientation estimation
- Quaternion output in `(w, x, y, z)` format
- Real-time OpenGL/FreeGLUT visualization
- Fixed world coordinate axes and rotating IMU body axes
- Orientation zeroing and gyro recalibration from the keyboard
- WSLg/OpenGL workaround that forces PyOpenGL to use the X11/GLX backend

---

## Hardware assumptions

The current program is configured for the following hardware:

| Device | I2C address | Purpose |
|---|---:|---|
| LSM6DS33 | `0x6B` | Gyroscope + accelerometer |
| LIS3MDL | `0x1E` | Magnetometer, optional |

The Raspberry Pi uses:

```text
I2C bus: 1
```

### LSM6DS33 settings

The program configures the LSM6DS33 for:

- Accelerometer: **±2 g**, 26 Hz
- Gyroscope: **±500 deg/s**, 26 Hz
- Block Data Update enabled
- Register auto-increment enabled

The conversion constants used by the program are:

```text
Gyroscope:     0.0175 deg/s per LSB
Accelerometer: 0.000061 g per LSB
Gravity:       9.80665 m/s^2
```

### Magnetometer

If a LIS3MDL is detected at `0x1E`, the program automatically enables 9-DOF Madgwick fusion.

If no magnetometer is detected, the program continues in 6-DOF mode using only the gyroscope and accelerometer. In 6-DOF mode, roll and pitch are corrected using gravity, but yaw can drift over time because there is no absolute magnetic heading reference.

---

# 1. Raspberry Pi setup

The Raspberry Pi must:

1. Have I2C enabled.
2. Have the IMU connected to I2C bus 1.
3. Have SSH enabled.
4. Have `i2c-tools` installed.
5. Allow the SSH user to access `/dev/i2c-1` and run the I2C utilities used by the script.

Install the I2C tools on the Raspberry Pi:

```bash
sudo apt update
sudo apt install -y i2c-tools
```

Check that the I2C device exists:

```bash
ls -l /dev/i2c-1
```

Scan the bus:

```bash
i2cdetect -y 1
```

You should see `6b` for the LSM6DS33.

If your IMU board also contains the LIS3MDL magnetometer, you should normally also see `1e`.

Example:

```text
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- 1e --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
...
60: -- -- -- -- -- -- -- -- -- -- -- 6b -- -- -- --
```

Verify SSH from the Windows/WSL machine before trying the Python program:

```bash
ssh <pi-user>@<pi-ip>
```

For the current setup, the Raspberry Pi address used by the Python script defaults to:

```text
192.168.8.235
```

---

# 2. WSL / Ubuntu setup

This program is intended to run in **Ubuntu under WSL2 with WSLg** so the OpenGL window appears on the Windows desktop.

From **Windows PowerShell**, make sure WSL is current:

```powershell
wsl --update
wsl --shutdown
```

Open Ubuntu again after WSL shuts down.

Check that WSLg provides an X display:

```bash
echo $DISPLAY
```

A normal result is similar to:

```text
:0
```

---

# 3. Install Ubuntu dependencies

Inside Ubuntu WSL:

```bash
sudo apt update
sudo apt install -y \
    python3 \
    python3-pip \
    python3-venv \
    freeglut3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    mesa-utils \
    openssh-client
```

Verify that OpenGL is available:

```bash
glxinfo -B
```

A working setup should show a valid display and OpenGL version, for example:

```text
name of display: :0
direct rendering: Yes
OpenGL version string: 4.x ...
```

Software rendering such as `llvmpipe` is acceptable for this simple visualization.

---

# 4. Create the Python environment

Create a project directory:

```bash
mkdir -p ~/robt402
cd ~/robt402
```

Create a virtual environment:

```bash
python3 -m venv .venv
```

Activate it:

```bash
source .venv/bin/activate
```

Upgrade `pip`:

```bash
pip install --upgrade pip
```

Install the Python dependencies:

```bash
pip install paramiko PyOpenGL
```

If you encounter PyOpenGL/GLUT context problems, the tested version is:

```bash
pip install --upgrade "PyOpenGL==3.1.10"
```

---

# 5. Configure the Raspberry Pi connection

The program reads connection settings from environment variables:

```text
RPI_HOST
RPI_USER
RPI_PASSWORD
```

Set them before running the program:

```bash
export RPI_HOST=192.168.8.235
export RPI_USER=gleb
export RPI_PASSWORD='your_raspberry_pi_password'
```

Then start the program:

```bash
python3 imu_3d_wsl_opengl_fixed.py
```

The script currently contains fallback values if the environment variables are not set. For a GitHub repository, using environment variables is preferred so that real passwords are not committed to source control.

---

# 6. WSLg / PyOpenGL GLX fix

WSLg exposes both Wayland and X11. In some configurations, PyOpenGL and FreeGLUT may select different display backends, which can produce this error:

```text
OpenGL.error.Error: Attempt to retrieve context when no valid context
```

The fixed script handles this before importing OpenGL:

```python
os.environ["PYOPENGL_PLATFORM"] = "glx"
os.environ.pop("WAYLAND_DISPLAY", None)
```

This forces PyOpenGL and FreeGLUT to use the X11/GLX path provided by WSLg.

You can also force the same behavior from the terminal:

```bash
export PYOPENGL_PLATFORM=glx
unset WAYLAND_DISPLAY
python3 imu_3d_wsl_opengl_fixed.py
```

The program also creates the GLUT/OpenGL context on the **main thread before starting the SSH worker thread**. This avoids another common source of OpenGL context errors.

---

# 7. Running the program

Activate the environment if necessary:

```bash
cd ~/robt402
source .venv/bin/activate
```

Optionally export the connection information:

```bash
export RPI_HOST=192.168.8.235
export RPI_USER=gleb
export RPI_PASSWORD='your_raspberry_pi_password'
```

Run:

```bash
python3 imu_3d_wsl_opengl_fixed.py
```

Expected startup output is similar to:

```text
OpenGL backend: GLX/X11 (DISPLAY=:0)
OpenGL visualization started.
Keyboard: Z zero | X clear zero | R recalibrate | I invert | Q/Esc quit
Connecting to Raspberry Pi at gleb@192.168.8.235:22 ...
[HH:MM:SS] SSH connected.
```

The script then checks the Raspberry Pi and I2C bus, configures the IMU, and starts streaming data.

---

# 8. Initial gyroscope calibration

At startup, **do not move the IMU**.

The program takes 50 gyroscope samples to estimate the stationary gyroscope bias:

```text
Keep IMU still: calibrating gyro 0/50
...
Keep IMU still: calibrating gyro 50/50
```

At the configured sensor rate this takes approximately two seconds.

Once calibration is finished, the terminal prints a message similar to:

```text
Gyro bias calibrated: X=..., Y=..., Z=... deg/s
```

The orientation stream then begins.

If the sensor was moved during calibration, press `R` in the OpenGL window and keep the IMU stationary while it recalibrates.

---

# 9. Quaternion output

The program represents every quaternion in the following order:

```text
(w, x, y, z)
```

During normal operation, the terminal prints orientation values such as:

```text
Quaternion w x y z:  0.998324  0.012381 -0.055102  0.013005
```

The quaternion is normalized and is also converted to an OpenGL rotation matrix for the 3D visualization.

---

# 10. OpenGL visualization

The OpenGL window contains:

- a fixed world coordinate frame;
- a reference grid;
- a rectangular IMU body;
- local X/Y/Z axes attached to the IMU body;
- connection/fusion status;
- the current quaternion;
- keyboard controls.

The fixed world axes remain stationary while the IMU body and its local axes rotate according to the estimated quaternion.

---

# 11. Keyboard controls

The OpenGL window must have keyboard focus.

| Key | Action |
|---|---|
| `Z` | Set the current physical orientation as the display zero/reference |
| `X` | Clear the display zero and return to the filter's absolute orientation |
| `R` | Recalculate the gyroscope bias; keep the IMU still |
| `I` | Invert the displayed quaternion rotation if the visual direction is reversed |
| `Q` | Quit |
| `Esc` | Quit |

### Recommended demonstration sequence

1. Place the IMU flat and stationary.
2. Start the Python program.
3. Keep the IMU still during the 50-sample gyro calibration.
4. Wait until the status becomes `Streaming IMU orientation`.
5. Click the OpenGL window.
6. Press `Z` to define the current pose as the visual zero.
7. Rotate the physical IMU and observe the 3D model.
8. If the visual rotation direction is reversed, press `I`.

---

# 12. Sensor fusion

The program implements the Madgwick orientation filter.

## 6-DOF mode

Used when no LIS3MDL is detected:

```text
gyroscope + accelerometer
          |
          v
     Madgwick IMU
          |
          v
   quaternion orientation
```

The accelerometer provides a gravity reference that limits roll/pitch drift. Yaw is primarily obtained by gyro integration and can therefore drift.

## 9-DOF mode

Used when the LIS3MDL is detected at `0x1E`:

```text
gyroscope + accelerometer + magnetometer
                 |
                 v
          Madgwick MARG/AHRS
                 |
                 v
          quaternion orientation
```

The magnetometer supplies an additional heading reference.

The filter gain is currently:

```python
MADGWICK_BETA = 0.08
```

Increasing `MADGWICK_BETA` gives stronger accelerometer/magnetometer correction but can make the orientation more sensitive to sensor noise. Decreasing it gives smoother gyro-dominated motion but allows more drift.

---

# 13. Data acquisition

The program prefers `i2ctransfer` because it can read a coherent block of sensor registers in a single transaction.

For the LSM6DS33, it reads 12 bytes beginning at register `0x22`:

```text
0x22-0x27  gyroscope X/Y/Z
0x28-0x2D  accelerometer X/Y/Z
```

Each measurement is a signed 16-bit value formed from a low byte and a high byte.

If the LIS3MDL is present, an additional 6-byte magnetometer block is read.

If `i2ctransfer` is unavailable, the program falls back to individual `i2cget` operations for the LSM6DS33 and switches to 6-DOF mode.

---

# 14. Program structure

The main parts of `imu_3d_wsl_opengl_fixed.py` are:

```text
Configuration
    |
    +-- Raspberry Pi / SSH settings
    +-- LSM6DS33 settings
    +-- LIS3MDL settings
    +-- Madgwick parameters

Quaternion math
    |
    +-- normalization
    +-- conjugate
    +-- multiplication
    +-- quaternion -> rotation matrix
    +-- quaternion -> Euler angles (display/debug helper)

MadgwickAHRS
    |
    +-- update_imu()   -> 6-DOF
    +-- update_marg()  -> 9-DOF

SSH worker thread
    |
    +-- connect to Raspberry Pi
    +-- scan I2C bus
    +-- configure sensors
    +-- stream measurements
    +-- gyro bias calibration
    +-- run sensor fusion
    +-- send quaternion/data to main thread

OpenGL main thread
    |
    +-- create GLUT context
    +-- draw world axes and grid
    +-- rotate IMU model from quaternion
    +-- draw status/quaternion overlay
    +-- handle keyboard controls
```

OpenGL rendering remains on the main thread. Sensor acquisition and SSH communication run in a background worker thread. A thread-safe `queue.Queue` transfers state updates to the renderer.

---

# 15. Troubleshooting

## `Attempt to retrieve context when no valid context`

Make sure you are running the fixed version of the script and that the GLX backend is selected:

```bash
export PYOPENGL_PLATFORM=glx
unset WAYLAND_DISPLAY
python3 imu_3d_wsl_opengl_fixed.py
```

Also check your PyOpenGL version:

```bash
pip show PyOpenGL
```

If necessary:

```bash
pip install --upgrade "PyOpenGL==3.1.10"
```

---

## No OpenGL window appears

Check WSLg:

```bash
echo $DISPLAY
glxinfo -B
```

If `DISPLAY` is empty or OpenGL cannot connect, from Windows PowerShell run:

```powershell
wsl --update
wsl --shutdown
```

Then reopen Ubuntu.

---

## SSH authentication fails

Test SSH manually:

```bash
ssh gleb@192.168.8.235
```

If this fails, fix the SSH credentials/network connection before troubleshooting the Python program.

Verify the variables used by the script:

```bash
echo "$RPI_HOST"
echo "$RPI_USER"
```

Re-export them if necessary.

---

## `LSM6DS33 at I2C address 0x6B was not detected`

SSH into the Pi and run:

```bash
i2cdetect -y 1
```

Confirm that `6b` is present.

Also check:

```bash
ls -l /dev/i2c-1
```

If the bus does not exist, make sure I2C is enabled on the Raspberry Pi.

---

## `i2cget`, `i2cset`, or `i2ctransfer` not found

Install `i2c-tools` on the Pi:

```bash
sudo apt update
sudo apt install -y i2c-tools
```

Check:

```bash
command -v i2cdetect
command -v i2cget
command -v i2cset
command -v i2ctransfer
```

---

## Cube moves in the opposite direction

Press:

```text
I
```

This displays the conjugate quaternion and reverses the visualization convention.

This is a visualization coordinate-system adjustment; it does not modify the raw sensor measurements.

---

## Cube slowly rotates while the sensor is stationary

Possible causes include:

- the IMU was moved during startup calibration;
- gyroscope bias changed with temperature;
- the program is running in 6-DOF mode and yaw is drifting.

Try:

1. Place the IMU on a stationary surface.
2. Press `R`.
3. Do not move the board until recalibration completes.

If you have a magnetometer, verify that `1e` appears in:

```bash
i2cdetect -y 1
```

---

## Rotation is noisy or unstable

Check that the sensor is configured using the expected ranges and that the board wiring is reliable.

You can also tune:

```python
MADGWICK_BETA = 0.08
```

Do not change the scaling constants unless the corresponding LSM6DS33 range configuration is changed as well.

---

# 16. Security note

Do not commit a real Raspberry Pi password to a public Git repository.

Prefer:

```bash
export RPI_HOST=192.168.8.235
export RPI_USER=gleb
export RPI_PASSWORD='your_password'
```

For a longer-term project, SSH key authentication is preferable to storing passwords in code.

---

# 17. Quick-start summary

### Raspberry Pi

```bash
sudo apt update
sudo apt install -y i2c-tools
i2cdetect -y 1
```

Confirm that `6b` is visible.

### Ubuntu WSL

```bash
sudo apt update
sudo apt install -y \
    python3 python3-pip python3-venv \
    freeglut3-dev libgl1-mesa-dev libglu1-mesa-dev \
    mesa-utils openssh-client

mkdir -p ~/robt402
cd ~/robt402
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install paramiko "PyOpenGL==3.1.10"
```

Verify graphics:

```bash
glxinfo -B
```

Set the Pi connection:

```bash
export RPI_HOST=192.168.8.235
export RPI_USER=gleb
export RPI_PASSWORD='your_raspberry_pi_password'
```

Run:

```bash
python3 imu_3d_wsl_opengl_fixed.py
```

Keep the IMU stationary during startup calibration, then press `Z` and rotate the physical sensor.

---

## Project files

```text
README.md
imu_3d_wsl_opengl_fixed.py
```

