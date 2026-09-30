#!/usr/bin/env python3
"""
ROBT 402 - Ubuntu/WSL SSH IMU quaternion + OpenGL cube

This program is meant to run inside Ubuntu under WSL/WSLg.
The Raspberry Pi remains connected to the physical IMU.  This program:
  1. connects from WSL to the Raspberry Pi over SSH;
  2. configures/reads the LSM6DS33 over the Pi's I2C bus;
  3. estimates orientation with a Madgwick filter;
  4. prints quaternion orientation to the WSL terminal; and
  5. displays a real-time quaternion-driven 3D cube in a FreeGLUT/OpenGL window.

Keyboard controls in the OpenGL window:
  Z  zero the current orientation
  X  clear the zero/reference orientation
  R  recalibrate gyro bias (keep the IMU still)
  I  invert cube rotation if the visual direction is backwards
  Q / Esc  quit

Recommended Ubuntu/WSL setup:
    sudo apt update
    sudo apt install -y python3 python3-venv python3-pip \
        freeglut3-dev libgl1-mesa-dev libglu1-mesa-dev mesa-utils openssh-client

    python3 -m venv .venv
    source .venv/bin/activate
    pip install --upgrade pip
    pip install paramiko PyOpenGL

Run:
    python3 imu_3d_wsl_opengl.py 192.168.8.235

Optional environment variables:
    export RPI_USER=gleb
    export RPI_PASSWORD=robot

If no host argument is supplied, RPI_HOST is used, with 192.168.8.235
as the final default.

Keep the IMU still for the first ~2 seconds while gyro bias is calibrated.
"""

import argparse
import math
import os

# WSLg exposes both Wayland and X11.  PyOpenGL/FreeGLUT can otherwise
# disagree about which backend owns the current context, producing:
#   OpenGL.error.Error: Attempt to retrieve context when no valid context
# Force both sides onto the X11/GLX path before importing OpenGL.
os.environ["PYOPENGL_PLATFORM"] = "glx"
os.environ.pop("WAYLAND_DISPLAY", None)

import queue
import sys
import threading
import time
from datetime import datetime

import paramiko

try:
    from OpenGL.GL import (
        GL_COLOR_BUFFER_BIT,
        GL_DEPTH_BUFFER_BIT,
        GL_DEPTH_TEST,
        GL_LINES,
        GL_MODELVIEW,
        GL_PROJECTION,
        GL_QUADS,
        glBegin,
        glClear,
        glClearColor,
        glColor3f,
        glEnable,
        glEnd,
        glLoadIdentity,
        glMatrixMode,
        glMultMatrixf,
        glPopMatrix,
        glPushMatrix,
        glRasterPos2f,
        glVertex3f,
        glViewport,
    )
    from OpenGL.GLU import gluLookAt, gluPerspective
    import OpenGL.GLUT as GLUT
    from OpenGL.GLUT import (
        GLUT_BITMAP_8_BY_13,
        GLUT_DEPTH,
        GLUT_DOUBLE,
        GLUT_RGB,
        glutBitmapCharacter,
        glutCreateWindow,
        glutDisplayFunc,
        glutInit,
        glutInitDisplayMode,
        glutInitWindowPosition,
        glutInitWindowSize,
        glutKeyboardFunc,
        glutMainLoop,
        glutPostRedisplay,
        glutReshapeFunc,
        glutSwapBuffers,
        glutTimerFunc,
    )
except Exception as exc:
    raise SystemExit(
        "PyOpenGL/FreeGLUT could not be loaded. Install the WSL dependencies shown "
        "at the top of this file. Original error: " + str(exc)
    )

# ============================================================================
# Raspberry Pi / SSH settings
# ============================================================================

DEFAULT_HOST = os.getenv("RPI_HOST", "192.168.8.235")
HOST = DEFAULT_HOST
USERNAME = os.getenv("RPI_USER", "gleb")
PASSWORD = os.getenv("RPI_PASSWORD", "robot")
SSH_PORT = 22

REMOTE_PATH = "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
I2C_BUS = 1
IMU_ADDR = "0x6b"
MAG_ADDR = "0x1e"


# ============================================================================
# LSM6DS33 configuration
# ============================================================================
# Same ranges as the user's working program:
#   Gyroscope:     +/-500 deg/s -> 17.50 mdps/LSB
#   Accelerometer: +/-2 g       -> 0.061 mg/LSB
# Registers:
#   CTRL1_XL = 0x20 -> accel 26 Hz, +/-2 g
#   CTRL2_G  = 0x24 -> gyro  26 Hz, +/-500 dps
#   CTRL3_C  = 0x44 -> BDU=1, IF_INC=1 for safe multi-byte reads

GYRO_DPS_PER_LSB = 0.0175
ACCEL_G_PER_LSB = 0.000061
GRAVITY = 9.80665

CTRL1_XL_VALUE = "0x20"
CTRL2_G_VALUE = "0x24"
CTRL3_C_VALUE = "0x44"

# LIS3MDL common configuration for MinIMU-9 style boards:
#   CTRL_REG1 0x20 = 0x70 -> ultra-high-performance XY, 80 Hz ODR
#   CTRL_REG2 0x21 = 0x00 -> +/-4 gauss
#   CTRL_REG3 0x22 = 0x00 -> continuous conversion
#   CTRL_REG4 0x23 = 0x0C -> ultra-high-performance Z
# At +/-4 gauss the nominal sensitivity is 6842 LSB/gauss. Madgwick normalizes
# the magnetic vector, so exact magnetic units are not required for fusion.
MAG_GAUSS_PER_LSB = 1.0 / 6842.0

# Madgwick gain. Larger = stronger accel/mag correction, but noisier.
MADGWICK_BETA = 0.08

# At 26 Hz ODR, 50 samples is about 2 seconds.
GYRO_CALIBRATION_SAMPLES = 50

# Polling period. The sensor itself is configured for 26 Hz.
REMOTE_SLEEP_SECONDS = 0.035


# ============================================================================
# Thread communication / lifecycle
# ============================================================================

data_queue = queue.Queue()
stop_event = threading.Event()
recalibrate_event = threading.Event()
ssh_holder = {"client": None}


def debug(message):
    timestamp = datetime.now().strftime("%H:%M:%S")
    text = f"[{timestamp}] {message}"
    print(text, flush=True)
    data_queue.put(("log", text))


def signed_16bit(low, high):
    value = (high << 8) | low
    if value & 0x8000:
        value -= 65536
    return value


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


# ============================================================================
# Quaternion math
# Quaternion format everywhere in this program: (w, x, y, z)
# ============================================================================


def quat_normalize(q):
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    return (w / n, x / n, y / n, z / n)


def quat_conjugate(q):
    w, x, y, z = q
    return (w, -x, -y, -z)


def quat_multiply(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quat_to_matrix(q):
    w, x, y, z = quat_normalize(q)
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
    )


def rotate_vector(q, v):
    r = quat_to_matrix(q)
    x, y, z = v
    return (
        r[0][0] * x + r[0][1] * y + r[0][2] * z,
        r[1][0] * x + r[1][1] * y + r[1][2] * z,
        r[2][0] * x + r[2][1] * y + r[2][2] * z,
    )


def quaternion_to_euler_deg(q):
    """Display-only roll/pitch/yaw in degrees."""
    w, x, y, z = quat_normalize(q)

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return tuple(math.degrees(a) for a in (roll, pitch, yaw))


# ============================================================================
# Madgwick IMU filter (accelerometer + gyroscope)
# ============================================================================


class MadgwickAHRS:
    def __init__(self, beta=MADGWICK_BETA):
        self.beta = beta
        self.q = (1.0, 0.0, 0.0, 0.0)

    def reset(self):
        self.q = (1.0, 0.0, 0.0, 0.0)

    def _integrate(self, q_dot, dt):
        q1, q2, q3, q4 = self.q
        q1 += q_dot[0] * dt
        q2 += q_dot[1] * dt
        q3 += q_dot[2] * dt
        q4 += q_dot[3] * dt
        self.q = quat_normalize((q1, q2, q3, q4))
        return self.q

    def update_imu(self, gx_dps, gy_dps, gz_dps, ax, ay, az, dt):
        """Madgwick 6-DOF update: gyro + accelerometer."""
        dt = clamp(dt, 0.001, 0.2)

        gx = math.radians(gx_dps)
        gy = math.radians(gy_dps)
        gz = math.radians(gz_dps)

        q1, q2, q3, q4 = self.q  # q1=w, q2=x, q3=y, q4=z

        q_dot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz)
        q_dot2 = 0.5 * ( q1 * gx + q3 * gz - q4 * gy)
        q_dot3 = 0.5 * ( q1 * gy - q2 * gz + q4 * gx)
        q_dot4 = 0.5 * ( q1 * gz + q2 * gy - q3 * gx)

        norm_a = math.sqrt(ax * ax + ay * ay + az * az)
        if norm_a > 1e-9:
            ax /= norm_a
            ay /= norm_a
            az /= norm_a

            _2q1 = 2.0 * q1
            _2q2 = 2.0 * q2
            _2q3 = 2.0 * q3
            _2q4 = 2.0 * q4
            _4q1 = 4.0 * q1
            _4q2 = 4.0 * q2
            _4q3 = 4.0 * q3
            _8q2 = 8.0 * q2
            _8q3 = 8.0 * q3
            q1q1 = q1 * q1
            q2q2 = q2 * q2
            q3q3 = q3 * q3
            q4q4 = q4 * q4

            s1 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q2q2 - _2q2 * ay
            s2 = (
                _4q2 * q4q4 - _2q4 * ax + 4.0 * q1q1 * q2
                - _2q1 * ay - _4q2 + _8q2 * q2q2
                + _8q2 * q3q3 + _4q2 * az
            )
            s3 = (
                4.0 * q1q1 * q3 + _2q1 * ax + _4q3 * q4q4
                - _2q4 * ay - _4q3 + _8q3 * q2q2
                + _8q3 * q3q3 + _4q3 * az
            )
            s4 = 4.0 * q2q2 * q4 - _2q2 * ax + 4.0 * q3q3 * q4 - _2q3 * ay

            norm_s = math.sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)
            if norm_s > 1e-12:
                s1 /= norm_s
                s2 /= norm_s
                s3 /= norm_s
                s4 /= norm_s

                q_dot1 -= self.beta * s1
                q_dot2 -= self.beta * s2
                q_dot3 -= self.beta * s3
                q_dot4 -= self.beta * s4

        return self._integrate((q_dot1, q_dot2, q_dot3, q_dot4), dt)

    def update_marg(
        self,
        gx_dps, gy_dps, gz_dps,
        ax, ay, az,
        mx, my, mz,
        dt,
    ):
        """Madgwick 9-DOF MARG update: gyro + accelerometer + magnetometer."""
        norm_m = math.sqrt(mx * mx + my * my + mz * mz)
        if norm_m < 1e-12:
            return self.update_imu(gx_dps, gy_dps, gz_dps, ax, ay, az, dt)

        dt = clamp(dt, 0.001, 0.2)
        gx = math.radians(gx_dps)
        gy = math.radians(gy_dps)
        gz = math.radians(gz_dps)

        q1, q2, q3, q4 = self.q

        q_dot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz)
        q_dot2 = 0.5 * ( q1 * gx + q3 * gz - q4 * gy)
        q_dot3 = 0.5 * ( q1 * gy - q2 * gz + q4 * gx)
        q_dot4 = 0.5 * ( q1 * gz + q2 * gy - q3 * gx)

        norm_a = math.sqrt(ax * ax + ay * ay + az * az)
        if norm_a < 1e-12:
            return self._integrate((q_dot1, q_dot2, q_dot3, q_dot4), dt)

        ax /= norm_a
        ay /= norm_a
        az /= norm_a
        mx /= norm_m
        my /= norm_m
        mz /= norm_m

        _2q1mx = 2.0 * q1 * mx
        _2q1my = 2.0 * q1 * my
        _2q1mz = 2.0 * q1 * mz
        _2q2mx = 2.0 * q2 * mx
        _2q1 = 2.0 * q1
        _2q2 = 2.0 * q2
        _2q3 = 2.0 * q3
        _2q4 = 2.0 * q4
        _2q1q3 = 2.0 * q1 * q3
        _2q3q4 = 2.0 * q3 * q4
        q1q1 = q1 * q1
        q1q2 = q1 * q2
        q1q3 = q1 * q3
        q1q4 = q1 * q4
        q2q2 = q2 * q2
        q2q3 = q2 * q3
        q2q4 = q2 * q4
        q3q3 = q3 * q3
        q3q4 = q3 * q4
        q4q4 = q4 * q4

        hx = (
            mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2
            + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4
        )
        hy = (
            _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3
            - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4
        )
        _2bx = math.sqrt(hx * hx + hy * hy)
        _2bz = (
            -_2q1mx * q3 + _2q1my * q2 + mz * q1q1
            + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4
            - mz * q3q3 + mz * q4q4
        )
        _4bx = 2.0 * _2bx
        _4bz = 2.0 * _2bz

        f1 = 2.0 * q2q4 - _2q1q3 - ax
        f2 = 2.0 * q1q2 + _2q3q4 - ay
        f3 = 1.0 - 2.0 * q2q2 - 2.0 * q3q3 - az
        f4 = _2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx
        f5 = _2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my
        f6 = _2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz

        s1 = -_2q3 * f1 + _2q2 * f2 - _2bz * q3 * f4 + (-_2bx * q4 + _2bz * q2) * f5 + _2bx * q3 * f6
        s2 = _2q4 * f1 + _2q1 * f2 - 4.0 * q2 * f3 + _2bz * q4 * f4 + (_2bx * q3 + _2bz * q1) * f5 + (_2bx * q4 - _4bz * q2) * f6
        s3 = -_2q1 * f1 + _2q4 * f2 - 4.0 * q3 * f3 + (-_4bx * q3 - _2bz * q1) * f4 + (_2bx * q2 + _2bz * q4) * f5 + (_2bx * q1 - _4bz * q3) * f6
        s4 = _2q2 * f1 + _2q3 * f2 + (-_4bx * q4 + _2bz * q2) * f4 + (-_2bx * q1 + _2bz * q3) * f5 + _2bx * q2 * f6

        norm_s = math.sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)
        if norm_s > 1e-12:
            s1 /= norm_s
            s2 /= norm_s
            s3 /= norm_s
            s4 /= norm_s
            q_dot1 -= self.beta * s1
            q_dot2 -= self.beta * s2
            q_dot3 -= self.beta * s3
            q_dot4 -= self.beta * s4

        return self._integrate((q_dot1, q_dot2, q_dot3, q_dot4), dt)


# ============================================================================
# Remote command helper
# ============================================================================


def run_command(ssh, command, quiet=False):
    full_command = f"export PATH={REMOTE_PATH}; {command}"
    if not quiet:
        debug(f"RUN: {full_command}")

    stdin, stdout, stderr = ssh.exec_command(full_command)
    exit_code = stdout.channel.recv_exit_status()
    out = stdout.read().decode(errors="replace").strip()
    err = stderr.read().decode(errors="replace").strip()

    if not quiet:
        if out:
            debug(f"STDOUT: {out}")
        if err:
            debug(f"STDERR: {err}")
        debug(f"Exit code: {exit_code}")

    return exit_code, out, err


# ============================================================================
# SSH worker
# ============================================================================


def build_fallback_read_command():
    regs = [
        "0x22", "0x23", "0x24", "0x25", "0x26", "0x27",
        "0x28", "0x29", "0x2a", "0x2b", "0x2c", "0x2d",
    ]
    gets = " ".join(
        f"$(i2cget -y {I2C_BUS} {IMU_ADDR} {reg})" for reg in regs
    )
    return (
        f"export PATH={REMOTE_PATH}; "
        f"while true; do echo \"{gets}\"; sleep 0.08; done"
    )


def build_block_read_command(use_magnetometer):
    # One coherent block read from the LSM6DS33, plus an optional LIS3MDL read.
    if use_magnetometer:
        return (
            f"export PATH={REMOTE_PATH}; "
            f"while true; do "
            f"g=$(i2ctransfer -y {I2C_BUS} w1@{IMU_ADDR} 0x22 r12) || exit 1; "
            f"m=$(i2ctransfer -y {I2C_BUS} w1@{MAG_ADDR} 0xa8 r6) || exit 1; "
            f'echo "$g $m"; '
            f"sleep {REMOTE_SLEEP_SECONDS}; "
            f"done"
        )
    return (
        f"export PATH={REMOTE_PATH}; "
        f"while true; do "
        f"i2ctransfer -y {I2C_BUS} w1@{IMU_ADDR} 0x22 r12; "
        f"sleep {REMOTE_SLEEP_SECONDS}; "
        f"done"
    )


def ssh_worker():
    ssh = None
    madgwick = MadgwickAHRS()

    try:
        data_queue.put(("status", "Connecting to Raspberry Pi..."))
        debug(f"Connecting to {USERNAME}@{HOST}:{SSH_PORT} ...")

        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(
            HOST,
            port=SSH_PORT,
            username=USERNAME,
            password=PASSWORD,
            look_for_keys=False,
            allow_agent=False,
            timeout=10,
            banner_timeout=10,
            auth_timeout=10,
        )
        ssh_holder["client"] = ssh

        debug("SSH connected.")
        data_queue.put(("status", f"Connected: {USERNAME}@{HOST}"))

        # Basic tests.
        run_command(ssh, "echo SSH_TEST_OK")
        run_command(ssh, "hostname")
        run_command(ssh, f"ls -l /dev/i2c-{I2C_BUS}")

        # Check the IMU address.
        code, output, _ = run_command(ssh, f"i2cdetect -y {I2C_BUS}")
        if code != 0 or "6b" not in output.lower():
            raise RuntimeError("LSM6DS33 at I2C address 0x6B was not detected.")

        _, whoami, _ = run_command(
            ssh, f"i2cget -y {I2C_BUS} {IMU_ADDR} 0x0f"
        )
        debug(f"WHO_AM_I = {whoami} (LSM6DS33 is normally 0x69)")

        # Configure IMU. These match the scaling constants above.
        run_command(ssh, f"i2cset -y {I2C_BUS} {IMU_ADDR} 0x10 {CTRL1_XL_VALUE}")
        run_command(ssh, f"i2cset -y {I2C_BUS} {IMU_ADDR} 0x11 {CTRL2_G_VALUE}")
        run_command(ssh, f"i2cset -y {I2C_BUS} {IMU_ADDR} 0x12 {CTRL3_C_VALUE}")

        # Check for the common companion LIS3MDL magnetometer at 0x1E.
        has_magnetometer = "1e" in output.lower()
        if has_magnetometer:
            debug("LIS3MDL magnetometer detected at 0x1E; enabling 9-DOF fusion.")
            run_command(ssh, f"i2cset -y {I2C_BUS} {MAG_ADDR} 0x20 0x70")
            run_command(ssh, f"i2cset -y {I2C_BUS} {MAG_ADDR} 0x21 0x00")
            run_command(ssh, f"i2cset -y {I2C_BUS} {MAG_ADDR} 0x22 0x00")
            run_command(ssh, f"i2cset -y {I2C_BUS} {MAG_ADDR} 0x23 0x0c")
            data_queue.put(("fusion_mode", "9DOF"))
        else:
            debug("No magnetometer detected at 0x1E; using 6-DOF fusion.")
            data_queue.put(("fusion_mode", "6DOF"))

        # Prefer i2ctransfer for coherent block reads.
        has_i2ctransfer, _, _ = run_command(
            ssh, "command -v i2ctransfer >/dev/null 2>&1", quiet=True
        )
        if has_i2ctransfer == 0:
            stream_command = build_block_read_command(has_magnetometer)
            debug("Using i2ctransfer block reads.")
        else:
            if has_magnetometer:
                debug("i2ctransfer is required for the 9-DOF stream; falling back to 6-DOF.")
                has_magnetometer = False
                data_queue.put(("fusion_mode", "6DOF"))
            stream_command = build_fallback_read_command()
            debug("i2ctransfer not found; using slower i2cget fallback.")

        stdin, stdout, stderr = ssh.exec_command(stream_command)
        debug("Remote IMU stream started.")

        bias_sum = [0.0, 0.0, 0.0]
        bias_count = 0
        gyro_bias = [0.0, 0.0, 0.0]
        calibrated = False
        last_time = None
        sample_counter = 0

        data_queue.put((
            "status",
            f"Keep IMU still: calibrating gyro 0/{GYRO_CALIBRATION_SAMPLES}",
        ))

        for line in iter(stdout.readline, ""):
            if stop_event.is_set():
                break

            line = line.strip()
            if not line:
                continue

            parts = line.split()
            expected_fields = 18 if has_magnetometer else 12
            if len(parts) != expected_fields:
                # Some shells/tools can emit occasional extra text. Ignore it.
                if sample_counter % 20 == 0:
                    debug(f"Ignored line with {len(parts)} fields: {line}")
                continue

            try:
                values = [int(v, 16) for v in parts]
            except ValueError:
                continue

            gx_raw = signed_16bit(values[0], values[1])
            gy_raw = signed_16bit(values[2], values[3])
            gz_raw = signed_16bit(values[4], values[5])
            ax_raw = signed_16bit(values[6], values[7])
            ay_raw = signed_16bit(values[8], values[9])
            az_raw = signed_16bit(values[10], values[11])

            if has_magnetometer:
                mx_raw = signed_16bit(values[12], values[13])
                my_raw = signed_16bit(values[14], values[15])
                mz_raw = signed_16bit(values[16], values[17])
                mx = mx_raw * MAG_GAUSS_PER_LSB
                my = my_raw * MAG_GAUSS_PER_LSB
                mz = mz_raw * MAG_GAUSS_PER_LSB
            else:
                mx_raw = my_raw = mz_raw = 0
                mx = my = mz = 0.0

            gx_uncal = gx_raw * GYRO_DPS_PER_LSB
            gy_uncal = gy_raw * GYRO_DPS_PER_LSB
            gz_uncal = gz_raw * GYRO_DPS_PER_LSB

            ax = ax_raw * ACCEL_G_PER_LSB * GRAVITY
            ay = ay_raw * ACCEL_G_PER_LSB * GRAVITY
            az = az_raw * ACCEL_G_PER_LSB * GRAVITY

            # Recalibration can be requested from the GUI.
            if recalibrate_event.is_set():
                recalibrate_event.clear()
                bias_sum = [0.0, 0.0, 0.0]
                bias_count = 0
                gyro_bias = [0.0, 0.0, 0.0]
                calibrated = False
                madgwick.reset()
                last_time = None
                data_queue.put((
                    "status",
                    f"Keep IMU still: calibrating gyro 0/{GYRO_CALIBRATION_SAMPLES}",
                ))
                debug("Gyro recalibration started.")

            if not calibrated:
                bias_sum[0] += gx_uncal
                bias_sum[1] += gy_uncal
                bias_sum[2] += gz_uncal
                bias_count += 1

                data_queue.put(("data", {
                    "gx": gx_uncal,
                    "gy": gy_uncal,
                    "gz": gz_uncal,
                    "ax": ax,
                    "ay": ay,
                    "az": az,
                    "gx_raw": gx_raw,
                    "gy_raw": gy_raw,
                    "gz_raw": gz_raw,
                    "ax_raw": ax_raw,
                    "ay_raw": ay_raw,
                    "az_raw": az_raw,
                    "mx": mx, "my": my, "mz": mz,
                    "mx_raw": mx_raw, "my_raw": my_raw, "mz_raw": mz_raw,
                    "q": madgwick.q,
                    "calibrating": True,
                    "calibration_count": bias_count,
                }))

                if bias_count >= GYRO_CALIBRATION_SAMPLES:
                    gyro_bias = [s / bias_count for s in bias_sum]
                    calibrated = True
                    last_time = time.monotonic()
                    debug(
                        "Gyro bias calibrated: "
                        f"X={gyro_bias[0]:.4f}, "
                        f"Y={gyro_bias[1]:.4f}, "
                        f"Z={gyro_bias[2]:.4f} deg/s"
                    )
                    data_queue.put(("status", "Streaming IMU orientation"))
                else:
                    if bias_count % 5 == 0:
                        data_queue.put((
                            "status",
                            f"Keep IMU still: calibrating gyro "
                            f"{bias_count}/{GYRO_CALIBRATION_SAMPLES}",
                        ))
                continue

            gx = gx_uncal - gyro_bias[0]
            gy = gy_uncal - gyro_bias[1]
            gz = gz_uncal - gyro_bias[2]

            now = time.monotonic()
            dt = now - last_time if last_time is not None else 1.0 / 26.0
            last_time = now

            if has_magnetometer:
                q = madgwick.update_marg(gx, gy, gz, ax, ay, az, mx, my, mz, dt)
            else:
                q = madgwick.update_imu(gx, gy, gz, ax, ay, az, dt)

            sample_counter += 1
            if sample_counter % 100 == 1:
                debug(
                    f"Sample q=({q[0]:.3f}, {q[1]:.3f}, "
                    f"{q[2]:.3f}, {q[3]:.3f})"
                )

            data_queue.put(("data", {
                "gx": gx,
                "gy": gy,
                "gz": gz,
                "ax": ax,
                "ay": ay,
                "az": az,
                "gx_raw": gx_raw,
                "gy_raw": gy_raw,
                "gz_raw": gz_raw,
                "ax_raw": ax_raw,
                "ay_raw": ay_raw,
                "az_raw": az_raw,
                "mx": mx, "my": my, "mz": mz,
                "mx_raw": mx_raw, "my_raw": my_raw, "mz_raw": mz_raw,
                "q": q,
                "calibrating": False,
                "calibration_count": GYRO_CALIBRATION_SAMPLES,
            }))

        if not stop_event.is_set():
            remote_error = stderr.read().decode(errors="replace").strip()
            if remote_error:
                debug(f"Remote stream STDERR: {remote_error}")
            raise RuntimeError("Remote IMU stream ended unexpectedly.")

    except paramiko.AuthenticationException:
        debug("SSH authentication failed. Check username/password.")
        data_queue.put(("status", "SSH authentication failed"))
        data_queue.put(("error", "SSH authentication failed."))
    except Exception as exc:
        debug(f"ERROR: {type(exc).__name__}: {exc}")
        data_queue.put(("status", f"Error: {exc}"))
        data_queue.put(("error", str(exc)))
    finally:
        ssh_holder["client"] = None
        if ssh is not None:
            try:
                ssh.close()
            except Exception:
                pass
        debug("SSH worker finished.")



# ============================================================================
# OpenGL visualization state
# ============================================================================

latest_sensor_q = (1.0, 0.0, 0.0, 0.0)
zero_q = (1.0, 0.0, 0.0, 0.0)
use_conjugate = False
latest_status = "Starting..."
fusion_mode = "unknown"
latest_data = None
last_terminal_print = 0.0
window_width = 1000
window_height = 760


def ensure_wsl_gui_available():
    """Give a clear error if WSL has no WSLg/X display."""
    if not sys.platform.startswith("linux"):
        return
    if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        raise SystemExit(
            "No Linux GUI display was detected.\n\n"
            "From Windows PowerShell run:\n"
            "    wsl --update\n"
            "    wsl --shutdown\n\n"
            "Then reopen Ubuntu and check:\n"
            "    echo $DISPLAY\n"
            "    glxinfo -B\n"
        )


def display_quaternion():
    q_rel = quat_multiply(quat_conjugate(zero_q), latest_sensor_q)
    q_rel = quat_normalize(q_rel)
    if use_conjugate:
        q_rel = quat_conjugate(q_rel)
    return q_rel


def quaternion_to_gl_matrix(q):
    """Return a column-major 4x4 OpenGL rotation matrix."""
    r = quat_to_matrix(q)
    return (
        r[0][0], r[1][0], r[2][0], 0.0,
        r[0][1], r[1][1], r[2][1], 0.0,
        r[0][2], r[1][2], r[2][2], 0.0,
        0.0,     0.0,     0.0,     1.0,
    )


def draw_axes(length=2.5, line_scale=1.0):
    glBegin(GL_LINES)
    glColor3f(0.90, 0.20, 0.20)
    glVertex3f(0.0, 0.0, 0.0)
    glVertex3f(length * line_scale, 0.0, 0.0)

    glColor3f(0.20, 0.80, 0.30)
    glVertex3f(0.0, 0.0, 0.0)
    glVertex3f(0.0, length * line_scale, 0.0)

    glColor3f(0.25, 0.45, 0.95)
    glVertex3f(0.0, 0.0, 0.0)
    glVertex3f(0.0, 0.0, length * line_scale)
    glEnd()


def draw_grid():
    glColor3f(0.38, 0.38, 0.38)
    glBegin(GL_LINES)
    for i in range(-5, 6):
        v = i * 0.5
        glVertex3f(-2.5, v, -0.7)
        glVertex3f(2.5, v, -0.7)
        glVertex3f(v, -2.5, -0.7)
        glVertex3f(v, 2.5, -0.7)
    glEnd()


def draw_imu_body():
    """Draw a rectangular prism so orientation is easier to see than a cube."""
    x = 1.35
    y = 0.85
    z = 0.30

    vertices = [
        (-x, -y, -z), (x, -y, -z), (x, y, -z), (-x, y, -z),
        (-x, -y,  z), (x, -y,  z), (x, y,  z), (-x, y,  z),
    ]
    faces = [
        (0, 1, 2, 3),
        (4, 5, 6, 7),
        (0, 1, 5, 4),
        (2, 3, 7, 6),
        (1, 2, 6, 5),
        (0, 3, 7, 4),
    ]
    colors = [
        (0.55, 0.62, 0.72),
        (0.72, 0.78, 0.86),
        (0.42, 0.50, 0.62),
        (0.64, 0.70, 0.80),
        (0.48, 0.57, 0.68),
        (0.60, 0.67, 0.76),
    ]

    glBegin(GL_QUADS)
    for face, color in zip(faces, colors):
        glColor3f(*color)
        for idx in face:
            glVertex3f(*vertices[idx])
    glEnd()

    # IMU/body axes rotate with the body.
    draw_axes(length=1.9)


def draw_text_line(x, y, text):
    glRasterPos2f(x, y)
    for ch in text:
        glutBitmapCharacter(GLUT_BITMAP_8_BY_13, ord(ch))


def draw_overlay():
    q = display_quaternion()

    glMatrixMode(GL_PROJECTION)
    glPushMatrix()
    glLoadIdentity()
    glMatrixMode(GL_MODELVIEW)
    glPushMatrix()
    glLoadIdentity()

    glColor3f(0.95, 0.95, 0.95)
    draw_text_line(-0.97, 0.93, f"Status: {latest_status}")
    draw_text_line(-0.97, 0.87, f"Fusion: {fusion_mode}")
    draw_text_line(
        -0.97,
        0.81,
        f"q = ({q[0]: .4f}, {q[1]: .4f}, {q[2]: .4f}, {q[3]: .4f})",
    )
    draw_text_line(-0.97, -0.91, "Z zero | X clear zero | R recalibrate | I invert | Q/Esc quit")

    glPopMatrix()
    glMatrixMode(GL_PROJECTION)
    glPopMatrix()
    glMatrixMode(GL_MODELVIEW)


def process_queue():
    global latest_sensor_q, latest_status, fusion_mode, latest_data
    global last_terminal_print

    try:
        while True:
            msg_type, message = data_queue.get_nowait()

            if msg_type == "status":
                latest_status = message
            elif msg_type == "fusion_mode":
                fusion_mode = "9-DOF gyro+accel+mag" if message == "9DOF" else "6-DOF gyro+accel"
            elif msg_type == "data":
                latest_data = message
                latest_sensor_q = message["q"]

                # Task 1-style quaternion output in the terminal, throttled so
                # the terminal remains readable during the live demo.
                now = time.monotonic()
                if not message.get("calibrating", False) and now - last_terminal_print >= 0.20:
                    q = display_quaternion()
                    print(
                        f"Quaternion w x y z: "
                        f"{q[0]: .6f} {q[1]: .6f} {q[2]: .6f} {q[3]: .6f}",
                        flush=True,
                    )
                    last_terminal_print = now
            # 'log' is already printed by debug(); 'error' is reflected in status.
    except queue.Empty:
        pass


def display():
    process_queue()
    q = display_quaternion()

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    gluLookAt(
        4.7, -6.2, 4.3,
        0.0, 0.0, 0.0,
        0.0, 0.0, 1.0,
    )

    draw_grid()

    # Fixed world axes.
    draw_axes(length=2.7)

    # Quaternion-controlled IMU body and body axes.
    glPushMatrix()
    glMultMatrixf(quaternion_to_gl_matrix(q))
    draw_imu_body()
    glPopMatrix()

    draw_overlay()
    glutSwapBuffers()


def reshape(width, height):
    global window_width, window_height
    window_width = max(1, width)
    window_height = max(1, height)

    glViewport(0, 0, window_width, window_height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45.0, window_width / float(window_height), 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)


def timer(_value):
    process_queue()
    glutPostRedisplay()
    if not stop_event.is_set():
        glutTimerFunc(16, timer, 0)


def keyboard(key, _x, _y):
    global zero_q, use_conjugate, latest_status

    if isinstance(key, bytes):
        key = key.decode("utf-8", errors="ignore")

    key_lower = key.lower()

    if key == "\x1b" or key_lower == "q":
        request_exit()
    elif key_lower == "z":
        zero_q = latest_sensor_q
        latest_status = "Display orientation zeroed at current pose"
        print("Display orientation zeroed.", flush=True)
    elif key_lower == "x":
        zero_q = (1.0, 0.0, 0.0, 0.0)
        latest_status = "Display zero/reference cleared"
        print("Display zero cleared.", flush=True)
    elif key_lower == "r":
        recalibrate_event.set()
        latest_status = "Recalibration requested - keep the IMU still"
        print("Gyro recalibration requested; keep IMU still.", flush=True)
    elif key_lower == "i":
        use_conjugate = not use_conjugate
        latest_status = (
            "Cube rotation inverted" if use_conjugate else "Normal cube rotation restored"
        )
        print(latest_status, flush=True)

    glutPostRedisplay()


def request_exit():
    stop_event.set()
    client = ssh_holder.get("client")
    if client is not None:
        try:
            client.close()
        except Exception:
            pass

    try:
        leave_main_loop = getattr(GLUT, "glutLeaveMainLoop", None)
        if leave_main_loop is not None:
            leave_main_loop()
        else:
            os._exit(0)
    except Exception:
        os._exit(0)


def close_callback():
    request_exit()


def init_opengl():
    glClearColor(0.07, 0.08, 0.11, 1.0)
    glEnable(GL_DEPTH_TEST)


def parse_arguments():
    """Parse this program's arguments and leave unknown GLUT options alone."""
    parser = argparse.ArgumentParser(
        description="Visualize Raspberry Pi IMU orientation over SSH in WSL/OpenGL."
    )
    parser.add_argument(
        "host",
        nargs="?",
        default=DEFAULT_HOST,
        help=(
            "Raspberry Pi host/IP address, e.g. 192.168.8.235 "
            f"(default: {DEFAULT_HOST})"
        ),
    )
    args, glut_args = parser.parse_known_args()
    return args, [sys.argv[0], *glut_args]


def main():
    global HOST

    args, glut_argv = parse_arguments()
    HOST = args.host

    ensure_wsl_gui_available()

    # Create the OpenGL/GLUT context first, on the main thread.  Only after
    # the window and callbacks exist do we start the SSH worker.  This keeps
    # all OpenGL context creation/registration isolated to the main thread.
    glutInit(glut_argv)
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
    glutInitWindowSize(window_width, window_height)
    glutInitWindowPosition(80, 60)
    glutCreateWindow(b"ROBT 402 - WSL IMU Quaternion 3D Visualization")

    # Ask FreeGLUT to return from glutMainLoop when the window closes.
    try:
        set_option = getattr(GLUT, "glutSetOption", None)
        action_on_close = getattr(GLUT, "GLUT_ACTION_ON_WINDOW_CLOSE", None)
        returns = getattr(GLUT, "GLUT_ACTION_GLUTMAINLOOP_RETURNS", None)
        if set_option is not None and action_on_close is not None and returns is not None:
            set_option(action_on_close, returns)
    except Exception:
        pass

    init_opengl()
    glutDisplayFunc(display)
    glutReshapeFunc(reshape)
    glutKeyboardFunc(keyboard)
    try:
        close_func = getattr(GLUT, "glutCloseFunc", None)
        if close_func is not None:
            close_func(close_callback)
    except Exception:
        pass
    glutTimerFunc(16, timer, 0)

    worker = threading.Thread(target=ssh_worker, daemon=True)
    worker.start()

    print(f"OpenGL backend: GLX/X11 (DISPLAY={os.getenv('DISPLAY', '<unset>')})", flush=True)
    print("OpenGL visualization started.", flush=True)
    print("Keyboard: Z zero | X clear zero | R recalibrate | I invert | Q/Esc quit", flush=True)
    print(f"Connecting to Raspberry Pi at {USERNAME}@{HOST}:{SSH_PORT} ...", flush=True)

    glutMainLoop()
    stop_event.set()


if __name__ == "__main__":
    main()
