# Tests & Diagnostic Tools

This directory contains test scripts and diagnostic tools for **pib3 framework developers**.

These scripts are used to verify framework functionality and troubleshoot issues. They are not intended as user examples to build upon.

## Directory Structure

```
tests/
├── README.md                       # This file
├── diagnose_connection.py          # Connection troubleshooting tool
├── read_servo_calibration.py       # Read servo hardware calibration
├── test_low_latency.py             # Tests direct Tinkerforge motor control
├── test_tinkerforge_optimization.py # Unit tests for Tinkerforge internals
└── ...
```

## Running Tests & Tools

### Connection Diagnostic

Troubleshoots connection issues to the robot:

```bash
python tests/diagnose_connection.py --host 172.26.34.149
python tests/diagnose_connection.py --host 172.26.34.149 --port 9090
```

Checks:
1. Network ping
2. TCP port connectivity
3. Rosbridge WebSocket protocol

### Direct Motor Control Test

Tests direct Tinkerforge motor control (default mode, bypasses ROS):

```bash
# Test elbow motors (default)
python tests/test_low_latency.py --host 172.26.34.149

# Test hand motors
python tests/test_low_latency.py --host 172.26.34.149 --motors hands

# Test only left hand
python tests/test_low_latency.py --host 172.26.34.149 --motors hands-left

# Skip ROS topic check
python tests/test_low_latency.py --host 172.26.34.149 --skip-ros-check
```

Available motor groups: `elbows`, `elbows-left`, `elbows-right`, `hands`, `hands-left`, `hands-right`, `all`.

### Unit Tests

```bash
python -m pytest tests/test_tinkerforge_optimization.py
```

## For Users

If you're looking for **example code** to learn from and build upon, see the `examples/` directory instead:

```
examples/
├── basic_joint_control.py       # Joint control basics
├── camera_ai_imu_example.py     # Camera and sensor usage
├── audio_play_speak_and_record.py  # Audio features
├── trajectory_playback.py       # Trajectory generation & playback
├── scan_and_draw.py             # Full sense-plan-act demo
└── ...
```
