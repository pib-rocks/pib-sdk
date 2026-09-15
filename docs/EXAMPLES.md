# Runnable examples

All scripts are self-contained and run from a checkout after
`pip install -e .`; installed-package users can copy them anywhere. Use
`--host` to override `pib.local`. Read a script before moving hardware.

| Script | Environment | Purpose |
|---|---|---|
| [`kinematics.py`](../examples/kinematics.py) | **Works offline** | Solve IK, print motor order/limits, and verify with FK |
| [`first_steps.py`](../examples/first_steps.py) | **Needs a live robot** | Connect, read telemetry, configure one motor, make a small move, and return |
| [`poses_timed.py`](../examples/poses_timed.py) | **Needs a live robot** | Save current commanded arm targets and smoothly replay named poses |
| [`vision_depth.py`](../examples/vision_depth.py) | **Needs a live robot** | Save JPEG/depth files and inspect valid depth in millimetres |
| [`drawing_image.py`](../examples/drawing_image.py) | **Needs a live robot** | Trace line art, solve a drawing trajectory, and play it |
| [`voice_assistant.py`](../examples/voice_assistant.py) | **Needs a live robot** | Create a chat, send a message, and poll for a reply |
| [`program_and_button.py`](../examples/program_and_button.py) | **Needs a live robot** | Run a saved Blockly program and optionally bind an RGB button |
| [`display_and_relay.py`](../examples/display_and_relay.py) | **Needs a live robot** | Show built-in/custom imagery and optionally pulse the relay |

Examples intentionally have no test-only fakes. A script marked live opens
real rosbridge and/or pib-backend connections. Checked command failures raise
an exception; APIs that intentionally return no delivery result are called
out in the reference.

## Run

```bash
python examples/kinematics.py --side right --xyz -400 100 900
python examples/first_steps.py --host pib.local --motor elbow_right --angle 10
python examples/poses_timed.py --host pib.local --save-name sdk_start sdk_start:1 rest:2
python examples/vision_depth.py --host pib.local --output-dir captures
python examples/drawing_image.py line-art.png --host pib.local
python examples/voice_assistant.py --host pib.local --message "Introduce yourself."
python examples/program_and_button.py --host pib.local --program-number PROGRAM_ID --button 5
python examples/display_and_relay.py --host pib.local --image logo.png
```

The pose command assumes a saved Cerebra pose named `rest` already exists.

Compile every example without connecting to a robot:

```bash
python tools/check_examples.py
```
