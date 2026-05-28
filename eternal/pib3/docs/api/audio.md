# Audio System

The pib3 audio system provides unified audio playback, recording, text-to-speech (TTS), and device management capabilities that work seamlessly across both the real robot and Webots simulation.

## Overview

The audio system consists of several key components:

- **Audio Playback**: Play raw audio data or WAV files on local speakers, robot speakers, or both
- **Audio Recording**: Record from local microphone or robot's microphone array
- **Text-to-Speech**: Synthesize speech using Piper TTS with German and English voices
- **Device Management**: List and select specific speakers and microphones
- **Output/Input Routing**: Control where audio plays/records using `AudioOutput` and `AudioInput` enums

## Audio Format

All audio in the pib3 system uses a standardized format:

- **Sample rate**: 16000 Hz (16kHz)
- **Channels**: 1 (Mono)
- **Bit depth**: 16-bit signed integers (int16)

## Quick Start

```python
from pib3 import Robot, AudioOutput, AudioInput
import numpy as np

with Robot(host="172.26.34.149") as robot:
    # Text-to-speech (German speech model and on the robot device by default)
    robot.speak("Hallo! Ich bin pib, dein freundlicher Roboter.")

    # Play on local speakers only
    robot.speak("Testing local audio", output=AudioOutput.LOCAL)

    # Play on both robot and laptop
    robot.speak("Playing everywhere!", output=AudioOutput.LOCAL_AND_ROBOT)

    # Play a WAV file (on the robot device)
    robot.play_file("sound.wav", output=AudioOutput.ROBOT)

    # Generate and play a tone
    tone = (np.sin(2 * np.pi * 440 * np.linspace(0, 1, 16000)) * 16000).astype(np.int16)
    robot.play_audio(tone, output=AudioOutput.ROBOT)

    # Record from local microphone - returns numpy int16 array
    audio_data = robot.record_audio(duration=5.0, input_source=AudioInput.LOCAL)

    # Record from robot microphone
    audio_data = robot.record_audio(duration=5.0, input_source=AudioInput.ROBOT)

    # The same audio_data format works for playback!
    robot.play_audio(audio_data)

    # List available devices
    for device in robot.get_audio_output_devices():
        print(f"Speaker: {device}")
    for device in robot.get_audio_input_devices():
        print(f"Microphone: {device}")
```

## AudioOutput Enum

The `AudioOutput` enum controls where audio is played:

```python
from pib3 import AudioOutput

# Available options:
AudioOutput.LOCAL              # Play on local machine (laptop) speakers only
AudioOutput.ROBOT              # Play on robot speakers only
AudioOutput.LOCAL_AND_ROBOT    # Play on both simultaneously
```

### Backend-Specific Behavior

- **Real Robot**: Default is `AudioOutput.ROBOT`
  - `LOCAL`: Plays on your laptop/computer
  - `ROBOT`: Sends audio to robot via `/audio_playback` ROS topic
  - `LOCAL_AND_ROBOT`: Plays on both (best-effort synchronization)

- **Webots Simulation**: Default is `AudioOutput.LOCAL`
  - `LOCAL`: Plays on your laptop/computer
  - `ROBOT`: Automatically redirects to `LOCAL` (no robot speakers in simulation)
  - `LOCAL_AND_ROBOT`: Automatically redirects to `LOCAL` (no duplication)

## AudioInput Enum

The `AudioInput` enum controls where audio is recorded from:

```python
from pib3 import AudioInput

# Available options:
AudioInput.LOCAL    # Record from local machine (laptop) microphone
AudioInput.ROBOT    # Record from robot's microphone array
```

### Backend-Specific Behavior

- **Real Robot**: Default is `AudioInput.ROBOT`
  - `LOCAL`: Records from your laptop/computer microphone
  - `ROBOT`: Records from robot via `/audio_input` ROS topic

- **Webots Simulation**: Default is `AudioInput.LOCAL`
  - `LOCAL`: Records from your laptop/computer microphone
  - `ROBOT`: Automatically redirects to `LOCAL` (no robot microphone in simulation)

## API Reference

### Playback Methods

#### `play_audio()`

Play raw audio data.

```python
robot.play_audio(
    data: Union[bytes, np.ndarray, List[int]],
    sample_rate: int = 16000,
    output: Optional[AudioOutput] = None,
    block: bool = True
) -> bool
```

**Parameters:**

- `data`: Audio data as bytes, numpy array (int16), or list of int16 samples
- `sample_rate`: Sample rate in Hz (default: 16000)
- `output`: Playback destination (LOCAL, ROBOT, LOCAL_AND_ROBOT). If None, uses backend default
- `block`: If True, wait for playback to complete before returning

**Returns:** `True` if playback succeeded

**Example:**

```python
import numpy as np

# Generate a 440Hz tone for 1 second
sample_rate = 16000
t = np.linspace(0, 1.0, sample_rate, dtype=np.float32)
tone = (np.sin(2 * np.pi * 440 * t) * 16000).astype(np.int16)

# Play on robot
robot.play_audio(tone, output=AudioOutput.ROBOT)

# Play on both robot and laptop
robot.play_audio(tone, output=AudioOutput.LOCAL_AND_ROBOT)
```

---

#### `play_file()`

Play audio from a WAV file.

```python
robot.play_file(
    filepath: Union[str, Path],
    output: Optional[AudioOutput] = None,
    block: bool = True
) -> bool
```

**Parameters:**

- `filepath`: Path to WAV file
- `output`: Playback destination. If None, uses backend default
- `block`: If True, wait for playback to complete

**Returns:** `True` if playback succeeded

**Example:**

```python
# Play a sound effect on robot
robot.play_file("sounds/beep.wav", output=AudioOutput.ROBOT)

# Play background music locally while robot works
robot.play_file("music.wav", output=AudioOutput.LOCAL, block=False)
```

---

### Text-to-Speech

#### `speak()`

Synthesize and play text-to-speech.

On `RealRobotBackend`, `speak()` prefers the on-board
`/play_audio_from_speech` ROS service — TTS runs on the robot itself, so
no audio data is streamed over rosbridge. It transparently falls back to
the base-class Piper path for `AudioOutput.LOCAL` or if the robot service
is unavailable. `WebotsBackend` (and any other backend) uses Piper locally.

```python
robot.speak(
    text: str,
    output: Optional[AudioOutput] = None,
    voice: Optional[str] = None,          # Piper voice (local path only)
    block: bool = True,
    use_robot_tts: Optional[bool] = None, # RealRobotBackend only; None = auto
    language: str = "de",                 # RealRobotBackend only; robot TTS language
) -> bool
```

**Parameters:**

- `text`: Text to speak
- `output`: Playback destination. If None, uses backend default
- `voice`: Piper voice model name (default: `"de_DE-thorsten-high"` for German). Ignored when the robot-side TTS path is taken.
- `block`: If True, wait for speech to complete
- `use_robot_tts` *(RealRobotBackend)*: Force robot-side TTS (`True`), force local Piper (`False`), or auto-decide (`None`, default — prefers robot TTS whenever `output` includes `ROBOT`)
- `language` *(RealRobotBackend)*: Language code for the robot's TTS service (e.g. `"en"`, `"de"`)

**Returns:** `True` if speech was played successfully

**Available Voices:**

- `"de_DE-thorsten-high"` - German (Thorsten voice, high quality) - **Default**
- `"en_US-lessac-medium"` - English (Lessac voice, medium quality)
- Other Piper voices from [rhasspy/piper-voices](https://huggingface.co/rhasspy/piper-voices)

Voice models are automatically downloaded on first use and cached in `~/.cache/pib3/piper_models/`.

**Example:**

```python
# German (default)
robot.speak("Hallo! Wie geht es dir?")

# English
robot.speak("Hello! How are you?", voice="en_US-lessac-medium")

# Play on both robot and laptop
robot.speak("Wichtige Nachricht!", output=AudioOutput.LOCAL_AND_ROBOT)
```

---

### Recording Methods

#### `record_audio()`

Record audio for a specified duration.

```python
robot.record_audio(
    duration: float,
    sample_rate: int = 16000,
    input_source: Optional[AudioInput] = None
) -> Optional[np.ndarray]
```

**Parameters:**

- `duration`: Recording duration in seconds
- `sample_rate`: Sample rate in Hz (default: 16000)
- `input_source`: Recording source (LOCAL or ROBOT). If None, uses backend default

**Returns:** Audio data as numpy array of int16 samples, or None if failed

**Example:**

```python
from pib3 import AudioInput
from pib3.backends.audio import save_audio_file

# Record from local microphone - returns numpy int16 array
audio_data = robot.record_audio(duration=5.0, input_source=AudioInput.LOCAL)

# Record from robot microphone
audio_data = robot.record_audio(duration=5.0, input_source=AudioInput.ROBOT)

# Save recording to file
save_audio_file("recording.wav", audio_data)

# Play back immediately - same format works for playback!
robot.play_audio(audio_data)
```

> **Important:** `record_audio()` requires `AudioInput`, not `AudioOutput`. Passing `AudioOutput` will raise a `TypeError` with a helpful message.

---

#### `record_to_file()`

Record audio and save directly to a WAV file.

```python
robot.record_to_file(
    filepath: Union[str, Path],
    duration: float,
    sample_rate: int = 16000,
    input_source: Optional[AudioInput] = None
) -> bool
```

**Parameters:**

- `filepath`: Output file path (WAV format)
- `duration`: Recording duration in seconds
- `sample_rate`: Sample rate in Hz (default: 16000)
- `input_source`: Recording source (LOCAL or ROBOT). If None, uses backend default

**Returns:** `True` if recording was saved successfully

**Example:**

```python
# Record from local microphone to file
robot.record_to_file("my_recording.wav", duration=10.0)

# Record from robot microphone
robot.record_to_file("robot_audio.wav", duration=5.0, input_source=AudioInput.ROBOT)
```

---

### Device Management

> **Note:** Device selection only applies to **LOCAL** audio (laptop speakers/microphones). Robot audio uses fixed ROS topics (`/audio_playback` for output, `/audio_input` for input) with no device selection.

#### `get_audio_output_devices()`

List available **local** audio output (speaker) devices.

```python
robot.get_audio_output_devices() -> List[AudioDevice]
```

**Returns:** List of AudioDevice objects that support output

**Example:**

```python
for device in robot.get_audio_output_devices():
    print(device)  # e.g., "[0] Built-in Speakers (output)"
```

---

#### `get_audio_input_devices()`

List available audio input (microphone) devices.

```python
robot.get_audio_input_devices() -> List[AudioDevice]
```

**Returns:** List of AudioDevice objects that support input

**Example:**

```python
for device in robot.get_audio_input_devices():
    print(device)  # e.g., "[1] USB Microphone (input)"
```

---

#### `set_audio_output_device()`

Set the audio output device for local playback.

```python
robot.set_audio_output_device(
    device: Optional[Union[int, str, AudioDevice]]
) -> None
```

**Parameters:**

- `device`: Device index (int), name substring (str), AudioDevice object, or None for system default

**Example:**

```python
# List available devices
devices = robot.get_audio_output_devices()
for d in devices:
    print(d)

# Select by index
robot.set_audio_output_device(0)

# Select by name (substring match)
robot.set_audio_output_device("Speakers")

# Select by AudioDevice object
robot.set_audio_output_device(devices[0])

# Reset to system default
robot.set_audio_output_device(None)
```

---

#### `set_audio_input_device()`

Set the audio input device for local recording.

```python
robot.set_audio_input_device(
    device: Optional[Union[int, str, AudioDevice]]
) -> None
```

**Parameters:**

- `device`: Device index (int), name substring (str), AudioDevice object, or None for system default

**Example:**

```python
# List available devices
devices = robot.get_audio_input_devices()
for d in devices:
    print(d)

# Select by name
robot.set_audio_input_device("USB Microphone")

# Reset to system default
robot.set_audio_input_device(None)
```

---

#### `get_current_audio_output_device()` / `get_current_audio_input_device()`

Get the currently selected device.

```python
robot.get_current_audio_output_device() -> Optional[AudioDevice]
robot.get_current_audio_input_device() -> Optional[AudioDevice]
```

**Returns:** Currently selected AudioDevice, or system default if None was set

---

### Legacy Recording API

#### `subscribe_audio_stream()`

Subscribe to audio stream from the robot's microphone array.

> **Note:** For simple recording, prefer `record_audio()`. Use this for continuous streaming or custom processing.

```python
robot.subscribe_audio_stream(
    callback: Callable[[List[int]], None]
) -> roslibpy.Topic
```

**Parameters:**

- `callback`: Function called with list of int16 audio samples for each chunk (~1024 samples)

**Returns:** Topic object (call `.unsubscribe()` to stop streaming)

**Example:**

```python
from pib3 import AudioStreamReceiver

# Using AudioStreamReceiver helper
receiver = AudioStreamReceiver()
sub = robot.subscribe_audio_stream(receiver.on_audio)

# Record for 10 seconds
import time
time.sleep(10)
sub.unsubscribe()

# Save to WAV file
receiver.save_wav("recording.wav")
```

---

### Helper Classes

#### `AudioDevice`

Represents an audio device (speaker or microphone).

```python
from pib3 import AudioDevice

# Properties:
device.index       # int: Device index for sounddevice
device.name        # str: Human-readable device name
device.channels    # int: Number of channels supported
device.sample_rate # float: Default sample rate
device.is_input    # bool: True if this is an input (microphone) device
device.is_output   # bool: True if this is an output (speaker) device
```

---

### Standalone Device Functions

These functions work without a robot connection and can be used for device discovery.

#### `list_audio_devices()`

List available audio devices with optional filtering.

```python
from pib3 import list_audio_devices

# List all devices
all_devices = list_audio_devices()

# List only input devices (microphones)
inputs = list_audio_devices(input=True, output=False)

# List only output devices (speakers)
outputs = list_audio_devices(input=False, output=True)
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `input` | `bool` | `True` | Include input (microphone) devices |
| `output` | `bool` | `True` | Include output (speaker) devices |

**Returns:** `List[AudioDevice]`

!!! note "Deprecated Functions"
    The following functions are deprecated and will be removed in a future version:
    
    - `list_audio_input_devices()` → Use `list_audio_devices(input=True, output=False)`
    - `list_audio_output_devices()` → Use `list_audio_devices(input=False, output=True)`

---

#### `AudioStreamReceiver`

Helper class for buffering and saving audio from the robot's microphone.

```python
from pib3 import AudioStreamReceiver

receiver = AudioStreamReceiver(
    sample_rate: int = 16000,
    max_buffer_seconds: float = 300.0
)
```

**Methods:**

- `on_audio(samples)` - Callback for incoming audio (pass to `subscribe_audio_stream()`)
- `get_audio()` - Get buffered audio as numpy array (int16)
- `get_audio_float()` - Get buffered audio as normalized float32 in range [-1, 1]
- `clear()` - Clear the audio buffer
- `save_wav(filepath)` - Save buffered audio to WAV file

**Properties:**

- `duration` - Duration of buffered audio in seconds
- `sample_count` - Number of samples in buffer

---

## ROS Topics

The audio system uses the following ROS topics for communication with the robot:

### `/audio_playback`

**Type:** `std_msgs/msg/Int16MultiArray`
**Direction:** Client → Robot
**Purpose:** Send audio data to robot for playback through speakers

### `/audio_input`

**Type:** `std_msgs/msg/Int16MultiArray`
**Direction:** Robot → Client
**Purpose:** Stream audio from robot's microphone array to client

### `/audio_stream`

**Type:** `std_msgs/msg/Int16MultiArray`
**Direction:** Robot → Client
**Purpose:** Legacy topic for audio streaming (use `/audio_input` for new code)

---

## Installation Requirements

Audio functionality is included in the core pib3 package:

```bash
pip install pib3
```

### Platform-Specific Requirements

#### Linux

PortAudio is required for audio playback and recording:

```bash
sudo apt-get install libportaudio2 portaudio19-dev
pip install pib3
```

#### macOS

No additional system dependencies required:

```bash
pip install pib3
```

#### Windows

No additional system dependencies required:

```bash
pip install pib3
```

---

## Advanced Usage

### Custom Audio Processing

```python
import numpy as np
from pib3 import Robot, AudioOutput

with Robot(host="172.26.34.149") as robot:
    # Generate custom waveform
    sample_rate = 16000
    duration = 2.0
    t = np.linspace(0, duration, int(sample_rate * duration))

    # Frequency sweep from 200Hz to 800Hz
    freq = np.linspace(200, 800, len(t))
    audio = (np.sin(2 * np.pi * freq * t) * 16000).astype(np.int16)

    # Apply fade in/out
    fade_samples = int(0.1 * sample_rate)
    audio[:fade_samples] *= np.linspace(0, 1, fade_samples)
    audio[-fade_samples:] *= np.linspace(1, 0, fade_samples)

    robot.play_audio(audio, output=AudioOutput.ROBOT)
```

### Recording and Playback Round-Trip

```python
from pib3 import Robot, AudioOutput, AudioInput

with Robot(host="172.26.34.149") as robot:
    # Record from robot microphone
    robot.speak("Ich höre jetzt zu. Bitte sprechen Sie.")
    audio_data = robot.record_audio(duration=5.0, input_source=AudioInput.ROBOT)

    # Play back the recording - same format works for playback!
    robot.speak("Das haben Sie gesagt:")
    robot.play_audio(audio_data, output=AudioOutput.ROBOT)
```

### Device Selection Example

```python
from pib3 import Robot

with Robot(host="172.26.34.149") as robot:
    # List and select output device
    print("Available speakers:")
    for device in robot.get_audio_output_devices():
        print(f"  {device}")

    robot.set_audio_output_device("Headphones")  # Select by name

    # List and select input device
    print("\nAvailable microphones:")
    for device in robot.get_audio_input_devices():
        print(f"  {device}")

    robot.set_audio_input_device(0)  # Select by index

    # Now record and play with selected devices
    audio_data = robot.record_audio(duration=3.0, input_source=AudioInput.LOCAL)
    robot.play_audio(audio_data, output=AudioOutput.LOCAL)  # Same format!
```

---

## Complete Example

Here's a complete example demonstrating the main audio features:

```python
from pib3 import Robot, AudioOutput, AudioInput
import numpy as np

with Robot(host="172.26.34.149") as robot:
    # 1. Text-to-speech (German by default)
    robot.speak("Hallo, ich bin pib!")
    
    # 2. Play on different outputs
    robot.speak("On robot speakers", output=AudioOutput.ROBOT)
    robot.speak("On local speakers", output=AudioOutput.LOCAL)
    robot.speak("On both!", output=AudioOutput.LOCAL_AND_ROBOT)
    
    # 3. Play a WAV file
    robot.play_file("notification.wav", output=AudioOutput.ROBOT)
    
    # 4. Generate and play a tone (440Hz for 1 second)
    sample_rate = 16000
    duration = 1.0
    t = np.linspace(0, duration, int(sample_rate * duration))
    tone = (np.sin(2 * np.pi * 440 * t) * 16000).astype(np.int16)
    robot.play_audio(tone, output=AudioOutput.LOCAL)
    
    # 5. Record from microphone
    print("Recording for 3 seconds...")
    audio_data = robot.record_audio(duration=3.0, input_source=AudioInput.LOCAL)
    print(f"Recorded {len(audio_data)} samples")
    
    # 6. Play back recording
    robot.play_audio(audio_data, output=AudioOutput.LOCAL)
    
    # 7. List available devices
    print("Output devices:")
    for device in robot.get_audio_output_devices():
        print(f"  {device}")
    
    print("Input devices:")
    for device in robot.get_audio_input_devices():
        print(f"  {device}")
```

---

## Troubleshooting

### No audio playback or recording

**Problem:** `sounddevice not available` warning

**Solution:**
```bash
# Linux
sudo apt-get install libportaudio2 portaudio19-dev
pip install --force-reinstall pib3

# macOS/Windows
pip install --force-reinstall pib3
```

**If you see**: "OSError: PortAudio library not found" during `import pib3` or when running audio code

- This means the Python `sounddevice` package is installed but the native PortAudio
    library is missing or incompatible on the system.
- Fix on Debian/Ubuntu:

```bash
sudo apt-get install libportaudio2 portaudio19-dev
pip install --upgrade --force-reinstall sounddevice
```

After installing PortAudio, re-installing `pib3` in the virtualenv may be necessary.

---

### TTS not working

**Problem:** `piper-tts is required for text-to-speech`

**Solution:**
```bash
pip install piper-tts
```

Voice models are downloaded automatically on first use (~20-50MB per voice).

---

### Audio not playing on robot

**Problem:** Audio plays locally but not on robot

**Checklist:**
1. Verify robot connection: `robot.is_connected` should be `True`
2. Check output setting: Use `output=AudioOutput.ROBOT`
3. Verify rosbridge is running on robot
4. Check robot's audio system is functional

---

### Recording returns empty or None

**Problem:** `record_audio()` returns empty array or None

**Checklist:**
1. Check if input device is available: `robot.get_audio_input_devices()`
2. Verify microphone permissions (especially on macOS)
3. Try selecting a specific device: `robot.set_audio_input_device(0)`
4. For robot recording, ensure `/audio_input` topic is publishing

---

### Audio quality issues

**Problem:** Distorted or choppy audio

**Solutions:**
- Ensure audio is 16kHz, mono, int16 format
- Check network latency (for robot playback/recording)
- Verify sample rate matches (16000 Hz)

---

## See Also

- [Backends API](backends/base.md) - Base backend interface
- [Robot Backend](backends/robot.md) - Real robot backend
- [Webots Backend](backends/webots.md) - Simulation backend
