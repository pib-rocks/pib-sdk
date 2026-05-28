"""
Audio subsystem for PIB robot.

This module provides unified audio playback, recording, and TTS capabilities that work
across both the real robot and Webots simulation.

Key Components:
- AudioOutput: Enum for specifying playback destination (LOCAL, ROBOT, LOCAL_AND_ROBOT)
- AudioInput: Enum for specifying recording source (LOCAL, ROBOT)
- AudioDevice: Dataclass representing an audio device (speaker or microphone)
- PiperTTS: Text-to-speech synthesis using Piper with Thorsten German voice
- LocalAudioPlayer: Cross-platform local audio playback using sounddevice
- LocalAudioRecorder: Cross-platform local audio recording using sounddevice
- RobotAudioPlayer: Send audio to robot via /audio_playback ROS topic
- RobotAudioRecorder: Receive audio from robot via /audio_input ROS topic

Audio Format (standard throughout):
- Sample rate: 16000 Hz (16kHz)
- Channels: 1 (Mono)
- Bit depth: 16-bit signed integers (int16)

Platform-specific requirements:
- Linux: sudo apt-get install libportaudio2 portaudio19-dev (before pip install pib3)
- macOS: No additional requirements
- Windows: No additional requirements

Usage:
    >>> with Robot(host="...") as robot:
    ...     # Play audio on robot speaker
    ...     robot.play_audio(audio_data, output=AudioOutput.ROBOT)
    ...
    ...     # Play on both local and robot
    ...     robot.play_file("sound.wav", output=AudioOutput.LOCAL_AND_ROBOT)
    ...
    ...     # Text-to-speech (German by default)
    ...     robot.speak("Hallo, ich bin pib!")
    ...
    ...     # Record from local microphone
    ...     audio = robot.record_audio(duration=5.0, input=AudioInput.LOCAL)
    ...
    ...     # Record from robot microphone
    ...     audio = robot.record_audio(duration=5.0, input=AudioInput.ROBOT)
    ...
    ...     # List available devices
    ...     speakers = list_audio_output_devices()
    ...     microphones = list_audio_input_devices()
"""

import io
import logging
import sys
import threading
import time
import wave
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Callable, List, Optional, Tuple, Union

import numpy as np

logger = logging.getLogger(__name__)

# Platform-specific installation help for sounddevice
if sys.platform.startswith('linux'):
    _SOUNDDEVICE_HELP = (
        "sounddevice is required for audio playback and recording.\n"
        "On Linux, first install PortAudio:\n"
        "    sudo apt-get install libportaudio2 portaudio19-dev\n"
        "Then install sounddevice:\n"
        "    pip install sounddevice"
    )
elif sys.platform == 'darwin':  # macOS
    _SOUNDDEVICE_HELP = (
        "sounddevice is required for audio playback and recording.\n"
        "Install with:\n"
        "    pip install sounddevice"
    )
elif sys.platform == 'win32':  # Windows
    _SOUNDDEVICE_HELP = (
        "sounddevice is required for audio playback and recording.\n"
        "Install with:\n"
        "    pip install sounddevice"
    )
else:
    _SOUNDDEVICE_HELP = (
        "sounddevice is required for audio playback and recording.\n"
        "Install with: pip install sounddevice"
    )

# Audio playback/recording imports
# Importing `sounddevice` can fail in two ways:
#  - ImportError when the Python package is not installed
#  - OSError (or other Exception) when the PortAudio native library is
#    missing or broken (e.g. "PortAudio library not found").
#
# Handle both cases gracefully so that importing `pib3` does not crash on
# machines where sound support is not available; callers should check
# `HAS_SOUNDDEVICE` before attempting hardware playback/recording.
try:
    import sounddevice as sd
    HAS_SOUNDDEVICE = True
except (ImportError, OSError) as e:
    sd = None
    HAS_SOUNDDEVICE = False
    # Log the underlying exception and provide the platform-specific hint
    logger.warning(f"sounddevice not available: {e}\n{_SOUNDDEVICE_HELP}")

try:
    import roslibpy
    HAS_ROSLIBPY = True
except ImportError:
    roslibpy = None
    HAS_ROSLIBPY = False


# ==================== CONSTANTS ====================

DEFAULT_SAMPLE_RATE = 16000
DEFAULT_CHANNELS = 1
DEFAULT_SAMPLE_WIDTH = 2  # 16-bit = 2 bytes
DEFAULT_CHUNK_SIZE = 1024

# Piper TTS settings
PIPER_MODEL_DIR = Path.home() / ".cache" / "pib3" / "piper_models"
DEFAULT_PIPER_VOICE = "de_DE-thorsten-high"  # German Thorsten voice


# ==================== AUDIO ENUMS ====================


class AudioOutput(Enum):
    """
    Destination for audio playback.

    Attributes:
        LOCAL: Play on local machine (laptop) speakers only.
        ROBOT: Play on robot speakers only (via /audio_playback topic).
        LOCAL_AND_ROBOT: Play on both simultaneously (best-effort sync).

    Note:
        In Webots simulation, ROBOT and LOCAL_AND_ROBOT both resolve to
        LOCAL-only playback (no duplication).
    """
    LOCAL = "local"
    ROBOT = "robot"
    LOCAL_AND_ROBOT = "local_and_robot"


class AudioInput(Enum):
    """
    Source for audio recording.

    Attributes:
        LOCAL: Record from local machine (laptop) microphone.
        ROBOT: Record from robot's microphone (via /audio_input topic).

    Note:
        In Webots simulation, ROBOT resolves to LOCAL recording.
    """
    LOCAL = "local"
    ROBOT = "robot"


# ==================== AUDIO DEVICE ====================


@dataclass
class AudioDevice:
    """
    Represents an audio device (speaker or microphone).

    Attributes:
        index: Device index for sounddevice.
        name: Human-readable device name.
        channels: Number of channels supported.
        sample_rate: Default sample rate.
        is_input: True if this is an input (microphone) device.
        is_output: True if this is an output (speaker) device.
    """
    index: int
    name: str
    channels: int
    sample_rate: float
    is_input: bool
    is_output: bool

    def __str__(self) -> str:
        device_type = []
        if self.is_input:
            device_type.append("input")
        if self.is_output:
            device_type.append("output")
        return f"[{self.index}] {self.name} ({', '.join(device_type)})"


def list_audio_devices(
    input: bool = True,
    output: bool = True,
) -> List[AudioDevice]:
    """
    List available audio devices.

    Args:
        input: Include input (microphone) devices.
        output: Include output (speaker) devices.

    Returns:
        List of AudioDevice objects matching the criteria.

    Raises:
        ImportError: If sounddevice is not available.

    Example:
        >>> # List all devices
        >>> devices = list_audio_devices()
        >>>
        >>> # List only microphones
        >>> mics = list_audio_devices(input=True, output=False)
        >>>
        >>> # List only speakers
        >>> speakers = list_audio_devices(input=False, output=True)
    """
    if not HAS_SOUNDDEVICE:
        raise ImportError(_SOUNDDEVICE_HELP)

    devices = []
    for i, dev in enumerate(sd.query_devices()):
        is_input = dev['max_input_channels'] > 0
        is_output = dev['max_output_channels'] > 0

        # Filter based on requested device types
        if not input and is_input and not is_output:
            continue
        if not output and is_output and not is_input:
            continue
        if not input and not output:
            continue

        # Include device if it matches requested type
        include = False
        if input and is_input:
            include = True
        if output and is_output:
            include = True

        if include:
            devices.append(AudioDevice(
                index=i,
                name=dev['name'],
                channels=max(dev['max_input_channels'], dev['max_output_channels']),
                sample_rate=dev['default_samplerate'],
                is_input=is_input,
                is_output=is_output,
            ))
    return devices


def list_audio_input_devices() -> List[AudioDevice]:
    """
    List available audio input (microphone) devices.

    .. deprecated:: Use list_audio_devices(input=True, output=False) instead.

    Returns:
        List of AudioDevice objects that support input.
    """
    return list_audio_devices(input=True, output=False)


def list_audio_output_devices() -> List[AudioDevice]:
    """
    List available audio output (speaker) devices.

    .. deprecated:: Use list_audio_devices(input=False, output=True) instead.

    Returns:
        List of AudioDevice objects that support output.
    """
    return list_audio_devices(input=False, output=True)


def get_default_audio_input_device() -> Optional[AudioDevice]:
    """
    Get the default audio input (microphone) device.

    Returns:
        Default input AudioDevice or None if not available.
    """
    if not HAS_SOUNDDEVICE:
        return None
    try:
        idx = sd.default.device[0]
        if idx is None:
            return None
        devices = list_audio_devices()
        for d in devices:
            if d.index == idx and d.is_input:
                return d
        # Fallback: return first input device
        for d in devices:
            if d.is_input:
                return d
        return None
    except Exception:
        return None


def get_default_audio_output_device() -> Optional[AudioDevice]:
    """
    Get the default audio output (speaker) device.

    Returns:
        Default output AudioDevice or None if not available.
    """
    if not HAS_SOUNDDEVICE:
        return None
    try:
        idx = sd.default.device[1]
        if idx is None:
            return None
        devices = list_audio_devices()
        for d in devices:
            if d.index == idx and d.is_output:
                return d
        # Fallback: return first output device
        for d in devices:
            if d.is_output:
                return d
        return None
    except Exception:
        return None


# ==================== LOCAL AUDIO PLAYER ====================


class LocalAudioPlayer:
    """
    Cross-platform local audio playback using sounddevice.

    Plays audio through the local machine's speakers.
    """

    def __init__(self, device: Optional[Union[int, str, AudioDevice]] = None):
        """
        Initialize local audio player.

        Args:
            device: Output device - can be index (int), name (str), or AudioDevice.
                    If None, uses system default.
        """
        if not HAS_SOUNDDEVICE:
            raise ImportError(_SOUNDDEVICE_HELP)

        self._device = self._resolve_device(device)
        self._stream = None
        self._lock = threading.Lock()
        self._stop_event = threading.Event()

    def _resolve_device(self, device: Optional[Union[int, str, AudioDevice]]) -> Optional[int]:
        """Resolve device parameter to device index."""
        if device is None:
            return None
        if isinstance(device, AudioDevice):
            return device.index
        if isinstance(device, int):
            return device
        if isinstance(device, str):
            # Search by name
            for d in list_audio_output_devices():
                if device.lower() in d.name.lower():
                    return d.index
            raise ValueError(f"Output device not found: {device}")
        raise ValueError(f"Invalid device type: {type(device)}")

    def play(
        self,
        data: Union[bytes, np.ndarray, List[int]],
        sample_rate: int = DEFAULT_SAMPLE_RATE,
        block: bool = True,
    ) -> bool:
        """
        Play audio data through local speakers.

        Args:
            data: Audio data as bytes, numpy array (int16), or list of int16.
            sample_rate: Sample rate in Hz (default: 16000).
            block: If True, wait for playback to complete.

        Returns:
            True if playback started/completed successfully.
        """
        # Convert to numpy int16 array
        if isinstance(data, bytes):
            audio = np.frombuffer(data, dtype=np.int16)
        elif isinstance(data, list):
            audio = np.array(data, dtype=np.int16)
        elif isinstance(data, np.ndarray):
            audio = data.astype(np.int16)
        else:
            raise ValueError(f"Unsupported audio data type: {type(data)}")

        # Convert to float32 for sounddevice (range -1 to 1)
        audio_float = audio.astype(np.float32) / 32768.0

        with self._lock:
            self.stop()
            self._stop_event.clear()

            try:
                if block:
                    sd.play(audio_float, samplerate=sample_rate, device=self._device)
                    sd.wait()
                else:
                    sd.play(audio_float, samplerate=sample_rate, device=self._device)
                return True
            except Exception as e:
                logger.warning(f"Local audio playback failed: {e}")
                return False

    def stop(self) -> None:
        """Stop current playback."""
        self._stop_event.set()
        try:
            sd.stop()
        except Exception:
            pass

    def is_playing(self) -> bool:
        """Check if audio is currently playing."""
        try:
            return sd.get_stream() is not None and sd.get_stream().active
        except Exception:
            return False

    @property
    def device(self) -> Optional[int]:
        """Get the current device index."""
        return self._device

    def set_device(self, device: Optional[Union[int, str, AudioDevice]]) -> None:
        """
        Set the output device.

        Args:
            device: Output device - can be index (int), name (str), or AudioDevice.
        """
        self._device = self._resolve_device(device)


# ==================== LOCAL AUDIO RECORDER ====================


class LocalAudioRecorder:
    """
    Cross-platform local audio recording using sounddevice.

    Records audio from the local machine's microphone.
    """

    def __init__(self, device: Optional[Union[int, str, AudioDevice]] = None):
        """
        Initialize local audio recorder.

        Args:
            device: Input device - can be index (int), name (str), or AudioDevice.
                    If None, uses system default.
        """
        if not HAS_SOUNDDEVICE:
            raise ImportError(_SOUNDDEVICE_HELP)

        self._device = self._resolve_device(device)
        self._recording = False
        self._lock = threading.Lock()

    def _resolve_device(self, device: Optional[Union[int, str, AudioDevice]]) -> Optional[int]:
        """Resolve device parameter to device index."""
        if device is None:
            return None
        if isinstance(device, AudioDevice):
            return device.index
        if isinstance(device, int):
            return device
        if isinstance(device, str):
            # Search by name
            for d in list_audio_input_devices():
                if device.lower() in d.name.lower():
                    return d.index
            raise ValueError(f"Input device not found: {device}")
        raise ValueError(f"Invalid device type: {type(device)}")

    def record(
        self,
        duration: float,
        sample_rate: int = DEFAULT_SAMPLE_RATE,
    ) -> np.ndarray:
        """
        Record audio for a specified duration.

        Args:
            duration: Recording duration in seconds.
            sample_rate: Sample rate in Hz (default: 16000).

        Returns:
            Audio data as numpy array of int16 samples.
        """
        with self._lock:
            self._recording = True
            try:
                num_samples = int(duration * sample_rate)
                audio_float = sd.rec(
                    num_samples,
                    samplerate=sample_rate,
                    channels=DEFAULT_CHANNELS,
                    dtype=np.float32,
                    device=self._device,
                )
                sd.wait()

                # Convert to int16
                audio = (audio_float.flatten() * 32767).astype(np.int16)
                return audio
            finally:
                self._recording = False

    def record_until_stopped(
        self,
        sample_rate: int = DEFAULT_SAMPLE_RATE,
        max_duration: float = 300.0,
    ) -> np.ndarray:
        """
        Start recording and continue until stop() is called.

        Args:
            sample_rate: Sample rate in Hz (default: 16000).
            max_duration: Maximum recording duration in seconds (default: 5 minutes).

        Returns:
            Audio data as numpy array of int16 samples.

        Note:
            Call stop() from another thread to end recording early.
        """
        with self._lock:
            self._recording = True
            try:
                num_samples = int(max_duration * sample_rate)
                audio_float = sd.rec(
                    num_samples,
                    samplerate=sample_rate,
                    channels=DEFAULT_CHANNELS,
                    dtype=np.float32,
                    device=self._device,
                )

                # Wait for recording to complete or stop() to be called
                while self._recording and sd.get_stream() and sd.get_stream().active:
                    time.sleep(0.1)

                sd.stop()

                # Trim to actual recorded length
                # (approximation based on how much was filled)
                audio = (audio_float.flatten() * 32767).astype(np.int16)
                # Remove trailing zeros
                nonzero = np.nonzero(audio)[0]
                if len(nonzero) > 0:
                    audio = audio[:nonzero[-1] + 1]
                return audio
            finally:
                self._recording = False

    def stop(self) -> None:
        """Stop current recording."""
        self._recording = False
        try:
            sd.stop()
        except Exception:
            pass

    def is_recording(self) -> bool:
        """Check if currently recording."""
        return self._recording

    @property
    def device(self) -> Optional[int]:
        """Get the current device index."""
        return self._device

    def set_device(self, device: Optional[Union[int, str, AudioDevice]]) -> None:
        """
        Set the input device.

        Args:
            device: Input device - can be index (int), name (str), or AudioDevice.
        """
        self._device = self._resolve_device(device)


# ==================== ROBOT AUDIO PLAYER ====================


class RobotAudioPlayer:
    """
    Send audio to robot for playback via /audio_playback ROS topic.

    The robot's audio_player node subscribes to this topic and plays
    received audio through the robot's speakers.
    """

    TOPIC_NAME = "/audio_playback"
    TOPIC_TYPE = "std_msgs/msg/Int16MultiArray"

    def __init__(self, client: "roslibpy.Ros"):
        """
        Initialize robot audio player.

        Args:
            client: Connected roslibpy.Ros instance.
        """
        if not HAS_ROSLIBPY:
            raise ImportError("roslibpy is required for robot audio.")

        self._client = client
        self._topic = roslibpy.Topic(
            client,
            self.TOPIC_NAME,
            self.TOPIC_TYPE,
        )

    def play(
        self,
        data: Union[bytes, np.ndarray, List[int]],
        sample_rate: int = DEFAULT_SAMPLE_RATE,
        block: bool = True,
    ) -> bool:
        """
        Send audio to robot for playback.

        Args:
            data: Audio data as bytes, numpy array (int16), or list of int16.
            sample_rate: Sample rate in Hz (default: 16000).
            block: If True, wait estimated playback duration.

        Returns:
            True if audio was sent successfully.
        """
        if not self._client.is_connected:
            logger.warning("Cannot play on robot: not connected")
            return False

        # Convert to list of int16
        if isinstance(data, bytes):
            int_data = np.frombuffer(data, dtype=np.int16).tolist()
        elif isinstance(data, np.ndarray):
            int_data = data.astype(np.int16).tolist()
        elif isinstance(data, list):
            int_data = [int(x) for x in data]
        else:
            raise ValueError(f"Unsupported audio data type: {type(data)}")

        # Construct Int16MultiArray message
        msg = roslibpy.Message({
            "layout": {"dim": [], "data_offset": 0},
            "data": int_data,
        })

        try:
            self._topic.publish(msg)

            if block:
                # Estimate playback duration and wait
                duration = len(int_data) / sample_rate
                time.sleep(duration)

            return True
        except Exception as e:
            logger.warning(f"Failed to send audio to robot: {e}")
            return False

    def stop(self) -> None:
        """Stop playback (sends empty message to clear queue)."""
        if self._client.is_connected:
            try:
                msg = roslibpy.Message({
                    "layout": {"dim": [], "data_offset": 0},
                    "data": [],
                })
                self._topic.publish(msg)
            except Exception:
                pass


# ==================== ROBOT AUDIO RECORDER ====================


class RobotAudioRecorder:
    """
    Record audio from robot's microphone via /audio_input ROS topic.

    Subscribes to the robot's audio stream and buffers incoming audio.
    """

    TOPIC_NAME = "/audio_input"
    TOPIC_TYPE = "std_msgs/msg/Int16MultiArray"

    def __init__(self, client: "roslibpy.Ros"):
        """
        Initialize robot audio recorder.

        Args:
            client: Connected roslibpy.Ros instance.
        """
        if not HAS_ROSLIBPY:
            raise ImportError("roslibpy is required for robot audio.")

        self._client = client
        self._topic = roslibpy.Topic(
            client,
            self.TOPIC_NAME,
            self.TOPIC_TYPE,
        )
        self._buffer: List[int] = []
        self._lock = threading.Lock()
        self._recording = False
        self._subscription_active = False

    def _on_message(self, message: dict) -> None:
        """Handle incoming audio message from robot."""
        if not self._recording:
            return

        data = message.get("data", [])
        if data:
            with self._lock:
                self._buffer.extend(data)

    def record(
        self,
        duration: float,
        sample_rate: int = DEFAULT_SAMPLE_RATE,
    ) -> np.ndarray:
        """
        Record audio from robot for a specified duration.

        Args:
            duration: Recording duration in seconds.
            sample_rate: Sample rate in Hz (default: 16000).

        Returns:
            Audio data as numpy array of int16 samples.
        """
        if not self._client.is_connected:
            raise RuntimeError("Cannot record from robot: not connected")

        with self._lock:
            self._buffer.clear()

        self._recording = True

        # Start subscription if not active
        if not self._subscription_active:
            self._topic.subscribe(self._on_message)
            self._subscription_active = True

        # Wait for specified duration
        time.sleep(duration)

        self._recording = False

        with self._lock:
            audio = np.array(self._buffer, dtype=np.int16)
            self._buffer.clear()

        return audio

    def start_recording(self) -> None:
        """
        Start continuous recording from robot.

        Use get_audio() to retrieve buffered audio and stop() to end.
        """
        if not self._client.is_connected:
            raise RuntimeError("Cannot record from robot: not connected")

        with self._lock:
            self._buffer.clear()

        self._recording = True

        if not self._subscription_active:
            self._topic.subscribe(self._on_message)
            self._subscription_active = True

    def stop(self) -> np.ndarray:
        """
        Stop recording and return captured audio.

        Returns:
            Audio data as numpy array of int16 samples.
        """
        self._recording = False

        with self._lock:
            audio = np.array(self._buffer, dtype=np.int16)
            self._buffer.clear()

        return audio

    def get_audio(self) -> np.ndarray:
        """
        Get currently buffered audio without stopping.

        Returns:
            Audio data as numpy array of int16 samples.
        """
        with self._lock:
            return np.array(self._buffer, dtype=np.int16)

    def clear_buffer(self) -> None:
        """Clear the audio buffer."""
        with self._lock:
            self._buffer.clear()

    def is_recording(self) -> bool:
        """Check if currently recording."""
        return self._recording

    def close(self) -> None:
        """Unsubscribe and clean up."""
        self._recording = False
        if self._subscription_active:
            try:
                self._topic.unsubscribe()
            except Exception:
                pass
            self._subscription_active = False


# ==================== PIPER TTS ====================


class PiperTTS:
    """
    Text-to-speech synthesis using Piper.

    Uses the Thorsten German voice by default. Voice models are automatically
    downloaded on first use.

    Piper is a fast, local neural TTS system that works well on
    resource-constrained devices like Raspberry Pi.
    """

    def __init__(self, voice: str = DEFAULT_PIPER_VOICE):
        """
        Initialize Piper TTS.

        Args:
            voice: Piper voice model name (default: German Thorsten).
        """
        self._voice = voice
        self._piper = None
        self._model_path = None
        self._initialized = False

    def _ensure_initialized(self) -> bool:
        """Ensure Piper is initialized and model is downloaded."""
        if self._initialized:
            return True

        try:
            from piper import PiperVoice
        except ImportError:
            raise ImportError(
                "piper-tts is required for text-to-speech. "
                "Install with: pip install piper-tts"
            )

        # Ensure model directory exists
        PIPER_MODEL_DIR.mkdir(parents=True, exist_ok=True)

        # Download model if needed
        model_path = self._download_model_if_needed()
        if model_path is None:
            return False

        # Load the model
        try:
            self._piper = PiperVoice.load(str(model_path))
            self._model_path = model_path
            self._initialized = True
            logger.info(f"Piper TTS initialized with voice: {self._voice}")
            return True
        except Exception as e:
            logger.error(f"Failed to load Piper model: {e}")
            return False

    def _download_model_if_needed(self) -> Optional[Path]:
        """Download Piper voice model if not already cached."""
        model_file = PIPER_MODEL_DIR / f"{self._voice}.onnx"
        config_file = PIPER_MODEL_DIR / f"{self._voice}.onnx.json"

        if model_file.exists() and config_file.exists():
            return model_file

        logger.info(f"Downloading Piper voice model: {self._voice}")

        try:
            import urllib.request

            # Piper model URLs (from official releases)
            base_url = "https://huggingface.co/rhasspy/piper-voices/resolve/main"

            # Map voice name to path
            # URL structure: {lang_prefix}/{locale}/{speaker}/{quality}/
            # e.g., de/de_DE/thorsten/high/ for voice de_DE-thorsten-high
            voice_parts = self._voice.split("-")
            if len(voice_parts) >= 3:
                locale = voice_parts[0]  # e.g., "de_DE"
                name = voice_parts[1]  # e.g., "thorsten"
                quality = voice_parts[2]  # e.g., "high"
                # Extract language prefix from locale (e.g., "de" from "de_DE")
                lang_prefix = locale.split("_")[0]
                voice_path = f"{lang_prefix}/{locale}/{name}/{quality}"
            else:
                voice_path = self._voice

            model_url = f"{base_url}/{voice_path}/{self._voice}.onnx"
            config_url = f"{base_url}/{voice_path}/{self._voice}.onnx.json"

            # Download model
            logger.info(f"Downloading: {model_url}")
            urllib.request.urlretrieve(model_url, model_file)

            # Download config
            logger.info(f"Downloading: {config_url}")
            urllib.request.urlretrieve(config_url, config_file)

            logger.info(f"Voice model downloaded to: {PIPER_MODEL_DIR}")
            return model_file

        except Exception as e:
            logger.error(f"Failed to download Piper model: {e}")
            # Clean up partial downloads
            if model_file.exists():
                model_file.unlink()
            if config_file.exists():
                config_file.unlink()
            return None

    def synthesize(
        self,
        text: str,
        sample_rate: int = DEFAULT_SAMPLE_RATE,
    ) -> np.ndarray:
        """
        Synthesize speech from text.

        Args:
            text: Text to synthesize.
            sample_rate: Target sample rate (resampling done if needed).

        Returns:
            Audio data as numpy array of int16 samples.

        Raises:
            ImportError: If piper-tts is not installed.
            RuntimeError: If synthesis fails.
        """
        if not self._ensure_initialized():
            raise RuntimeError("Failed to initialize Piper TTS")

        try:
            # Synthesize to WAV in memory using Piper's synthesize_wav
            wav_buffer = io.BytesIO()
            with wave.open(wav_buffer, "wb") as wav_file:
                self._piper.synthesize_wav(text, wav_file)

            # Read back the audio data
            wav_buffer.seek(0)
            with wave.open(wav_buffer, "rb") as wav_file:
                frames = wav_file.readframes(wav_file.getnframes())
                audio_data = np.frombuffer(frames, dtype=np.int16)
                native_rate = wav_file.getframerate()

            # Resample if needed
            if native_rate != sample_rate:
                duration = len(audio_data) / native_rate
                new_length = int(duration * sample_rate)
                indices = np.linspace(0, len(audio_data) - 1, new_length)
                audio_data = np.interp(indices, np.arange(len(audio_data)), audio_data).astype(np.int16)

            return audio_data

        except Exception as e:
            raise RuntimeError(f"TTS synthesis failed: {e}") from e

    @property
    def voice(self) -> str:
        """Get the current voice model name."""
        return self._voice


# ==================== AUDIO STREAM RECEIVER ====================


class AudioStreamReceiver:
    """
    Receives and buffers audio from robot's microphone.

    Designed to work with robot.subscribe_audio_stream() callback.

    Example:
        >>> receiver = AudioStreamReceiver()
        >>> sub = robot.subscribe_audio_stream(receiver.on_audio)
        >>> time.sleep(10)
        >>> sub.unsubscribe()
        >>> audio = receiver.get_audio()
        >>> receiver.save_wav("recording.wav")
    """

    def __init__(
        self,
        sample_rate: int = DEFAULT_SAMPLE_RATE,
        max_buffer_seconds: float = 300.0,
    ):
        """
        Initialize audio stream receiver.

        Args:
            sample_rate: Expected sample rate in Hz.
            max_buffer_seconds: Maximum audio to buffer (default: 5 minutes).
        """
        self.sample_rate = sample_rate
        self.max_samples = int(max_buffer_seconds * sample_rate)
        self._buffer: List[int] = []
        self._lock = threading.Lock()

    def on_audio(self, samples: Union[List[int], np.ndarray]) -> None:
        """
        Callback for incoming audio data.

        Pass this method directly to robot.subscribe_audio_stream().

        Args:
            samples: Audio samples (int16).
        """
        if isinstance(samples, np.ndarray):
            samples = samples.tolist()

        with self._lock:
            self._buffer.extend(samples)
            # Trim if exceeds max buffer
            if len(self._buffer) > self.max_samples:
                excess = len(self._buffer) - self.max_samples
                self._buffer = self._buffer[excess:]

    def get_audio(self) -> np.ndarray:
        """Get buffered audio as numpy array of int16."""
        with self._lock:
            return np.array(self._buffer, dtype=np.int16)

    def get_audio_float(self) -> np.ndarray:
        """Get buffered audio as normalized float32 in range [-1, 1]."""
        audio = self.get_audio()
        return audio.astype(np.float32) / 32768.0

    def clear(self) -> None:
        """Clear the audio buffer."""
        with self._lock:
            self._buffer.clear()

    @property
    def duration(self) -> float:
        """Get duration of buffered audio in seconds."""
        with self._lock:
            return len(self._buffer) / self.sample_rate

    @property
    def sample_count(self) -> int:
        """Get number of samples in buffer."""
        with self._lock:
            return len(self._buffer)

    def save_wav(self, filepath: Union[str, Path]) -> None:
        """
        Save buffered audio to WAV file.

        Args:
            filepath: Output file path.
        """
        audio = self.get_audio()
        with wave.open(str(filepath), "wb") as wav_file:
            wav_file.setnchannels(DEFAULT_CHANNELS)
            wav_file.setsampwidth(DEFAULT_SAMPLE_WIDTH)
            wav_file.setframerate(self.sample_rate)
            wav_file.writeframes(audio.tobytes())


# ==================== UTILITY FUNCTIONS ====================


def load_audio_file(filepath: Union[str, Path]) -> Tuple[np.ndarray, int]:
    """
    Load audio from a WAV file.

    Args:
        filepath: Path to WAV file.

    Returns:
        Tuple of (audio_data as int16 numpy array, sample_rate).
    """
    with wave.open(str(filepath), "rb") as wav_file:
        sample_rate = wav_file.getframerate()
        n_channels = wav_file.getnchannels()
        sample_width = wav_file.getsampwidth()
        frames = wav_file.readframes(wav_file.getnframes())

        # Convert to numpy
        if sample_width == 1:
            audio = np.frombuffer(frames, dtype=np.uint8).astype(np.int16) * 256 - 32768
        elif sample_width == 2:
            audio = np.frombuffer(frames, dtype=np.int16)
        elif sample_width == 4:
            audio = (np.frombuffer(frames, dtype=np.int32) / 65536).astype(np.int16)
        else:
            raise ValueError(f"Unsupported sample width: {sample_width}")

        # Convert stereo to mono if needed
        if n_channels == 2:
            audio = audio.reshape(-1, 2).mean(axis=1).astype(np.int16)
        elif n_channels > 2:
            audio = audio.reshape(-1, n_channels)[:, 0]  # Take first channel

        return audio, sample_rate


def save_audio_file(
    filepath: Union[str, Path],
    audio: np.ndarray,
    sample_rate: int = DEFAULT_SAMPLE_RATE,
) -> None:
    """
    Save audio to a WAV file.

    Args:
        filepath: Output file path.
        audio: Audio data as numpy array (int16).
        sample_rate: Sample rate in Hz.
    """
    audio_int16 = audio.astype(np.int16)
    with wave.open(str(filepath), "wb") as wav_file:
        wav_file.setnchannels(DEFAULT_CHANNELS)
        wav_file.setsampwidth(DEFAULT_SAMPLE_WIDTH)
        wav_file.setframerate(sample_rate)
        wav_file.writeframes(audio_int16.tobytes())


def resample_audio(
    audio: np.ndarray,
    orig_rate: int,
    target_rate: int = DEFAULT_SAMPLE_RATE,
) -> np.ndarray:
    """
    Resample audio to target sample rate.

    Args:
        audio: Audio data as int16 numpy array.
        orig_rate: Original sample rate.
        target_rate: Target sample rate.

    Returns:
        Resampled audio as int16 numpy array.
    """
    if orig_rate == target_rate:
        return audio

    duration = len(audio) / orig_rate
    new_length = int(duration * target_rate)
    indices = np.linspace(0, len(audio) - 1, new_length)
    return np.interp(indices, np.arange(len(audio)), audio).astype(np.int16)
