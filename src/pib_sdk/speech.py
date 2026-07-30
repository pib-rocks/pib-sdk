"""Speech-service helper for pib-sdk built on roslibpy.

Provides :class:`Speak` with a synchronous :meth:`Speak.say` method that mirrors
the ``PlayAudioFromSpeech`` ROS 2 service.

Service
-------
name:   ``play_audio_from_speech``
type:   ``datatypes/PlayAudioFromSpeech``

Request fields:
    * ``speech``   -- string
    * ``join``     -- bool
    * ``gender``   -- string ("Female" | "Male")
    * ``language`` -- string ("German" | "English")

Example
-------
    from pib_sdk.speech import Speak

    speaker = Speak(host="localhost", port=9090)
    try:
        speaker.say("hello world")                      # default: Emma (Female, English)
        speaker.say("guten tag", voice="Hannah")        # preset -> Female/German
        speaker.say("hi", gender="Male", language="English")
    finally:
        speaker.close()
"""

from __future__ import annotations

import threading
import time
from typing import Any

import roslibpy

# Map friendly voice names to (gender, language).
_VOICE_PRESETS: dict[str, tuple[str, str]] = {
    "Hannah": ("Female", "German"),
    "Daniel": ("Male", "German"),
    "Emma": ("Female", "English"),
    "Brian": ("Male", "English"),
}
_DEFAULT_VOICE = "Emma"


class Speak:
    """Thin wrapper around a ROS Bridge service that speaks text via TTS.

    Uses synchronous (blocking) service calls with a configurable timeout.
    """

    def __init__(
        self,
        host: str = "localhost",
        port: int = 9090,
        debug: bool = False,
        service_name: str = "play_audio_from_speech",
        service_type: str = "datatypes/PlayAudioFromSpeech",
        connect_timeout: float = 10.0,
    ) -> None:
        self._host = host
        self._port = port
        self._debug = debug
        self._service_name = service_name
        self._service_type = service_type

        self._ros = roslibpy.Ros(host=self._host, port=self._port)
        self._ros.run()

        deadline = time.time() + float(connect_timeout)
        while (not self._ros.is_connected) and (time.time() < deadline):
            time.sleep(0.05)

        if not self._ros.is_connected:
            raise RuntimeError(f"Could not connect to rosbridge at {self._host}:{self._port}")

        self._service = roslibpy.Service(self._ros, self._service_name, self._service_type)

    def __enter__(self) -> Speak:
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def close(self) -> None:
        """Close the underlying rosbridge connection."""
        try:
            if self._ros and self._ros.is_connected:
                self._ros.terminate()
        except Exception:
            pass

    def say(
        self,
        text: str,
        *,
        voice: str | None = None,
        gender: str | None = None,
        language: str | None = None,
        join: bool = True,
        timeout: float = 30.0,
    ) -> dict[str, Any]:
        """Speak ``text`` using the TTS service.

        Provide either a ``voice`` preset from ``{"Hannah", "Daniel", "Emma",
        "Brian"}``, or an explicit ``gender`` ("Female"/"Male") together with a
        ``language`` ("German"/"English"). If neither is given, defaults to
        Emma (Female/English).
        """
        if not isinstance(text, str) or not text.strip():
            raise ValueError("`text` must be a non-empty string.")

        gender_to_use, language_to_use = self._resolve_voice(voice, gender, language)

        if not self._ros.is_connected:
            raise RuntimeError("rosbridge connection is not active.")

        request = roslibpy.ServiceRequest(
            {
                "speech": text,
                "join": bool(join),
                "gender": gender_to_use,
                "language": language_to_use,
            }
        )

        done = threading.Event()
        result: dict[str, Any] = {"ok": False, "response": None, "error": None}

        def _on_success(response: Any) -> None:
            result["ok"] = True
            result["response"] = response
            done.set()

        def _on_error(error: Any) -> None:
            result["ok"] = False
            result["error"] = error
            done.set()

        self._service.call(request, callback=_on_success, errback=_on_error, timeout=timeout)
        done.wait(timeout=max(0.01, float(timeout)))

        if not done.is_set():
            raise TimeoutError(
                f"Service call to '{self._service_name}' did not return within {timeout} seconds."
            )
        if not result["ok"]:
            raise RuntimeError(f"Service error from '{self._service_name}': {result['error']}")

        return result["response"] or {}

    @staticmethod
    def _resolve_voice(
        voice: str | None, gender: str | None, language: str | None
    ) -> tuple[str, str]:
        if voice:
            preset = _VOICE_PRESETS.get(voice)
            if not preset:
                valid = ", ".join(_VOICE_PRESETS)
                raise ValueError(f"Unrecognized voice '{voice}'. Valid options: {valid}")
            return preset
        if gender and language:
            return str(gender), str(language)
        return _VOICE_PRESETS[_DEFAULT_VOICE]


# Backwards-compatible alias for the previous lowercase class name.
speak = Speak
