"""
Speech service helper for pib-sdk using roslibpy.

Provides class `speak` with a synchronous `.say(...)` method that mirrors the
PlayAudioFromSpeech ROS2 service semantics.

Service:
  name:   play_audio_from_speech
  type:   datatypes/PlayAudioFromSpeech
Request fields:
  - speech:   string
  - join:     bool
  - gender:   string   ("Female" | "Male")
  - language: string   ("German" | "English")

Examples:
    from pib_sdk.speech import speak

    sp = speak(host="localhost", port=9090, debug=False)
    try:
        sp.say("hello world")                  # default voice: Emma (Female, English)
        sp.say("guten tag", voice="Hannah")    # preset -> Female/German
        sp.say("hi", gender="Male", language="English")  # explicit
    finally:
        sp.close()
"""

from __future__ import annotations

import threading
import time
from typing import Optional, Tuple, Dict, Any

import roslibpy

# Map friendly voice names to (gender, language)
_VOICE_PRESETS: Dict[str, Tuple[str, str]] = {
    "Hannah": ("Female", "German"),
    "Daniel": ("Male", "German"),
    "Emma":   ("Female", "English"),
    "Brian":  ("Male", "English"),
}

class speak:
    """
    Thin wrapper around a ROS Bridge service to speak text via TTS.
    Uses synchronous (blocking) service calls with a configurable timeout.
    """

    def __init__(self, host: str = "localhost", port: int = 9090, debug: bool = False,
                 service_name: str = "play_audio_from_speech",
                 service_type: str = "datatypes/PlayAudioFromSpeech",
                 connect_timeout: float = 10.0) -> None:
        self._host = host
        self._port = port
        self._debug = debug
        self._service_name = service_name
        self._service_type = service_type

        self._ros = roslibpy.Ros(host=self._host, port=self._port)
        self._ros.run()

        # Wait a bit for connection
        deadline = time.time() + float(connect_timeout)
        while (not self._ros.is_connected) and (time.time() < deadline):
            time.sleep(0.05)

        if not self._ros.is_connected:
            raise RuntimeError(
                f"Could not connect to rosbridge at {self._host}:{self._port}"
            )

        self._svc = roslibpy.Service(self._ros, self._service_name, self._service_type)

    def __enter__(self) -> "speak":
        return self

    def __exit__(self, exc_type, exc, tb):
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
        voice: Optional[str] = None,
        gender: Optional[str] = None,
        language: Optional[str] = None,
        join: bool = True,
        timeout: float = 30.0,
    ) -> Dict[str, Any]:
        """
        Speak `text` using the TTS service.

        You can:
          - pass a `voice` in {"Hannah","Daniel","Emma","Brian"}, OR
          - pass explicit `gender` ("Female"/"Male") AND `language` ("German"/"English").

        If neither is provided, defaults to Emma (Female/English).
        """
        if not isinstance(text, str) or not text.strip():
            raise ValueError("`text` must be a non-empty string.")

        # Resolve voice/gender/language with a sensible default
        if voice:
            preset = _VOICE_PRESETS.get(voice)
            if not preset:
                raise ValueError(f"Unrecognized voice '{voice}'. Valid: {', '.join(_VOICE_PRESETS.keys())}")
            g, lang = preset
        elif gender and language:
            g, lang = str(gender), str(language)
        else:
            # default when nothing provided
            g, lang = _VOICE_PRESETS["Emma"]

        req = roslibpy.ServiceRequest(
            {
                "speech": text,
                "join": bool(join),
                "gender": g,
                "language": lang,
            }
        )

        done = threading.Event()
        result_holder: Dict[str, Any] = {"ok": False, "response": None, "error": None}

        def _on_success(response):
            result_holder["ok"] = True
            result_holder["response"] = response
            done.set()

        def _on_error(error):
            result_holder["ok"] = False
            result_holder["error"] = error
            done.set()

        if not self._ros.is_connected:
            raise RuntimeError("rosbridge connection is not active.")

        # roslibpy API: callback / errback (not success_callback / error_callback)
        self._svc.call(req, callback=_on_success, errback=_on_error, timeout=timeout)

        done.wait(timeout=max(0.01, float(timeout)))

        if not done.is_set():
            raise TimeoutError(
                f"Service call to '{self._service_name}' did not return within {timeout} seconds."
            )

        if not result_holder["ok"]:
            raise RuntimeError(f"Service error from '{self._service_name}': {result_holder['error']}")

        return result_holder["response"] or {}
