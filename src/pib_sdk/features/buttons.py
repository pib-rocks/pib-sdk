"""RGB-button -> program bindings, sourced from pib-backend/Cerebra.

pib's RGB buttons are hardware-fixed status lights with no color/brightness
API: pib-backend's ``rgb_button_control`` node hardcodes blue while idle,
green while a program runs, red on error, and off when unassigned -- there is
nothing to "set" about their appearance. What *is* controllable is which
saved program (see :mod:`pib_sdk.features.programs`) each button starts when
pressed; that binding is what this module reads and writes.

Example
-------
    from pib_sdk.backend import BackendClient
    from pib_sdk.features.buttons import list_button_bindings, set_button_program

    backend = BackendClient(host="localhost")
    for binding in list_button_bindings(backend):
        print(binding.bricklet_number, "->", binding.program_number)

    set_button_program(backend, bricklet_number=5, program_number=program_number)
"""

from __future__ import annotations

from dataclasses import dataclass

from pib_sdk.backend import BackendClient


@dataclass(frozen=True)
class ButtonBinding:
    """Which program (if any) a physical RGB button starts when pressed."""

    bricklet_number: int
    bricklet_uid: str
    program_number: str | None


def _to_binding(item: dict) -> ButtonBinding:
    return ButtonBinding(
        bricklet_number=item["brickletNumber"],
        bricklet_uid=item["brickletUid"],
        program_number=item.get("programNumber"),
    )


def list_button_bindings(backend: BackendClient) -> list[ButtonBinding]:
    """List every RGB button and the program (if any) bound to it."""
    return [_to_binding(item) for item in backend.list_button_programs()]


def set_button_program(
    backend: BackendClient, *, bricklet_number: int, program_number: str | None
) -> list[ButtonBinding]:
    """Bind ``bricklet_number`` to start ``program_number`` when pressed.

    Pass ``program_number=None`` to unassign the button. Returns every
    button's binding after the update (matching the backend's response).
    """
    updated = backend.set_button_programs(
        [{"brickletNumber": bricklet_number, "programNumber": program_number}]
    )
    return [_to_binding(item) for item in updated]
