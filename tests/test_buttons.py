"""Offline tests for pib_sdk.features.buttons (fake backend, no network)."""

from __future__ import annotations

from pib_sdk.features.buttons import ButtonBinding, list_button_bindings, set_button_program


class _FakeBackend:
    def __init__(self):
        self.set_button_programs_calls: list = []

    def list_button_programs(self):
        return [
            {"brickletNumber": 5, "brickletUid": "abc", "programNumber": None},
            {"brickletNumber": 6, "brickletUid": "def", "programNumber": "prog-1"},
        ]

    def set_button_programs(self, updates):
        self.set_button_programs_calls.append(updates)
        program_number = updates[0]["programNumber"]
        return [{"brickletNumber": 5, "brickletUid": "abc", "programNumber": program_number}]


def test_list_button_bindings_converts_to_dataclasses():
    bindings = list_button_bindings(_FakeBackend())
    assert bindings == [
        ButtonBinding(bricklet_number=5, bricklet_uid="abc", program_number=None),
        ButtonBinding(bricklet_number=6, bricklet_uid="def", program_number="prog-1"),
    ]


def test_set_button_program_sends_a_single_item_update_list():
    backend = _FakeBackend()

    bindings = set_button_program(backend, bricklet_number=5, program_number="prog-1")

    assert backend.set_button_programs_calls == [[{"brickletNumber": 5, "programNumber": "prog-1"}]]
    assert bindings == [
        ButtonBinding(bricklet_number=5, bricklet_uid="abc", program_number="prog-1")
    ]


def test_set_button_program_can_unassign_with_none():
    backend = _FakeBackend()

    set_button_program(backend, bricklet_number=5, program_number=None)

    assert backend.set_button_programs_calls == [[{"brickletNumber": 5, "programNumber": None}]]
