#!/usr/bin/env python3
"""Live robot: run a saved Blockly program and optionally bind a button."""

import argparse

from pib_sdk.backend import BackendClient
from pib_sdk.features.buttons import set_button_program
from pib_sdk.features.programs import Programs, list_programs


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="pib.local")
    parser.add_argument("--program-number")
    parser.add_argument("--button", type=int)
    parser.add_argument("--timeout", type=float, default=60.0)
    args = parser.parse_args()

    backend = BackendClient(args.host)
    available = list_programs(backend)
    if not available:
        raise RuntimeError("no Blockly programs are saved")
    program_number = args.program_number or available[0].program_number

    def output(line: str, is_stderr: bool) -> None:
        print("ERR" if is_stderr else "OUT", line)

    with Programs(args.host) as programs:
        result = programs.run(program_number, timeout=args.timeout, on_output=output)
    print("status:", result.status, "exit:", result.exit_code, "timed out:", result.timed_out)

    if args.button is not None:
        set_button_program(
            backend, bricklet_number=args.button, program_number=program_number
        )
        print("bound button", args.button, "to", program_number)


if __name__ == "__main__":
    main()
