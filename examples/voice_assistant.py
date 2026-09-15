#!/usr/bin/env python3
"""Live robot: send an assistant message and poll for its reply."""

import argparse
import time

from pib_sdk.backend import BackendClient
from pib_sdk.features.assistant import (
    Assistant,
    create_chat,
    list_chat_messages,
    list_personalities,
)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="pib.local")
    parser.add_argument("--message", default="Introduce yourself in one sentence.")
    parser.add_argument("--timeout", type=float, default=60.0)
    args = parser.parse_args()

    backend = BackendClient(args.host)
    personalities = list_personalities(backend)
    if not personalities:
        raise RuntimeError("no assistant personalities are configured")
    chat = create_chat(
        backend, topic="pib-sdk example", personality_id=personalities[0].personality_id
    )
    with Assistant(args.host) as assistant:
        if not assistant.set_state(turned_on=True, chat_id=chat.chat_id):
            raise RuntimeError("assistant state change was rejected")
        if not assistant.send_message(chat.chat_id, args.message):
            raise RuntimeError("message was rejected")

    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline:
        replies = [
            message
            for message in list_chat_messages(backend, chat.chat_id)
            if not message.is_user
        ]
        if replies:
            print(replies[-1].content)
            return
        time.sleep(1)
    raise TimeoutError("assistant reply did not arrive")


if __name__ == "__main__":
    main()
