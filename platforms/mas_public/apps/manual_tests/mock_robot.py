from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from src.common.config_loader import load_all_configs
from src.common.logger import setup_logging
from src.common.messages import ControlCommand
from src.messaging.factory import TransportFactory
from src.messaging.topics import CONTROL_COMMAND


def main() -> None:
    configs = load_all_configs()
    logger = setup_logging("mock_robot")
    subscriber = TransportFactory(configs["system"]["network"], logger).create_subscriber(
        "control_command", [CONTROL_COMMAND]
    )
    count = 0
    start = time.monotonic()
    last = start
    try:
        while True:
            received = subscriber.receive(timeout_ms=1000)
            if received is None:
                logger.warning("no control command received")
                continue
            _, payload = received
            command = ControlCommand.from_dict(payload)
            count += 1
            now = time.monotonic()
            fps = count / max(now - start, 1e-6)
            if now - last >= 1.0:
                logger.info("received %.1f cmd/s latest=%s", fps, command.commands)
                last = now
    except KeyboardInterrupt:
        logger.info("mock robot stopped")
    finally:
        subscriber.close()


if __name__ == "__main__":
    main()

