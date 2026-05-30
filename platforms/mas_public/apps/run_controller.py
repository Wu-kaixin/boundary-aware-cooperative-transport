import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.controller.controller_module import ControllerModule


if __name__ == "__main__":
    ControllerModule().run()
