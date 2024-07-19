from typing import Final
from enum import StrEnum


class MarkerStatus(StrEnum):
    STOPPED: Final[int] = "STOPPED"
    RUNNING: Final[int] = "RUNNING"
    FAILURE: Final[int] = "FAILURE"
