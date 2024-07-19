from typing import Final
from enum import StrEnum


class CameraStatus(StrEnum):
    STOPPED: Final[int] = "STOPPED"
    RUNNING: Final[int] = "RUNNING"
    FAILURE: Final[int] = "FAILURE"

    def in_runnable_state(self):
        return self == CameraStatus.RUNNING
