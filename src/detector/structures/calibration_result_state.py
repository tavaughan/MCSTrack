from enum import StrEnum
from typing import Final


class CalibrationResultState(StrEnum):
    # indicate to use this calibration (as opposed to simply storing it)
    # normally there shall only ever be one ACTIVE calibration for a given image resolution
    ACTIVE: Final[str] = "active"

    # store the calibration, but don't mark it for use
    RETAIN: Final[str] = "retain"

    # stage for deletion
    DELETE: Final[str] = "delete"
