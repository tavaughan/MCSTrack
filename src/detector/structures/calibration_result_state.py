from enum import StrEnum
from typing import Final


class CalibrationResultState(StrEnum):
    RETAIN: Final[int] = "retain"
    DELETE: Final[int] = "delete"  # stage for deletion
