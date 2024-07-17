from enum import StrEnum
from typing import Final


class CalibrationImageState(StrEnum):
    IGNORE: Final[int] = "ignore"
    SELECT: Final[int] = "select"
    DELETE: Final[int] = "delete"  # stage for deletion
