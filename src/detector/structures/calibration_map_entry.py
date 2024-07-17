from .calibration_map_value import CalibrationMapValue
from src.common.structures import ImageResolution
from pydantic import BaseModel, Field


class CalibrationMapEntry(BaseModel):
    key: ImageResolution = Field()
    value: CalibrationMapValue = Field()
