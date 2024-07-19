from .calibration_configuration import CalibratorConfiguration
from .camera_configuration import CameraConfiguration
from .marker_configuration import MarkerConfiguration
from pydantic import BaseModel, Field


class DetectorConfiguration(BaseModel):
    """
    Top-level schema for Detector initialization data
    """
    calibrator_configuration: CalibratorConfiguration = Field()
    camera_configuration: CameraConfiguration = Field()
    marker_configuration: MarkerConfiguration = Field()
