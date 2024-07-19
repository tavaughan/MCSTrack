from pydantic import BaseModel, Field
from typing import Union


class CameraConfiguration(BaseModel):
    driver: str = Field()
    capture_device: Union[str, int] = Field()  # Not used by all drivers (notably it IS used by OpenCV)
