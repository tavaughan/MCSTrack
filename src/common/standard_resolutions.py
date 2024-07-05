from .structures import ImageResolution
from typing import Final


class StandardResolutions:
    RES_640X360: Final[ImageResolution] = ImageResolution(x_px=640, y_px=360)
    RES_640X480: Final[ImageResolution] = ImageResolution(x_px=640, y_px=480)
    RES_800X600: Final[ImageResolution] = ImageResolution(x_px=800, y_px=600)
    RES_1024X768: Final[ImageResolution] = ImageResolution(x_px=1024, y_px=768)
    RES_1280X720: Final[ImageResolution] = ImageResolution(x_px=1280, y_px=720)
    RES_1280X800: Final[ImageResolution] = ImageResolution(x_px=1280, y_px=800)
    RES_1280X1024: Final[ImageResolution] = ImageResolution(x_px=1280, y_px=1024)
    RES_1920X1080: Final[ImageResolution] = ImageResolution(x_px=1920, y_px=1080)

    @staticmethod
    def as_list():
        return [
            StandardResolutions.RES_640X360,
            StandardResolutions.RES_640X480,
            StandardResolutions.RES_800X600,
            StandardResolutions.RES_1024X768,
            StandardResolutions.RES_1280X720,
            StandardResolutions.RES_1280X800,
            StandardResolutions.RES_1280X1024,
            StandardResolutions.RES_1920X1080]
