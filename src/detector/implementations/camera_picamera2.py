from ..exceptions import MCTDetectorRuntimeError
from ..interfaces import AbstractCamera
from ..structures import \
    CameraConfiguration, \
    CameraStatus
from src.common import StatusMessageSource
from src.common.structures import \
    ImageResolution, \
    KeyValueSimpleAbstract, \
    KeyValueSimpleAny, \
    KeyValueSimpleBool, \
    KeyValueSimpleFloat, \
    KeyValueSimpleInt, \
    KeyValueMetaAbstract, \
    KeyValueMetaBool, \
    KeyValueMetaFloat, \
    KeyValueMetaInt
import datetime
import logging
import numpy
from picamera2 import Picamera2
from picamera2.configuration import CameraConfiguration as Picamera2Configuration
from typing import Final


logger = logging.getLogger(__name__)

_RANGE_MINIMUM_INDEX: Final[int] = 0
_RANGE_MAXIMUM_INDEX: Final[int] = 1
_RANGE_DEFAULT_INDEX: Final[int] = 2
_MICROSECONDS_PER_SECOND: Final[int] = 1000000

_CAMERA_RESOLUTION_KEY: Final[str] = "size"

_CAMERA_CONTROLS_KEY: Final[str] = "controls"
_CAMERA_FPS_KEY: Final[str] = "FramesPerSecond"
_CAMERA_FPS_DEFAULT: Final[float] = 30.0
_CAMERA_FPS_RANGE_MINIMUM: Final[float] = 1.0
_CAMERA_FPS_RANGE_MAXIMUM: Final[float] = 30.0  # Appears to be the limit for global shutter camera (?)
_CAMERA_FPS_DIGIT_COUNT: Final[int] = 1
_CAMERA_AUTO_EXPOSURE_KEY: Final[str] = "AutoExposureGain"
_CAMERA_GAIN_KEY: Final[str] = "Gain"
_CAMERA_GAIN_MINIMUM: Final[float] = 1.0   # From self._camera.camera_controls with global shutter camera
_CAMERA_GAIN_MAXIMUM: Final[float] = 16.0  # From self._camera.camera_controls with global shutter camera
_CAMERA_GAIN_DEFAULT: Final[float] = 1.0   # Arbitrarily-chosen value
_CAMERA_GAIN_DIGIT_COUNT: Final[int] = 2
_CAMERA_EXPOSURE_KEY: Final[str] = "Exposure (us)"
_CAMERA_EXPOSURE_MINIMUM: Final[int] = 0      # From self._camera.camera_controls with global shutter camera
_CAMERA_EXPOSURE_MAXIMUM: Final[int] = 66666  # From self._camera.camera_controls with global shutter camera
_CAMERA_EXPOSURE_DEFAULT: Final[int] = 33333  # Arbitrarily-chosen value
_CAMERA_BRIGHTNESS_KEY: Final[str] = "Brightness"
_CAMERA_BRIGHTNESS_MINIMUM: Final[float] = -1.0  # From PiCamera2 manual
_CAMERA_BRIGHTNESS_MAXIMUM: Final[float] = 1.0  # From PiCamera2 manual
_CAMERA_BRIGHTNESS_DEFAULT: Final[float] = 0.0  # From PiCamera2 manual
_CAMERA_BRIGHTNESS_DIGIT_COUNT: Final[int] = 2
_CAMERA_CONTRAST_KEY: Final[str] = "Contrast"
_CAMERA_CONTRAST_MINIMUM: Final[float] = 0.0  # From PiCamera2 manual
_CAMERA_CONTRAST_MAXIMUM: Final[float] = 32.0  # From PiCamera2 manual
_CAMERA_CONTRAST_DEFAULT: Final[float] = 1.0  # From PiCamera2 manual
_CAMERA_CONTRAST_DIGIT_COUNT: Final[int] = 2
_CAMERA_SHARPNESS_KEY: Final[str] = "Sharpness"
_CAMERA_SHARPNESS_MINIMUM: Final[float] = 0.0  # From PiCamera2 manual
_CAMERA_SHARPNESS_MAXIMUM: Final[float] = 16.0  # From PiCamera2 manual
_CAMERA_SHARPNESS_DEFAULT: Final[float] = 1.0  # From PiCamera2 manual
_CAMERA_SHARPNESS_DIGIT_COUNT: Final[int] = 2

# _PICAMERA2_FRAME_RATE_KEY: Final[str] = "FrameRate"  # For consistency, use FrameDurationLimits instead
_PICAMERA2_FRAME_DURATION_LIMITS_KEY: Final[str] = "FrameDurationLimits"
_PICAMERA2_AEC_AGC_KEY: Final[str] = "AeEnable"  # Automatic exposure/gain adjustment
_PICAMERA2_GAIN_KEY: Final[str] = "AnalogueGain"
_PICAMERA2_EXPOSURE_KEY: Final[str] = "ExposureTime"  # Microseconds
_PICAMERA2_BRIGHTNESS_KEY: Final[str] = "Brightness"
_PICAMERA2_CONTRAST_KEY: Final[str] = "Contrast"
_PICAMERA2_SHARPNESS_KEY: Final[str] = "Sharpness"


class Picamera2Camera(AbstractCamera):

    _camera: Picamera2
    _camera_configuration: Picamera2Configuration

    _image: numpy.ndarray | None
    _image_timestamp_utc: datetime.datetime

    def __init__(
        self,
        configuration: CameraConfiguration,
        status_message_source: StatusMessageSource
    ):
        super().__init__(
            configuration=configuration,
            status_message_source=status_message_source)
        self._image = None
        self._image_timestamp_utc = datetime.datetime.min
        self._camera = Picamera2()
        self._camera_configuration = self._camera.create_video_configuration()
        self.set_status(CameraStatus.STOPPED)

    def get_changed_timestamp(self) -> datetime.datetime:
        return self._image_timestamp_utc

    def get_image(self) -> numpy.ndarray:
        if self._image is None:
            raise MCTDetectorRuntimeError(message="There is no captured image.")
        return self._image

    def get_resolution(self) -> ImageResolution:
        resolution: tuple[int, int] = self._camera_configuration[_CAMERA_RESOLUTION_KEY]
        return ImageResolution(x_px=resolution[0], y_px=resolution[1])

    def get_parameters(self, **_kwargs) -> list[KeyValueMetaAbstract]:
        if self.get_status() != CameraStatus.RUNNING:
            raise MCTDetectorRuntimeError(message="The capture is not active, and properties cannot be retrieved.")

        current_controls: dict = {
            # Custom settings shall override default values
            **{control[0]: control[1][_RANGE_DEFAULT_INDEX]
               for control in self._camera.camera_controls.items()},
            **self._camera_configuration[_CAMERA_CONTROLS_KEY]}

        return_value: list[KeyValueMetaAbstract] = list()

        frame_limits_us: tuple[float, float] = current_controls[_PICAMERA2_FRAME_DURATION_LIMITS_KEY]  # max, min
        frame_duration_us: float = (frame_limits_us[0] + frame_limits_us[1]) / 2.0
        frames_per_second: int = int(_MICROSECONDS_PER_SECOND / round(frame_duration_us))
        return_value.append(KeyValueMetaFloat(
            key=_CAMERA_FPS_KEY,
            value=frames_per_second,
            range_minimum=_CAMERA_FPS_RANGE_MINIMUM,
            range_maximum=_CAMERA_FPS_RANGE_MAXIMUM,
            digit_count=_CAMERA_FPS_DIGIT_COUNT))

        return_value.append(KeyValueMetaBool(
            key=_CAMERA_AUTO_EXPOSURE_KEY,
            value=current_controls[_PICAMERA2_AEC_AGC_KEY]))

        return_value.append(KeyValueMetaFloat(
            key=_CAMERA_GAIN_KEY,
            value=current_controls[_PICAMERA2_GAIN_KEY],
            range_minimum=_CAMERA_GAIN_MINIMUM,
            range_maximum=_CAMERA_GAIN_MAXIMUM,
            digit_count=_CAMERA_GAIN_DIGIT_COUNT))

        return_value.append(KeyValueMetaInt(
            key=_CAMERA_EXPOSURE_KEY,
            value=current_controls[_PICAMERA2_EXPOSURE_KEY],
            range_minimum=_CAMERA_EXPOSURE_MINIMUM,
            range_maximum=_CAMERA_EXPOSURE_MAXIMUM))

        return_value.append(KeyValueMetaFloat(
            key=_CAMERA_BRIGHTNESS_KEY,
            value=current_controls[_PICAMERA2_BRIGHTNESS_KEY],
            range_minimum=_CAMERA_BRIGHTNESS_MINIMUM,
            range_maximum=_CAMERA_BRIGHTNESS_MAXIMUM,
            digit_count=_CAMERA_BRIGHTNESS_DIGIT_COUNT))

        return_value.append(KeyValueMetaFloat(
            key=_CAMERA_CONTRAST_KEY,
            value=current_controls[_PICAMERA2_CONTRAST_KEY],
            range_minimum=_CAMERA_CONTRAST_MINIMUM,
            range_maximum=_CAMERA_CONTRAST_MAXIMUM,
            digit_count=_CAMERA_CONTRAST_DIGIT_COUNT))

        return_value.append(KeyValueMetaFloat(
            key=_CAMERA_SHARPNESS_KEY,
            value=current_controls[_PICAMERA2_SHARPNESS_KEY],
            range_minimum=_CAMERA_SHARPNESS_MINIMUM,
            range_maximum=_CAMERA_SHARPNESS_MAXIMUM,
            digit_count=_CAMERA_SHARPNESS_DIGIT_COUNT))

        return return_value

    @staticmethod
    def get_type_identifier() -> str:
        return "picamera2"

    # noinspection DuplicatedCode
    def set_parameters(self, parameters: list[KeyValueSimpleAny]) -> None:

        mismatched_keys: list[str] = list()

        key_value: KeyValueSimpleAbstract
        for key_value in parameters:
            if key_value.key == _CAMERA_FPS_KEY:
                if not isinstance(key_value, KeyValueSimpleFloat):
                    mismatched_keys.append(key_value.key)
                    continue
                limit: int = int(round(_MICROSECONDS_PER_SECOND / key_value.value))
                limits: tuple[int, int] = (limit, limit)
                self._camera_configuration[_CAMERA_CONTROLS_KEY][_PICAMERA2_FRAME_DURATION_LIMITS_KEY] = limits
            elif key_value.key == _CAMERA_AUTO_EXPOSURE_KEY:
                if not isinstance(key_value, KeyValueSimpleBool):
                    mismatched_keys.append(key_value.key)
                    continue
                self._camera_configuration[_CAMERA_CONTROLS_KEY][_PICAMERA2_AEC_AGC_KEY] = key_value.value
            elif key_value.key == _CAMERA_GAIN_KEY:
                if not isinstance(key_value, KeyValueSimpleFloat):
                    mismatched_keys.append(key_value.key)
                    continue
                self._camera_configuration[_CAMERA_CONTROLS_KEY][_PICAMERA2_GAIN_KEY] = key_value.value
            elif key_value.key == _CAMERA_EXPOSURE_KEY:
                if not isinstance(key_value, KeyValueSimpleInt):
                    mismatched_keys.append(key_value.key)
                    continue
                self._camera_configuration[_CAMERA_CONTROLS_KEY][_PICAMERA2_EXPOSURE_KEY] = key_value.value
            elif key_value.key == _CAMERA_BRIGHTNESS_KEY:
                if not isinstance(key_value, KeyValueSimpleFloat):
                    mismatched_keys.append(key_value.key)
                    continue
                self._camera_configuration[_CAMERA_CONTROLS_KEY][_PICAMERA2_BRIGHTNESS_KEY] = key_value.value
            elif key_value.key == _CAMERA_CONTRAST_KEY:
                if not isinstance(key_value, KeyValueSimpleFloat):
                    mismatched_keys.append(key_value.key)
                    continue
                self._camera_configuration[_CAMERA_CONTROLS_KEY][_PICAMERA2_CONTRAST_KEY] = key_value.value
            elif key_value.key == _CAMERA_SHARPNESS_KEY:
                if not isinstance(key_value, KeyValueSimpleFloat):
                    mismatched_keys.append(key_value.key)
                    continue
                self._camera_configuration[_CAMERA_CONTROLS_KEY][_PICAMERA2_SHARPNESS_KEY] = key_value.value
            else:
                mismatched_keys.append(key_value.key)

        if len(mismatched_keys) > 0:
            raise MCTDetectorRuntimeError(
                message=f"The following parameters could not be applied due to key mismatch: {str(mismatched_keys)}")

        if self.get_status() == CameraStatus.RUNNING:
            self._camera.stop()
            self._camera.configure(self._camera_configuration)
            self._camera.start()
            self._image = self._camera.capture_array()

    def start(self) -> None:

        if _PICAMERA2_FRAME_DURATION_LIMITS_KEY not in self._camera_configuration:
            minr = int(round(_MICROSECONDS_PER_SECOND / _CAMERA_FPS_DEFAULT))
            maxr = int(round(_MICROSECONDS_PER_SECOND / _CAMERA_FPS_DEFAULT))
            self._camera_configuration[_CAMERA_CONTROLS_KEY][_PICAMERA2_FRAME_DURATION_LIMITS_KEY] = (minr, maxr)

        if _PICAMERA2_AEC_AGC_KEY not in self._camera_configuration:
            self._camera_configuration[_CAMERA_CONTROLS_KEY][_PICAMERA2_AEC_AGC_KEY] = True

        if _PICAMERA2_GAIN_KEY not in self._camera_configuration:
            self._camera_configuration[_CAMERA_CONTROLS_KEY][_PICAMERA2_GAIN_KEY] = _CAMERA_GAIN_DEFAULT

        if _PICAMERA2_EXPOSURE_KEY not in self._camera_configuration:
            self._camera_configuration[_CAMERA_CONTROLS_KEY][_PICAMERA2_EXPOSURE_KEY] = _CAMERA_EXPOSURE_DEFAULT

        if _PICAMERA2_BRIGHTNESS_KEY not in self._camera_configuration:
            self._camera_configuration[_CAMERA_CONTROLS_KEY][_PICAMERA2_BRIGHTNESS_KEY] = _CAMERA_BRIGHTNESS_DEFAULT

        if _PICAMERA2_CONTRAST_KEY not in self._camera_configuration:
            self._camera_configuration[_CAMERA_CONTROLS_KEY][_PICAMERA2_CONTRAST_KEY] = _CAMERA_CONTRAST_DEFAULT

        if _PICAMERA2_SHARPNESS_KEY not in self._camera_configuration:
            self._camera_configuration[_CAMERA_CONTROLS_KEY][_PICAMERA2_SHARPNESS_KEY] = _CAMERA_SHARPNESS_DEFAULT

        self._camera.configure(self._camera_configuration)
        self._camera.start()
        self._image = self._camera.capture_array()
        self.set_status(CameraStatus.RUNNING)

    def stop(self) -> None:
        if self._image is not None:
            self._image = None
        self.set_status(CameraStatus.STOPPED)
        self._camera.stop()

    def update(self) -> None:
        self._image = self._camera.capture_array()

        if self._image is None:
            message: str = "Failed to grab frame."
            self.add_status_message(severity="error", message=message)
            self.set_status(CameraStatus.FAILURE)
            raise MCTDetectorRuntimeError(message=message)

        self._image_timestamp_utc = datetime.datetime.utcnow()
