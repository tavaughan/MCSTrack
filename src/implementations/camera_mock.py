from src.common import \
    Camera, \
    ImageResolution, \
    ImageUtils, \
    KeyValueSimpleAbstract, \
    KeyValueSimpleAny, \
    KeyValueSimpleFloat, \
    KeyValueMetaAbstract, \
    KeyValueMetaFloat, \
    MCTCameraRuntimeError, \
    SeverityLabel, \
    StatusMessageSource
import datetime
import logging
import numpy
from typing import Final


logger = logging.getLogger(__name__)


_MICROSECONDS_PER_SECOND: Final[int] = 1000000

_CAMERA_RESOLUTION_KEY: Final[str] = "size"
_CAMERA_FPS_KEY: Final[str] = "FramesPerSecond"
_CAMERA_FPS_DEFAULT: Final[float] = 30.0
_CAMERA_FPS_RANGE_MINIMUM: Final[float] = 1.0
_CAMERA_FPS_RANGE_MAXIMUM: Final[float] = 60.0  # Appears to be the limit for global shutter camera (?)
_CAMERA_FPS_DIGIT_COUNT: Final[int] = 1


class MockCamera(Camera):
    """
    The Mock classes are relatively simple implementations made for testing connectivity functionality.
    """

    _image: numpy.ndarray | None
    _image_timestamp_utc: datetime.datetime

    _current_frames_per_second: float
    _current_resolution: ImageResolution

    def __init__(
        self,
        configuration: Camera.Configuration,
        status_message_source: StatusMessageSource
    ):
        super().__init__(
            configuration=configuration,
            status_message_source=status_message_source)
        self._image = None
        self._image_timestamp_utc = datetime.datetime.min.replace(tzinfo=datetime.timezone.utc)
        self._current_resolution = ImageUtils.StandardResolutions.RES_640X480
        self._current_frames_per_second = 15.0
        self.set_status(Camera.Status.STOPPED)

    def get_changed_timestamp(self) -> datetime.datetime:
        return self._image_timestamp_utc

    def get_image(self) -> numpy.ndarray:
        if self._image is None:
            raise MCTCameraRuntimeError(message="There is no captured image.")
        return self._image

    def get_parameters(self, **_kwargs) -> list[KeyValueMetaAbstract]:
        if self.get_status() != Camera.Status.RUNNING:
            raise MCTCameraRuntimeError(message="The capture is not active, and properties cannot be retrieved.")

        return_value: list[KeyValueMetaAbstract] = list()

        return_value.append(KeyValueMetaFloat(
            key=_CAMERA_FPS_KEY,
            value=self._current_frames_per_second,
            range_minimum=_CAMERA_FPS_RANGE_MINIMUM,
            range_maximum=_CAMERA_FPS_RANGE_MAXIMUM,
            digit_count=_CAMERA_FPS_DIGIT_COUNT))

        return return_value

    def get_resolution(self) -> ImageResolution:
        return self._current_resolution

    @staticmethod
    def get_type_identifier() -> str:
        return "mock"

    # noinspection DuplicatedCode
    def set_parameters(self, parameters: list[KeyValueSimpleAny]) -> None:

        mismatched_keys: list[str] = list()

        key_value: KeyValueSimpleAbstract
        for key_value in parameters:
            if key_value.key == _CAMERA_FPS_KEY:
                if not isinstance(key_value, KeyValueSimpleFloat):
                    mismatched_keys.append(key_value.key)
                    continue
                self._current_frames_per_second = float(key_value.value)
            else:
                mismatched_keys.append(key_value.key)

        if len(mismatched_keys) > 0:
            raise MCTCameraRuntimeError(
                message=f"The following parameters could not be applied due to key mismatch: {str(mismatched_keys)}")

        if self.get_status() == Camera.Status.RUNNING:
            self._generate_image()

    def start(self) -> None:
        self.set_status(Camera.Status.RUNNING)
        self._generate_image()

    def stop(self) -> None:
        if self._image is not None:
            self._image = None
        self.set_status(Camera.Status.STOPPED)

    def update(self) -> None:
        now_timestamp_utc: datetime.datetime = datetime.datetime.now(tz=datetime.timezone.utc)
        if self._image is None:
            message: str = "Failed to grab frame."
            self.add_status_message(
                severity=SeverityLabel.ERROR,
                message=message)
            self.set_status(Camera.Status.FAILURE)
            raise MCTCameraRuntimeError(message=message)
        self._image_timestamp_utc = now_timestamp_utc

    def _generate_image(self):
        self._image = numpy.full(
            shape=(self._current_resolution.y_px, self._current_resolution.x_px, 3),
            fill_value=255,
            dtype=numpy.uint8)
