from ..structures import \
    CameraConfiguration, \
    CameraStatus
from src.common import StatusMessageSource
from src.common.structures import \
    CaptureFormat, \
    KeyValueSimpleAny, \
    KeyValueMetaAbstract, \
    SeverityLabel
import abc
import base64
import cv2
import datetime
import numpy


class AbstractCamera(abc.ABC):
    """
    Functions may raise MCTDetectorRuntimeError
    """

    _configuration: CameraConfiguration
    _status: CameraStatus
    _status_message_source: StatusMessageSource

    def __init__(
        self,
        configuration: CameraConfiguration,
        status_message_source: StatusMessageSource
    ):
        self._configuration = configuration
        self._status_message_source = status_message_source

    def __del__(self):
        pass

    def add_status_message(
        self,
        severity: SeverityLabel,
        message: str
    ) -> None:
        self._status_message_source.enqueue_status_message(severity=severity, message=message)

    def get_encoded_image(
        self,
        image_format: CaptureFormat
    ) -> str:
        encoded_frame: bool
        encoded_image_rgb_single_row: numpy.array
        encoded, encoded_image_rgb_single_row = cv2.imencode(image_format, self.get_image())
        encoded_image_rgb_bytes: bytes = encoded_image_rgb_single_row.tobytes()
        # noinspection PyTypeChecker
        encoded_image_rgb_base64: str = base64.b64encode(encoded_image_rgb_bytes)
        return encoded_image_rgb_base64

    def get_status(self) -> CameraStatus:
        return self._status

    def set_status(self, status: CameraStatus) -> None:
        self._status = status

    @abc.abstractmethod
    def get_changed_timestamp(self) -> datetime.datetime: ...

    @abc.abstractmethod
    def get_image(self) -> numpy.ndarray: ...

    @abc.abstractmethod
    def get_parameters(self) -> list[KeyValueMetaAbstract]: ...

    @staticmethod
    @abc.abstractmethod
    def get_type_identifier() -> str: ...  # Unique string associated with this type

    @abc.abstractmethod
    def set_parameters(self, parameters: list[KeyValueSimpleAny]) -> None: ...

    @abc.abstractmethod
    def start(self) -> None: ...

    @abc.abstractmethod
    def stop(self) -> None: ...

    @abc.abstractmethod
    def update(self) -> None: ...
