from ..structures import \
    MarkerConfiguration, \
    MarkerStatus
from src.common import StatusMessageSource
from src.common.structures import \
    DetectionParameters, \
    MarkerSnapshot, \
    SeverityLabel
import abc
import datetime
import numpy


class AbstractMarker(abc.ABC):
    """
    Functions may raise MCTDetectorRuntimeError
    """

    _configuration: MarkerConfiguration
    _status: MarkerStatus
    _status_message_source: StatusMessageSource

    def __init__(
        self,
        configuration: MarkerConfiguration,
        status_message_source: StatusMessageSource
    ):
        self._configuration = configuration
        self._status_message_source = status_message_source

    def add_status_message(
        self,
        severity: SeverityLabel,
        message: str
    ) -> None:
        self._status_message_source.enqueue_status_message(severity=severity, message=message)

    def get_status(self) -> MarkerStatus:
        return self._status

    def set_status(self, status: MarkerStatus) -> None:
        self._status = status

    @abc.abstractmethod
    def get_changed_timestamp(self) -> datetime.datetime: ...

    @abc.abstractmethod
    def get_markers_detected(self) -> list[MarkerSnapshot]: ...

    @abc.abstractmethod
    def get_markers_rejected(self) -> list[MarkerSnapshot]: ...

    @abc.abstractmethod
    def get_parameters(self) -> DetectionParameters: ...

    @staticmethod
    @abc.abstractmethod
    def get_type_identifier() -> str: ...  # Unique string associated with this type

    @abc.abstractmethod
    def set_parameters(self, parameters: DetectionParameters) -> None: ...

    @abc.abstractmethod
    def update(self, image: numpy.ndarray) -> None: ...
