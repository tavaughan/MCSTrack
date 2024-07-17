from ..api import \
    DetectorFrameGetResponse, \
    MarkerParametersGetResponse
from src.common import \
    EmptyResponse, \
    ErrorResponse
from src.common.structures import MarkerStatus
import abc
import datetime


class AbstractMarkerInterface(abc.ABC):
    marker_status: MarkerStatus  # internal bookkeeping
    marker_timestamp_utc: datetime.datetime

    @abc.abstractmethod
    def set_detection_parameters(self, **kwargs) -> EmptyResponse | ErrorResponse:
        pass

    @abc.abstractmethod
    def get_detection_parameters(self, **_kwargs) -> MarkerParametersGetResponse | ErrorResponse:
        pass

    @abc.abstractmethod
    def get_marker_snapshots(self, **kwargs) -> DetectorFrameGetResponse:
        pass

    @abc.abstractmethod
    def internal_update_marker_corners(self, marker_status) -> None:
        pass
