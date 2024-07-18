from src.common.structures import \
    DetectorFrame, \
    DetectionParameters, \
    MarkerStatus
import abc
import datetime


class AbstractMarkerInterface(abc.ABC):
    marker_status: MarkerStatus  # internal bookkeeping
    marker_timestamp_utc: datetime.datetime

    @abc.abstractmethod
    def set_detection_parameters(self, parameters: DetectionParameters) -> None:
        """
        May raise MCTDetectorRuntimeError
        """
        pass

    @abc.abstractmethod
    def get_detection_parameters(self) -> DetectionParameters:
        """
        May raise MCTDetectorRuntimeError
        """
        pass

    @abc.abstractmethod
    def get_marker_snapshots(self) -> DetectorFrame:
        """
        May raise MCTDetectorRuntimeError
        """
        pass

    @abc.abstractmethod
    def internal_update_marker_corners(self, marker_status) -> None:
        """
        May raise MCTDetectorRuntimeError
        """
        pass
