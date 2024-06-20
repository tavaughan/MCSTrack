from .live_connection import LiveConnection
from src.common.structures import \
    DetectorResolution, \
    ImageResolution, \
    IntrinsicParameters, \
    MarkerSnapshot
import datetime
import uuid


class LiveDetectorConnection(LiveConnection):
    request_id: uuid.UUID | None

    calibration_result_identifier: str | None
    calibrated_resolutions: list[DetectorResolution] | None
    current_resolution: ImageResolution | None
    current_intrinsic_parameters: IntrinsicParameters | None

    detected_marker_snapshots: list[MarkerSnapshot]
    rejected_marker_snapshots: list[MarkerSnapshot]
    marker_snapshot_timestamp: datetime.datetime

    def reset(self):
        super().reset()
        self.request_id = None
        self.calibration_result_identifier = None
        self.calibrated_resolutions = None
        self.current_resolution = None
        self.current_intrinsic_parameters = None
        self.detected_marker_snapshots = list()
        self.rejected_marker_snapshots = list()
        self.marker_snapshot_timestamp = datetime.datetime.min
