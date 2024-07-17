from .mct_component_address import MCTComponentAddress
from .connection import Connection
from src.common.api import \
    DequeueStatusMessagesResponse, \
    EmptyResponse, \
    ErrorResponse, \
    MCTRequestSeries, \
    MCTResponse, \
    MCTResponseSeries
from src.common.structures import \
    DetectorResolution, \
    ImageResolution, \
    IntrinsicParameters, \
    MarkerSnapshot
from src.detector.api import \
    CalibrationImageAddResponse, \
    CalibrationCalculateResponse, \
    CameraImageGetResponse, \
    CameraParametersGetResponse, \
    MarkerParametersGetResponse, \
    DetectorFrameGetResponse, \
    CalibrationImageGetResponse, \
    CalibrationResultGetResponse, \
    CalibrationResolutionListResponse, \
    CalibrationImageMetadataListResponse, \
    CalibrationResultMetadataListResponse, \
    DetectorStartRequest, \
    DetectorStopRequest
import datetime
import uuid


class DetectorConnection(Connection):

    # These are variables used directly by the MCTController for storing data
    request_id: uuid.UUID | None
    calibration_result_identifier: str | None
    calibrated_resolutions: list[DetectorResolution] | None
    current_resolution: ImageResolution | None
    current_intrinsic_parameters: IntrinsicParameters | None
    detected_marker_snapshots: list[MarkerSnapshot]
    rejected_marker_snapshots: list[MarkerSnapshot]
    marker_snapshot_timestamp: datetime.datetime

    def __init__(
        self,
        component_address: MCTComponentAddress
    ):
        super().__init__(component_address=component_address)
        self.request_id = None
        self.calibration_result_identifier = None
        self.calibrated_resolutions = None
        self.current_resolution = None
        self.current_intrinsic_parameters = None
        self.detected_marker_snapshots = list()
        self.rejected_marker_snapshots = list()
        self.marker_snapshot_timestamp = datetime.datetime.min

    def create_deinitialization_request_series(self) -> MCTRequestSeries:
        return MCTRequestSeries(series=[DetectorStopRequest()])

    def create_initialization_request_series(self) -> MCTRequestSeries:
        return MCTRequestSeries(series=[DetectorStartRequest()])

    def handle_deinitialization_response_series(
        self,
        response_series: MCTResponseSeries
    ) -> Connection.DeinitializationResult:
        response_count: int = len(response_series.series)
        if response_count != 1:
            self.enqueue_status_message(
                severity="warning",
                message=f"Expected exactly one response to deinitialization requests. Got {response_count}.")
        elif not isinstance(response_series.series[0], EmptyResponse):
            self.enqueue_status_message(
                severity="warning",
                message=f"The deinitialization response was not of the expected type EmptyResponse.")
        return Connection.DeinitializationResult.SUCCESS

    def handle_initialization_response_series(
        self,
        response_series: MCTResponseSeries
    ) -> Connection.InitializationResult:
        response_count: int = len(response_series.series)
        if response_count != 1:
            self.enqueue_status_message(
                severity="warning",
                message=f"Expected exactly one response to initialization requests. Got {response_count}.")
        elif not isinstance(response_series.series[0], EmptyResponse):
            self.enqueue_status_message(
                severity="warning",
                message=f"The initialization response was not of the expected type EmptyResponse.")
        return Connection.InitializationResult.SUCCESS

    def supported_response_types(self) -> list[type[MCTResponse]]:
        return [
            CalibrationCalculateResponse,
            CalibrationImageAddResponse,
            CalibrationImageGetResponse,
            CalibrationImageMetadataListResponse,
            CalibrationResolutionListResponse,
            CalibrationResultGetResponse,
            CalibrationResultMetadataListResponse,
            CameraImageGetResponse,
            CameraParametersGetResponse,
            DequeueStatusMessagesResponse,
            DetectorFrameGetResponse,
            EmptyResponse,
            ErrorResponse,
            MarkerParametersGetResponse]
