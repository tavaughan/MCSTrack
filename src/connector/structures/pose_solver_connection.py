from .component_address import ComponentAddress
from .connection import Connection
from src.common.api import \
    DequeueStatusMessagesResponse, \
    EmptyResponse, \
    ErrorResponse, \
    MCastRequestSeries, \
    MCastResponse, \
    MCastResponseSeries
from src.common.structures import Pose
from src.pose_solver.api import \
    AddTargetMarkerResponse, \
    GetPosesResponse, \
    StartPoseSolverRequest, \
    StopPoseSolverRequest
import datetime
import uuid


class PoseSolverConnection(Connection):

    # These are variables used directly by the Connector for storing data
    request_id: uuid.UUID | None
    detector_poses: list[Pose]
    target_poses: list[Pose]
    detector_timestamps: dict[str, datetime.datetime]  # access by detector_label
    poses_timestamp: datetime.datetime

    def __init__(
        self,
        component_address: ComponentAddress
    ):
        super().__init__(component_address=component_address)
        self.request_id = None
        self.detector_poses = list()
        self.target_poses = list()
        self.detector_timestamps = dict()
        self.poses_timestamp = datetime.datetime.min

    def create_deinitialization_request_series(self) -> MCastRequestSeries:
        return MCastRequestSeries(series=[StopPoseSolverRequest()])

    def create_initialization_request_series(self) -> MCastRequestSeries:
        return MCastRequestSeries(series=[StartPoseSolverRequest()])

    def handle_deinitialization_response_series(
        self,
        response_series: MCastResponseSeries
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
        response_series: MCastResponseSeries
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

    def supported_response_types(self) -> list[type[MCastResponse]]:
        return [
            AddTargetMarkerResponse,
            DequeueStatusMessagesResponse,
            EmptyResponse,
            ErrorResponse,
            GetPosesResponse]
