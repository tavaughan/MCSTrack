from .exceptions import ResponseSeriesNotExpected
from .structures import \
    MCTComponentAddress, \
    ConnectionReport, \
    Connection, \
    DetectorConnection, \
    PoseSolverConnection
from src.common import \
    EmptyResponse, \
    ErrorResponse, \
    MCTComponent, \
    MCTRequest, \
    MCTRequestSeries, \
    MCTResponse, \
    MCTResponseSeries, \
    StatusMessageSource
from src.common.structures import \
    ComponentRoleLabel, \
    COMPONENT_ROLE_LABEL_DETECTOR, \
    COMPONENT_ROLE_LABEL_POSE_SOLVER, \
    DetectorFrame, \
    ImageResolution, \
    IntrinsicParameters, \
    PoseSolverFrame
from src.detector.api import \
    CalibrationResultGetRequest, \
    CalibrationResultGetResponse, \
    CameraParametersGetRequest, \
    CameraParametersGetResponse, \
    DetectorFrameGetRequest, \
    DetectorFrameGetResponse, \
    CalibrationResolutionListRequest, \
    CalibrationResolutionListResponse, \
    CalibrationResultMetadataListRequest, \
    CalibrationResultMetadataListResponse
from src.pose_solver.api import \
    AddTargetMarkerResponse, \
    AddMarkerCornersRequest, \
    GetPosesRequest, \
    GetPosesResponse, \
    SetIntrinsicParametersRequest
import datetime
from enum import IntEnum, StrEnum
import logging
from typing import Callable, Final, get_args, TypeVar
import uuid

logger = logging.getLogger(__name__)
ConnectionType = TypeVar('ConnectionType', bound=Connection)


class MCTController(MCTComponent):

    class Status(StrEnum):
        STOPPED: Final[int] = "Idle"
        STARTING: Final[int] = "Starting"
        RUNNING: Final[int] = "Running"
        STOPPING: Final[int] = "Stopping"

    class StartupMode(StrEnum):
        DETECTING_ONLY: Final[str] = "detecting_only"
        DETECTING_AND_SOLVING: Final[str] = "detecting_and_solving"

    class StartupState(IntEnum):
        INITIAL: Final[int] = 0
        CONNECTING: Final[int] = 1
        STARTING_CAPTURE: Final[int] = 2
        GET_RESOLUTIONS: Final[int] = 3
        LIST_INTRINSICS: Final[int] = 4  # This and next phase can be combined with some API modification
        GET_INTRINSICS: Final[int] = 5
        SET_INTRINSICS: Final[int] = 6

    _status_message_source: StatusMessageSource
    _status: Status
    _startup_mode: StartupMode
    _startup_state: StartupState

    _connections: dict[str, Connection]
    _pending_request_ids: list[uuid.UUID]

    def __init__(
        self,
        serial_identifier: str,
        send_status_messages_to_logger: bool = False
    ):
        super().__init__(
            status_source_label=serial_identifier,
            send_status_messages_to_logger=send_status_messages_to_logger)

        self.status_message_source = StatusMessageSource(
            source_label="controller",
            send_to_logger=True)
        self._status = MCTController.Status.STOPPED
        self._startup_mode = MCTController.StartupMode.DETECTING_AND_SOLVING  # Will be overwritten on startup
        self._startup_state = MCTController.StartupState.INITIAL

        self._connections = dict()
        self._pending_request_ids = list()

    def add_connection(
        self,
        component_address: MCTComponentAddress
    ) -> None:
        label = component_address.label
        if label in self._connections:
            raise RuntimeError(f"Connection associated with {label} already exists.")
        if component_address.role == COMPONENT_ROLE_LABEL_DETECTOR:
            self._connections[label] = DetectorConnection(component_address=component_address)
        elif component_address.role == COMPONENT_ROLE_LABEL_POSE_SOLVER:
            self._connections[label] = PoseSolverConnection(component_address=component_address)
        else:
            raise ValueError(f"Unrecognized component role {component_address.role}.")

    def _advance_startup_state(self) -> None:
        if len(self._pending_request_ids) <= 0 and self._startup_state == MCTController.StartupState.STARTING_CAPTURE:
            self.status_message_source.enqueue_status_message(
                severity="debug",
                message="STARTING_CAPTURE complete")
            detector_labels: list[str] = self.get_active_detector_labels()
            for detector_label in detector_labels:
                request_series: MCTRequestSeries = MCTRequestSeries(
                    series=[
                        CalibrationResolutionListRequest(),
                        CameraParametersGetRequest()])
                self._pending_request_ids.append(self.request_series_push(
                    connection_label=detector_label,
                    request_series=request_series))
            self._startup_state = MCTController.StartupState.GET_RESOLUTIONS
        if len(self._pending_request_ids) <= 0 and self._startup_state == MCTController.StartupState.GET_RESOLUTIONS:
            self.status_message_source.enqueue_status_message(
                severity="debug",
                message="GET_RESOLUTIONS complete")
            for detector_label in self.get_active_detector_labels():
                detector_connection: DetectorConnection = self._get_connection(
                    connection_label=detector_label,
                    connection_type=DetectorConnection)
                if detector_connection is None:
                    self.status_message_source.enqueue_status_message(
                        severity="error",
                        message=f"Failed to find DetectorConnection with label {detector_label}.")
                    continue
                if detector_connection.current_resolution is None:
                    self.status_message_source.enqueue_status_message(
                        severity="error",
                        message=f"DetectorConnection with label {detector_label} doesn't have a resolution.")
                    continue
                requests: list[MCTRequest] = list()
                found_target_resolution: bool = False
                for detector_resolution in detector_connection.calibrated_resolutions:
                    if detector_resolution == detector_connection.current_resolution:
                        requests.append(
                            CalibrationResultMetadataListRequest(
                                detector_serial_identifier=detector_label,
                                image_resolution=detector_connection.current_resolution))
                        found_target_resolution = True
                if not found_target_resolution:
                    self.status_message_source.enqueue_status_message(
                        severity="error",
                        message=f"No calibration available for detector {detector_label} "
                                f"at resolution {str(detector_connection.current_resolution)}. "
                                "No intrinsics will be set.")
                request_series: MCTRequestSeries = MCTRequestSeries(series=requests)
                self._pending_request_ids.append(self.request_series_push(
                    connection_label=detector_label,
                    request_series=request_series))
            self._startup_state = MCTController.StartupState.LIST_INTRINSICS
        if len(self._pending_request_ids) <= 0 and self._startup_state == MCTController.StartupState.LIST_INTRINSICS:
            self.status_message_source.enqueue_status_message(
                severity="debug",
                message="LIST_INTRINSICS complete")
            for detector_label in self.get_active_detector_labels():
                detector_connection: DetectorConnection = self._get_connection(
                    connection_label=detector_label,
                    connection_type=DetectorConnection)
                if detector_connection is None:
                    self.status_message_source.enqueue_status_message(
                        severity="error",
                        message=f"Failed to find DetectorConnection with label {detector_label}.")
                    continue
                if detector_connection.calibration_result_identifier is not None:
                    request_series: MCTRequestSeries = MCTRequestSeries(
                        series=[
                            CalibrationResultGetRequest(
                                result_identifier=detector_connection.calibration_result_identifier)])
                    self._pending_request_ids.append(self.request_series_push(
                        connection_label=detector_label,
                        request_series=request_series))
            self._startup_state = MCTController.StartupState.GET_INTRINSICS
        if len(self._pending_request_ids) <= 0 and self._startup_state == MCTController.StartupState.GET_INTRINSICS:
            self.status_message_source.enqueue_status_message(
                severity="debug",
                message="GET_INTRINSICS complete")
            if self._startup_mode == MCTController.StartupMode.DETECTING_ONLY:
                self._startup_state = MCTController.StartupState.INITIAL
                self._status = MCTController.Status.RUNNING  # We're done
            else:
                pose_solver_labels: list[str] = self.get_active_pose_solver_labels()
                for pose_solver_label in pose_solver_labels:
                    requests: list[MCTRequest] = list()
                    for detector_label in self.get_active_detector_labels():
                        detector_connection: DetectorConnection = self._get_connection(
                            connection_label=detector_label,
                            connection_type=DetectorConnection)
                        if detector_connection is None:
                            self.status_message_source.enqueue_status_message(
                                severity="error",
                                message=f"Failed to find DetectorConnection with label {detector_label}.")
                            continue
                        if detector_connection.current_intrinsic_parameters is not None:
                            requests.append(SetIntrinsicParametersRequest(
                                detector_label=detector_label,
                                intrinsic_parameters=detector_connection.current_intrinsic_parameters))
                    request_series: MCTRequestSeries = MCTRequestSeries(series=requests)
                    self._pending_request_ids.append(self.request_series_push(
                        connection_label=pose_solver_label,
                        request_series=request_series))
                self._startup_state = MCTController.StartupState.SET_INTRINSICS
        if len(self._pending_request_ids) <= 0 and self._startup_state == MCTController.StartupState.SET_INTRINSICS:
            self.status_message_source.enqueue_status_message(
                severity="debug",
                message="SET_INTRINSICS complete")
            self._startup_state = MCTController.StartupState.INITIAL
            self._status = MCTController.Status.RUNNING

    def contains_connection_label(self, label: str) -> bool:
        return label in self._connections

    def get_active_detector_labels(self) -> list[str]:
        """
        See get_component_labels.
        """
        return self.get_component_labels(role=COMPONENT_ROLE_LABEL_DETECTOR, active=True)

    def get_active_pose_solver_labels(self) -> list[str]:
        """
        See get_component_labels.
        """
        return self.get_component_labels(role=COMPONENT_ROLE_LABEL_POSE_SOLVER, active=True)

    def get_component_labels(
        self,
        role: str | None = None,
        active: bool | None = None
    ) -> list[str]:
        """
        Return the list of all labels corresponding to components of the given `role`, and given `active` state.
        None provided to `role` or `active` is treated as a wildcard (i.e. not filtered on that criteria).
        """
        if role is not None:
            valid_roles: list[str] = list(get_args(ComponentRoleLabel))
            if role not in valid_roles:
                raise ValueError(f"role must be among the valid values {str(valid_roles)}")
        return_value: list[str] = list()
        for connection_label, connection in self._connections.items():
            if role is not None and connection.get_role() != role:
                continue
            if active is not None and connection.is_active() != active:
                continue
            return_value.append(connection_label)
        return return_value

    def get_connection_reports(self) -> list[ConnectionReport]:
        return_value: list[ConnectionReport] = list()
        for connection in self._connections.values():
            return_value.append(connection.get_report())
        return return_value

    def _get_connection(
        self,
        connection_label: str,
        connection_type: type[ConnectionType]
    ) -> ConnectionType | None:
        if connection_label not in self._connections:
            return None
        connection: ConnectionType = self._connections[connection_label]
        if not isinstance(connection, connection_type):
            return None
        return connection

    def get_live_detector_intrinsics(
        self,
        detector_label: str
    ) -> IntrinsicParameters | None:
        """
        returns None if the detector does not exist, or if it has not been started.
        """
        detector_connection: DetectorConnection = self._get_connection(
            connection_label=detector_label,
            connection_type=DetectorConnection)
        if detector_connection is None:
            return None
        return detector_connection.current_intrinsic_parameters

    def get_live_detector_frame(
        self,
        detector_label: str
    ) -> DetectorFrame | None:
        """
        returns None if the detector does not exist, or has not been started, or if it has not yet gotten frames.
        """
        detector_connection: DetectorConnection = self._get_connection(
            connection_label=detector_label,
            connection_type=DetectorConnection)
        if detector_connection is None:
            return None
        return DetectorFrame(
            detected_marker_snapshots=detector_connection.detected_marker_snapshots,
            rejected_marker_snapshots=detector_connection.rejected_marker_snapshots,
            timestamp_utc_iso8601=detector_connection.marker_snapshot_timestamp.isoformat())

    def get_live_pose_solver_frame(
        self,
        pose_solver_label: str
    ) -> PoseSolverFrame | None:
        """
        returns None if the pose solver does not exist, or has not been started, or if it has not yet gotten frames.
        """
        pose_solver_connection: PoseSolverConnection = self._get_connection(
            connection_label=pose_solver_label,
            connection_type=PoseSolverConnection)
        if pose_solver_connection is None:
            return None
        return PoseSolverFrame(
            detector_poses=pose_solver_connection.detector_poses,
            target_poses=pose_solver_connection.target_poses,
            timestamp_utc_iso8601=pose_solver_connection.poses_timestamp.isoformat())

    def get_status(self) -> Status:
        return self._status

    def handle_error_response(
        self,
        response: ErrorResponse
    ):
        self.status_message_source.enqueue_status_message(
            severity="error",
            message=f"Received error: {response.message}")

    def handle_response_get_capture_properties(
        self,
        response: CameraParametersGetResponse,
        detector_label: str
    ) -> None:
        detector_connection: DetectorConnection = self._get_connection(
            connection_label=detector_label,
            connection_type=DetectorConnection)
        if detector_connection is None:
            self.status_message_source.enqueue_status_message(
                severity="error",
                message=f"Failed to find DetectorConnection with label {detector_label}.")
            return
        # TODO: The code below will be rendered obsolete once the communications are refactored such that
        #       the Detector provides us with the current resolution without being explicitly asked for it.
        for key_value in response.parameters:
            if key_value.key == "Resolution":  # Hard-coded key that will be removed in a future revision
                detector_connection.current_resolution = ImageResolution.from_str(key_value.value)
                return
        if detector_connection.current_resolution is None:
            self.status_message_source.enqueue_status_message(
                severity="error",
                message=f"Failed to find resolution of DetectorConnection with label {detector_label}.")
            return

    def handle_response_get_calibration_result(
        self,
        response: CalibrationResultGetResponse
    ) -> None:
        detector_label: str = response.intrinsic_calibration.detector_serial_identifier
        detector_connection: DetectorConnection = self._get_connection(
            connection_label=detector_label,
            connection_type=DetectorConnection)
        if detector_connection is None:
            self.status_message_source.enqueue_status_message(
                severity="error",
                message=f"Failed to find DetectorConnection with label {detector_label}.")
            return
        detector_connection.current_intrinsic_parameters = response.intrinsic_calibration.calibrated_values

    def handle_response_get_marker_snapshots(
        self,
        response: DetectorFrameGetResponse,
        detector_label: str
    ):
        detector_connection: DetectorConnection = self._get_connection(
            connection_label=detector_label,
            connection_type=DetectorConnection)
        if detector_connection is None:
            self.status_message_source.enqueue_status_message(
                severity="error",
                message=f"Failed to find DetectorConnection with label {detector_label}.")
            return
        detector_connection.detected_marker_snapshots = response.detected_marker_snapshots
        detector_connection.rejected_marker_snapshots = response.rejected_marker_snapshots
        detector_connection.marker_snapshot_timestamp = \
            datetime.datetime.utcnow()  # TODO: This should come from the detector

    def handle_response_get_poses(
        self,
        response: GetPosesResponse,
        pose_solver_label: str
    ) -> None:
        pose_solver_connection: PoseSolverConnection = self._get_connection(
            connection_label=pose_solver_label,
            connection_type=PoseSolverConnection)
        if pose_solver_connection is None:
            self.status_message_source.enqueue_status_message(
                severity="error",
                message=f"Failed to find PoseSolverConnection with label {pose_solver_label}.")
            return
        pose_solver_connection.detector_poses = response.detector_poses
        pose_solver_connection.target_poses = response.target_poses
        pose_solver_connection.poses_timestamp = \
            datetime.datetime.utcnow()  # TODO: This should come from the pose solver

    def handle_response_list_calibration_detector_resolutions(
        self,
        response: CalibrationResolutionListResponse,
        detector_label: str
    ) -> None:
        detector_connection: DetectorConnection = self._get_connection(
            connection_label=detector_label,
            connection_type=DetectorConnection)
        if detector_connection is None:
            self.status_message_source.enqueue_status_message(
                severity="error",
                message=f"Failed to find DetectorConnection with label {detector_label}.")
            return
        detector_connection.calibrated_resolutions = response.resolutions

    def handle_response_list_calibration_result_metadata(
        self,
        response: CalibrationResultMetadataListResponse,
        detector_label: str
    ) -> None:
        if len(response.metadata_list) <= 0:
            self.status_message_source.enqueue_status_message(
                severity="error",
                message=f"No calibration was available for detector {detector_label}. No intrinsics will be set.")
            return
        newest_result_id: str = response.metadata_list[0].identifier  # placeholder, maybe
        newest_timestamp: datetime.datetime = datetime.datetime.min
        for result_metadata in response.metadata_list:
            timestamp: datetime.datetime = datetime.datetime.fromisoformat(result_metadata.timestamp_utc)
            if timestamp > newest_timestamp:
                newest_result_id = result_metadata.identifier
        detector_connection: DetectorConnection = self._get_connection(
            connection_label=detector_label,
            connection_type=DetectorConnection)
        if detector_connection is None:
            self.status_message_source.enqueue_status_message(
                severity="error",
                message=f"Failed to find DetectorConnection with label {detector_label}.")
            return
        detector_connection.calibration_result_identifier = newest_result_id

    def handle_response_unknown(
        self,
        response: MCTResponse
    ):
        self.status_message_source.enqueue_status_message(
            severity="error",
            message=f"Received unexpected response: {str(type(response))}")

    def handle_response_series(
        self,
        response_series: MCTResponseSeries,
        task_description: str | None = None,
        expected_response_count: int | None = None
    ) -> bool:
        if expected_response_count is not None:
            response_count: int = len(response_series.series)
            task_text: str = str()
            if task_description is not None:
                task_text = f" during {task_description}"
            if response_count < expected_response_count:
                self.status_message_source.enqueue_status_message(
                    severity="warning",
                    message=f"Received a response series{task_text}, "
                            f"but it contained fewer responses ({response_count}) "
                            f"than expected ({expected_response_count}).")
            elif response_count > expected_response_count:
                self.status_message_source.enqueue_status_message(
                    severity="warning",
                    message=f"Received a response series{task_text}, "
                            f"but it contained more responses ({response_count}) "
                            f"than expected ({expected_response_count}).")

        success: bool = True
        response: MCTResponse
        for response in response_series.series:
            if isinstance(response, AddTargetMarkerResponse):
                success = True  # we don't currently do anything with this response in this interface
            elif isinstance(response, CalibrationResultGetResponse):
                self.handle_response_get_calibration_result(response=response)
                success = True
            elif isinstance(response, CameraParametersGetResponse):
                self.handle_response_get_capture_properties(
                    response=response,
                    detector_label=response_series.responder)
                success = True
            elif isinstance(response, DetectorFrameGetResponse):
                self.handle_response_get_marker_snapshots(
                    response=response,
                    detector_label=response_series.responder)
            elif isinstance(response, GetPosesResponse):
                self.handle_response_get_poses(
                    response=response,
                    pose_solver_label=response_series.responder)
                success = True
            elif isinstance(response, CalibrationResolutionListResponse):
                self.handle_response_list_calibration_detector_resolutions(
                    response=response,
                    detector_label=response_series.responder)
                success = True
            elif isinstance(response, CalibrationResultMetadataListResponse):
                self.handle_response_list_calibration_result_metadata(
                    response=response,
                    detector_label=response_series.responder)
                success = True
            elif isinstance(response, ErrorResponse):
                self.handle_error_response(response=response)
                success = False
            elif not isinstance(response, EmptyResponse):
                self.handle_response_unknown(response=response)
                success = False
        return success

    def is_idle(self):
        return self._status == MCTController.Status.STOPPED

    def is_running(self):
        return self._status == MCTController.Status.RUNNING

    def is_transitioning(self):
        return self._status == MCTController.Status.STARTING or self._status == MCTController.Status.STOPPING

    def remove_connection(
        self,
        label: str
    ):
        if label not in self._connections:
            raise RuntimeError(f"Failed to find connection associated with {label}.")
        self._connections.pop(label)

    def request_series_push(
        self,
        connection_label: str,
        request_series: MCTRequestSeries
    ) -> uuid.UUID:
        if connection_label not in self._connections:
            raise RuntimeError(f"Failed to find connection with label {connection_label}.")
        elif not self._connections[connection_label].is_active():
            raise RuntimeError(f"Connection with label {connection_label} is not active.")
        return self._connections[connection_label].enqueue_request_series(
            request_series=request_series)

    def response_series_pop(
        self,
        request_series_id: uuid.UUID
    ) -> tuple[uuid.UUID | None, MCTResponseSeries | None]:
        """
        Only "pop" if there is a response (not None).
        Return value is a tuple whose elements comprise:
          - UUID of the request if no response has been received, or None
          - MCTResponseSeries if a response has been received, or None
        The dual return values allow easier reassignment of completed request ID's in calling code
        """
        for connection in self._connections.values():
            response_result: Connection.PopResponseSeriesResult = connection.pop_response_series_if_responded(
                request_series_id=request_series_id)
            if response_result.status == Connection.PopResponseSeriesResult.Status.UNTRACKED:
                continue
            elif response_result.status == Connection.PopResponseSeriesResult.Status.RESPONDED:
                return None, response_result.response_series
            else:  # queued, in progress
                return request_series_id, None  # Connection is tracking desired request series, but awaiting response
        # Cannot be found
        raise ResponseSeriesNotExpected()

    def start_up(
        self,
        mode: str = StartupMode.DETECTING_AND_SOLVING
    ) -> None:
        if mode not in MCTController.StartupMode:
            raise ValueError(f"Unexpected mode \"{mode}\".")
        self._startup_mode = MCTController.StartupMode(mode)

        if self._status != MCTController.Status.STOPPED:
            raise RuntimeError("Cannot start up if controller isn't first stopped.")
        for connection in self._connections.values():
            if mode == MCTController.StartupMode.DETECTING_ONLY and \
               connection.get_role() == COMPONENT_ROLE_LABEL_POSE_SOLVER:
                continue
            connection.start_up()

        self._startup_state = MCTController.StartupState.STARTING_CAPTURE
        self._status = MCTController.Status.STARTING

    def shut_down(self) -> None:
        if self._status != MCTController.Status.RUNNING:
            raise RuntimeError("Cannot shut down if controller isn't first running.")
        for connection in self._connections.values():
            if connection.is_start_up_finished():
                connection.shut_down()

        self._status = MCTController.Status.STOPPING

    def supported_request_types(self) -> dict[type[MCTRequest], Callable[[dict], MCTResponse]]:
        return super().supported_request_types()

    # Right now this function doesn't update on its own - must be called externally and regularly
    async def update(
        self
    ) -> None:
        connections = list(self._connections.values())
        for connection in connections:
            await connection.update()

        if self._status == MCTController.Status.STARTING and \
           self._startup_state == MCTController.StartupState.STARTING_CAPTURE:
            startup_finished: bool = True
            for connection in connections:
                if self._startup_mode == MCTController.StartupMode.DETECTING_ONLY and \
                   connection.get_role() == COMPONENT_ROLE_LABEL_POSE_SOLVER:
                    continue
                if not connection.is_start_up_finished():
                    startup_finished = False
                    break
            if startup_finished:
                self._advance_startup_state()
        elif self._status == MCTController.Status.STOPPING:
            shutdown_finished: bool = True
            for connection in connections:
                if not connection.is_shut_down():
                    shutdown_finished = False
                    break
            if shutdown_finished:
                self._status = MCTController.Status.STOPPED

        if self.is_running():
            for detector_label in self.get_active_detector_labels():
                detector_connection: DetectorConnection = self._get_connection(
                    connection_label=detector_label,
                    connection_type=DetectorConnection)
                if detector_connection is None:
                    self.status_message_source.enqueue_status_message(
                        severity="error",
                        message=f"Failed to find DetectorConnection with label {detector_label}.")
                    continue
                if detector_connection.request_id is not None:
                    _, detector_connection.request_id = self.update_request(
                        request_id=detector_connection.request_id)
                if detector_connection.request_id is None:
                    detector_connection.request_id = self.request_series_push(
                        connection_label=detector_label,
                        request_series=MCTRequestSeries(series=[DetectorFrameGetRequest()]))
            for pose_solver_label in self.get_active_pose_solver_labels():
                pose_solver_connection: PoseSolverConnection = self._get_connection(
                    connection_label=pose_solver_label,
                    connection_type=PoseSolverConnection)
                if pose_solver_connection is None:
                    self.status_message_source.enqueue_status_message(
                        severity="error",
                        message=f"Failed to find PoseSolverConnection with label {pose_solver_label}.")
                    continue
                if pose_solver_connection.request_id is not None:
                    _, pose_solver_connection.request_id = self.update_request(
                        request_id=pose_solver_connection.request_id)
                if pose_solver_connection.request_id is None:
                    solver_request_list: list[MCTRequest] = list()
                    detector_labels: list[str] = self.get_active_detector_labels()
                    for detector_label in detector_labels:
                        current_detector_frame: DetectorFrame = self.get_live_detector_frame(
                            detector_label=detector_label)
                        current_detector_frame_timestamp: datetime.datetime = current_detector_frame.timestamp_utc()
                        current_is_new: bool = False
                        if detector_label in pose_solver_connection.detector_timestamps:
                            old_detector_frame_timestamp = \
                                pose_solver_connection.detector_timestamps[detector_label]
                            if current_detector_frame_timestamp > old_detector_frame_timestamp:
                                current_is_new = True
                        else:
                            current_is_new = True
                        if current_is_new:
                            pose_solver_connection.detector_timestamps[detector_label] = \
                                current_detector_frame_timestamp
                            marker_request: AddMarkerCornersRequest = AddMarkerCornersRequest(
                                detected_marker_snapshots=current_detector_frame.detected_marker_snapshots,
                                rejected_marker_snapshots=current_detector_frame.rejected_marker_snapshots,
                                detector_label=detector_label,
                                detector_timestamp_utc_iso8601=current_detector_frame.timestamp_utc_iso8601)
                            solver_request_list.append(marker_request)
                    solver_request_list.append(GetPosesRequest())
                    request_series: MCTRequestSeries = MCTRequestSeries(series=solver_request_list)
                    pose_solver_connection.request_id = self.request_series_push(
                        connection_label=pose_solver_label,
                        request_series=request_series)

        if len(self._pending_request_ids) > 0:
            completed_request_ids: list[uuid.UUID] = list()
            for request_id in self._pending_request_ids:
                _, remaining_request_id = self.update_request(request_id=request_id)
                if remaining_request_id is None:
                    completed_request_ids.append(request_id)
            for request_id in completed_request_ids:
                self._pending_request_ids.remove(request_id)
            if len(self._pending_request_ids) == 0:
                self._advance_startup_state()

    def update_request(
        self,
        request_id: uuid.UUID,
        task_description: str | None = None,
        expected_response_count: int | None = None
    ) -> (bool, uuid.UUID | None):
        """
        Returns a tuple of:
        - success at handling the response (False if no response has been received)
        - value that request_id shall take for subsequent iterations (None means a response series has been received)
        """

        response_series: MCTResponseSeries | None
        _, response_series = self.response_series_pop(request_series_id=request_id)
        if response_series is None:
            return False, request_id  # try again next loop

        success: bool = self.handle_response_series(
            response_series=response_series,
            task_description=task_description,
            expected_response_count=expected_response_count)
        return success, None  # We've handled the request, request_id can be set to None
