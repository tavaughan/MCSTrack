from .exceptions import ResponseSeriesNotExpected
from .structures import \
    ComponentAddress, \
    ConnectionReport, \
    LiveConnection, \
    LiveDetectorConnection, \
    LivePoseSolverConnection
from src.common import \
    DequeueStatusMessagesRequest, \
    DequeueStatusMessagesResponse, \
    EmptyResponse, \
    ErrorResponse, \
    MCastComponent, \
    MCastRequest, \
    MCastRequestSeries, \
    MCastResponse, \
    MCastResponseSeries, \
    mcast_websocket_send_recv, \
    StatusMessageSource
from src.common.structures import \
    COMPONENT_ROLE_LABEL_DETECTOR, \
    COMPONENT_ROLE_LABEL_POSE_SOLVER, \
    DetectorFrame, \
    DetectorResolution, \
    ImageResolution, \
    IntrinsicParameters, \
    PoseSolverFrame
from src.calibrator.api import \
    AddCalibrationImageResponse, \
    CalibrateResponse, \
    GetCalibrationImageResponse, \
    GetCalibrationResultRequest, \
    GetCalibrationResultResponse, \
    ListCalibrationDetectorResolutionsRequest, \
    ListCalibrationDetectorResolutionsResponse, \
    ListCalibrationResultMetadataRequest, \
    ListCalibrationImageMetadataResponse, \
    ListCalibrationResultMetadataResponse
from src.detector.api import \
    GetCaptureDeviceResponse, \
    GetCapturePropertiesRequest, \
    GetCapturePropertiesResponse, \
    GetCaptureImageResponse, \
    GetDetectionParametersResponse, \
    GetMarkerSnapshotsRequest, \
    GetMarkerSnapshotsResponse, \
    StartCaptureRequest, \
    StopCaptureRequest
from src.pose_solver.api import \
    AddTargetMarkerResponse, \
    AddMarkerCornersRequest, \
    GetPosesRequest, \
    GetPosesResponse, \
    SetIntrinsicParametersRequest, \
    StartPoseSolverRequest, \
    StopPoseSolverRequest
import datetime
from enum import IntEnum, StrEnum
import logging
from typing import Callable, Final, Optional, Tuple, TypeVar
import uuid
from websockets import \
    connect

logger = logging.getLogger(__name__)
LiveConnectionType = TypeVar('LiveConnectionType', bound=LiveConnection)


SUPPORTED_RESPONSE_TYPES: list[type[MCastResponse]] = [
    AddCalibrationImageResponse,
    AddTargetMarkerResponse,
    CalibrateResponse,
    DequeueStatusMessagesResponse,
    EmptyResponse,
    ErrorResponse,
    GetCalibrationImageResponse,
    GetCalibrationResultResponse,
    GetCaptureDeviceResponse,
    GetCaptureImageResponse,
    GetCapturePropertiesResponse,
    GetDetectionParametersResponse,
    GetMarkerSnapshotsResponse,
    GetPosesResponse,
    ListCalibrationDetectorResolutionsResponse,
    ListCalibrationImageMetadataResponse,
    ListCalibrationResultMetadataResponse]


class Connector(MCastComponent):

    class Status(IntEnum):
        STOPPED: Final[int] = 0
        STARTING: Final[int] = 1
        RUNNING: Final[int] = 2
        STOPPING: Final[int] = 3

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

    class Connection:
        def __init__(
            self,
            component_address: ComponentAddress,
            live_connection: LiveConnection
        ):
            self.static = component_address
            self.live_connection = live_connection

        static: ComponentAddress
        live_connection: LiveConnection

    _serial_identifier: str

    _status_message_source: StatusMessageSource
    _status: Status
    _startup_mode: StartupMode
    _startup_state: StartupState

    _connections: dict[str, Connection]
    _pending_request_ids: list[uuid.UUID]

    _request_series_by_label: dict[str, list[Tuple[MCastRequestSeries, uuid.UUID]]]

    # None indicates that no response has been received yet.
    _response_series_by_id: dict[uuid.UUID, MCastResponseSeries | None]

    def __init__(
        self,
        serial_identifier: str,
        send_status_messages_to_logger: bool = False
    ):
        super().__init__(
            status_source_label=serial_identifier,
            send_status_messages_to_logger=send_status_messages_to_logger)

        self._serial_identifier = serial_identifier

        self.status_message_source = StatusMessageSource(
            source_label="connector",
            send_to_logger=True)
        self._status = Connector.Status.STOPPED
        self._startup_mode = Connector.StartupMode.DETECTING_AND_SOLVING  # Will be overwritten on startup
        self._startup_state = Connector.StartupState.INITIAL

        self._connections = dict()
        self._pending_request_ids = list()

        self._request_series_by_label = dict()
        self._response_series_by_id = dict()

    def add_connection(
        self,
        component_address: ComponentAddress
    ) -> None:
        label = component_address.label
        if label in self._connections:
            raise RuntimeError(f"Connection associated with {label} already exists.")
        connection_dynamic: LiveConnection
        if component_address.role == COMPONENT_ROLE_LABEL_DETECTOR:
            connection_dynamic = LiveDetectorConnection()
        elif component_address.role == COMPONENT_ROLE_LABEL_POSE_SOLVER:
            connection_dynamic = LivePoseSolverConnection()
        else:
            raise ValueError(f"Unrecognized component role {component_address.role}.")
        self._connections[label] = Connector.Connection(
            component_address=component_address,
            live_connection=connection_dynamic)

    def begin_connecting(self, label: str) -> None:
        if label not in self._connections:
            message: str = f"label {label} is not in list. Returning."
            self.add_status_message(severity="error", message=message)
            return
        if self._connections[label].live_connection.status == "connected":
            message: str = f"label {label} is already connected. Returning."
            self.add_status_message(severity="warning", message=message)
            return
        self._connections[label].live_connection.status = "connecting"
        self._connections[label].live_connection.attempt_count = 0

    def begin_disconnecting(self, label: str) -> None:
        if label not in self._connections:
            message: str = f"label {label} is not in list. Returning."
            self.add_status_message(severity="error", message=message)
            return
        self._connections[label].live_connection.status = "disconnecting"
        self._connections[label].live_connection.attempt_count = 0
        self._connections[label].live_connection.socket = None

    def contains_connection_label(self, label: str) -> bool:
        return label in self._connections

    def get_connection_reports(self) -> list[ConnectionReport]:
        return_value: list[ConnectionReport] = list()
        for connection in self._connections.values():
            return_value.append(ConnectionReport(
                label=connection.static.label,
                role=connection.static.role,
                ip_address=str(connection.static.ip_address),
                port=int(connection.static.port),
                status=connection.live_connection.status))
        return return_value

    def get_connected_detector_labels(self) -> list[str]:
        return self.get_connected_role_labels(role=COMPONENT_ROLE_LABEL_DETECTOR)

    def get_connected_pose_solver_labels(self) -> list[str]:
        return self.get_connected_role_labels(role=COMPONENT_ROLE_LABEL_POSE_SOLVER)

    def get_connected_role_labels(self, role: str) -> list[str]:
        return_value: list[str] = list()
        for connection in self._connections.values():
            if connection.static.role == role and connection.live_connection.status == "connected":
                return_value.append(connection.static.label)
        return return_value

    def get_live_connection(
        self,
        connection_label: str,
        live_connection_type: type[LiveConnectionType]
    ) -> LiveConnectionType | None:
        if connection_label not in self._connections:
            return None
        live_connection: LiveConnectionType = self._connections[connection_label].live_connection
        if not isinstance(live_connection, live_connection_type):
            return None
        return live_connection

    def get_live_detector_intrinsics(
        self,
        detector_label: str
    ) -> IntrinsicParameters | None:
        """
        returns None if the detector does not exist, or if it has not been started.
        """
        live_detector_connection: LiveDetectorConnection = self.get_live_connection(
            connection_label=detector_label,
            live_connection_type=LiveDetectorConnection)
        if live_detector_connection is None:
            return None
        return live_detector_connection.current_intrinsic_parameters

    def get_live_detector_frame(
        self,
        detector_label: str
    ) -> DetectorFrame | None:
        """
        returns None if the detector does not exist, or has not been started, or if it has not yet gotten frames.
        """
        live_detector_connection: LiveDetectorConnection = self.get_live_connection(
            connection_label=detector_label,
            live_connection_type=LiveDetectorConnection)
        if live_detector_connection is None:
            return None
        return DetectorFrame(
            detected_marker_snapshots=live_detector_connection.detected_marker_snapshots,
            rejected_marker_snapshots=live_detector_connection.rejected_marker_snapshots,
            timestamp_utc_iso8601=live_detector_connection.marker_snapshot_timestamp.isoformat())

    def get_live_pose_solver_frame(
        self,
        pose_solver_label: str
    ) -> PoseSolverFrame | None:
        """
        returns None if the pose solver does not exist, or has not been started, or if it has not yet gotten frames.
        """
        live_pose_solver_connection: LivePoseSolverConnection = self.get_live_connection(
            connection_label=pose_solver_label,
            live_connection_type=LivePoseSolverConnection)
        if live_pose_solver_connection is None:
            return None
        return PoseSolverFrame(
            detector_poses=live_pose_solver_connection.detector_poses,
            target_poses=live_pose_solver_connection.target_poses,
            timestamp_utc_iso8601=live_pose_solver_connection.poses_timestamp.isoformat())

    def get_status(self):
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
        response: GetCapturePropertiesResponse,
        detector_label: str
    ) -> None:
        live_detector_connection: LiveDetectorConnection = self.get_live_connection(
            connection_label=detector_label,
            live_connection_type=LiveDetectorConnection)
        if live_detector_connection is None:
            self.status_message_source.enqueue_status_message(
                severity="error",
                message=f"Failed to find LiveDetectorConnection with label {detector_label}.")
            return
        live_detector_connection.current_resolution = ImageResolution(
            x_px=response.resolution_x_px,
            y_px=response.resolution_y_px)

    def handle_response_get_calibration_result(
        self,
        response: GetCalibrationResultResponse
    ) -> None:
        detector_label: str = response.intrinsic_calibration.detector_serial_identifier
        live_detector_connection: LiveDetectorConnection = self.get_live_connection(
            connection_label=detector_label,
            live_connection_type=LiveDetectorConnection)
        if live_detector_connection is None:
            self.status_message_source.enqueue_status_message(
                severity="error",
                message=f"Failed to find LiveDetectorConnection with label {detector_label}.")
            return
        live_detector_connection.current_intrinsic_parameters = response.intrinsic_calibration.calibrated_values

    def handle_response_get_marker_snapshots(
        self,
        response: GetMarkerSnapshotsResponse,
        detector_label: str
    ):
        live_detector_connection: LiveDetectorConnection = self.get_live_connection(
            connection_label=detector_label,
            live_connection_type=LiveDetectorConnection)
        if live_detector_connection is None:
            self.status_message_source.enqueue_status_message(
                severity="error",
                message=f"Failed to find LiveDetectorConnection with label {detector_label}.")
            return
        live_detector_connection.detected_marker_snapshots = response.detected_marker_snapshots
        live_detector_connection.rejected_marker_snapshots = response.rejected_marker_snapshots
        live_detector_connection.marker_snapshot_timestamp = \
            datetime.datetime.utcnow()  # TODO: This should come from the detector

    def handle_response_get_poses(
        self,
        response: GetPosesResponse,
        pose_solver_label: str
    ) -> None:
        live_pose_solver_connection: LivePoseSolverConnection = self.get_live_connection(
            connection_label=pose_solver_label,
            live_connection_type=LivePoseSolverConnection)
        if live_pose_solver_connection is None:
            self.status_message_source.enqueue_status_message(
                severity="error",
                message=f"Failed to find LivePoseSolverConnection with label {pose_solver_label}.")
            return
        live_pose_solver_connection.detector_poses = response.detector_poses
        live_pose_solver_connection.target_poses = response.target_poses
        live_pose_solver_connection.poses_timestamp = \
            datetime.datetime.utcnow()  # TODO: This should come from the pose solver

    def handle_response_list_calibration_detector_resolutions(
        self,
        response: ListCalibrationDetectorResolutionsResponse,
        detector_label: str
    ) -> None:
        live_detector_connection: LiveDetectorConnection = self.get_live_connection(
            connection_label=detector_label,
            live_connection_type=LiveDetectorConnection)
        if live_detector_connection is None:
            self.status_message_source.enqueue_status_message(
                severity="error",
                message=f"Failed to find LiveDetectorConnection with label {detector_label}.")
            return
        live_detector_connection.calibrated_resolutions = response.detector_resolutions

    def handle_response_list_calibration_result_metadata(
        self,
        response: ListCalibrationResultMetadataResponse,
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
        live_detector_connection: LiveDetectorConnection = self.get_live_connection(
            connection_label=detector_label,
            live_connection_type=LiveDetectorConnection)
        if live_detector_connection is None:
            self.status_message_source.enqueue_status_message(
                severity="error",
                message=f"Failed to find LiveDetectorConnection with label {detector_label}.")
            return
        live_detector_connection.calibration_result_identifier = newest_result_id

    def handle_response_unknown(
        self,
        response: MCastResponse
    ):
        self.status_message_source.enqueue_status_message(
            severity="error",
            message=f"Received unexpected response: {str(type(response))}")

    def handle_response_series(
        self,
        response_series: MCastResponseSeries,
        task_description: Optional[str] = None,
        expected_response_count: Optional[int] = None
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
        response: MCastResponse
        for response in response_series.series:
            if isinstance(response, AddTargetMarkerResponse):
                success = True  # we don't currently do anything with this response in this interface
            elif isinstance(response, GetCalibrationResultResponse):
                self.handle_response_get_calibration_result(response=response)
                success = True
            elif isinstance(response, GetCapturePropertiesResponse):
                self.handle_response_get_capture_properties(
                    response=response,
                    detector_label=response_series.responder)
                success = True
            elif isinstance(response, GetMarkerSnapshotsResponse):
                self.handle_response_get_marker_snapshots(
                    response=response,
                    detector_label=response_series.responder)
            elif isinstance(response, GetPosesResponse):
                self.handle_response_get_poses(
                    response=response,
                    pose_solver_label=response_series.responder)
                success = True
            elif isinstance(response, ListCalibrationDetectorResolutionsResponse):
                self.handle_response_list_calibration_detector_resolutions(
                    response=response,
                    detector_label=response_series.responder)
                success = True
            elif isinstance(response, ListCalibrationResultMetadataResponse):
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

    def ignore_request_and_response(
        self,
        client_identifier: str,
        request_id: uuid.UUID
    ):
        if client_identifier in self._request_series_by_label:
            for stored_request_index in range(len(self._request_series_by_label[client_identifier])):
                stored_request_id: uuid.UUID = \
                    self._request_series_by_label[client_identifier][stored_request_index][1]
                if stored_request_id == request_id:
                    self._request_series_by_label[client_identifier].pop(stored_request_index)
                    break
        if request_id in self._response_series_by_id:
            del self._response_series_by_id[request_id]

    def is_running(self):
        return self._status == Connector.Status.RUNNING

    def is_in_transition(self):
        return self._status == Connector.Status.STARTING or self._status == Connector.Status.STOPPING

    def on_active_request_ids_processed(self) -> None:
        if self._status == Connector.Status.STARTING:
            if self._startup_state == Connector.StartupState.STARTING_CAPTURE:
                self.status_message_source.enqueue_status_message(
                    severity="debug",
                    message="STARTING_CAPTURE complete")
                detector_labels: list[str] = self.get_connected_detector_labels()
                for detector_label in detector_labels:
                    request_series: MCastRequestSeries = MCastRequestSeries(
                        series=[
                            ListCalibrationDetectorResolutionsRequest(),
                            GetCapturePropertiesRequest()])
                    self._pending_request_ids.append(self.request_series_push(
                        connection_label=detector_label,
                        request_series=request_series))
                self._startup_state = Connector.StartupState.GET_RESOLUTIONS
            elif self._startup_state == Connector.StartupState.GET_RESOLUTIONS:
                self.status_message_source.enqueue_status_message(
                    severity="debug",
                    message="GET_RESOLUTIONS complete")
                for detector_label in self.get_connected_detector_labels():
                    live_detector_connection: LiveDetectorConnection = self.get_live_connection(
                        connection_label=detector_label,
                        live_connection_type=LiveDetectorConnection)
                    if live_detector_connection is None:
                        self.status_message_source.enqueue_status_message(
                            severity="error",
                            message=f"Failed to find LiveDetectorConnection with label {detector_label}.")
                        continue
                    requests: list[MCastRequest] = list()
                    target_resolution: DetectorResolution = DetectorResolution(
                        detector_serial_identifier=detector_label,
                        image_resolution=live_detector_connection.current_resolution)
                    found_target_resolution: bool = False
                    for detector_resolution in live_detector_connection.calibrated_resolutions:
                        if detector_resolution == target_resolution:
                            requests.append(
                                ListCalibrationResultMetadataRequest(
                                    detector_serial_identifier=detector_label,
                                    image_resolution=target_resolution.image_resolution))
                            found_target_resolution = True
                    if not found_target_resolution:
                        self.status_message_source.enqueue_status_message(
                            severity="error",
                            message=f"No calibration available for detector {detector_label} "
                                    f"at resolution {str(live_detector_connection.current_resolution)}. "
                                    "No intrinsics will be set.")
                    request_series: MCastRequestSeries = MCastRequestSeries(series=requests)
                    self._pending_request_ids.append(self.request_series_push(
                        connection_label=detector_label,
                        request_series=request_series))
                self._startup_state = Connector.StartupState.LIST_INTRINSICS
            elif self._startup_state == Connector.StartupState.LIST_INTRINSICS:
                self.status_message_source.enqueue_status_message(
                    severity="debug",
                    message="LIST_INTRINSICS complete")
                for detector_label in self.get_connected_detector_labels():
                    live_detector_connection: LiveDetectorConnection = self.get_live_connection(
                        connection_label=detector_label,
                        live_connection_type=LiveDetectorConnection)
                    if live_detector_connection is None:
                        self.status_message_source.enqueue_status_message(
                            severity="error",
                            message=f"Failed to find LiveDetectorConnection with label {detector_label}.")
                        continue
                    request_series: MCastRequestSeries = MCastRequestSeries(
                        series=[
                            GetCalibrationResultRequest(
                                result_identifier=live_detector_connection.calibration_result_identifier)])
                    self._pending_request_ids.append(self.request_series_push(
                        connection_label=detector_label,
                        request_series=request_series))
                self._startup_state = Connector.StartupState.GET_INTRINSICS
            elif self._startup_state == Connector.StartupState.GET_INTRINSICS:
                self.status_message_source.enqueue_status_message(
                    severity="debug",
                    message="GET_INTRINSICS complete")
                if self._startup_mode == Connector.StartupMode.DETECTING_ONLY:
                    self._startup_state = Connector.StartupState.INITIAL
                    self._status = Connector.Status.RUNNING  # We're done
                else:
                    pose_solver_labels: list[str] = self.get_connected_pose_solver_labels()
                    for pose_solver_label in pose_solver_labels:
                        requests: list[MCastRequest] = list()
                        for detector_label in self.get_connected_detector_labels():
                            live_detector_connection: LiveDetectorConnection = self.get_live_connection(
                                connection_label=detector_label,
                                live_connection_type=LiveDetectorConnection)
                            if live_detector_connection is None:
                                self.status_message_source.enqueue_status_message(
                                    severity="error",
                                    message=f"Failed to find LiveDetectorConnection with label {detector_label}.")
                                continue
                            requests.append(SetIntrinsicParametersRequest(
                                detector_label=detector_label,
                                intrinsic_parameters=live_detector_connection.current_intrinsic_parameters))
                        requests.append(StartPoseSolverRequest())
                        request_series: MCastRequestSeries = MCastRequestSeries(series=requests)
                        self._pending_request_ids.append(self.request_series_push(
                            connection_label=pose_solver_label,
                            request_series=request_series))
                    self._startup_state = Connector.StartupState.SET_INTRINSICS
            elif self._startup_state == Connector.StartupState.SET_INTRINSICS:
                self.status_message_source.enqueue_status_message(
                    severity="debug",
                    message="SET_INTRINSICS complete")
                self._startup_state = Connector.StartupState.INITIAL
                self._status = Connector.Status.RUNNING
        elif self._status == Connector.Status.STOPPING:
            self._status = Connector.Status.STOPPED

    def start_tracking(
        self,
        mode: str = StartupMode.DETECTING_AND_SOLVING
    ) -> None:
        if mode not in Connector.StartupMode:
            raise ValueError(f"Unexpected mode \"{mode}\".")
        self._startup_mode = Connector.StartupMode(mode)

        detector_labels: list[str] = self.get_connected_detector_labels()
        for detector_label in detector_labels:
            request_series: MCastRequestSeries = MCastRequestSeries(
                series=[
                    StartCaptureRequest(),
                    ListCalibrationDetectorResolutionsRequest()])
            self._pending_request_ids.append(self.request_series_push(
                connection_label=detector_label,
                request_series=request_series))
        self._startup_state = Connector.StartupState.STARTING_CAPTURE
        self._status = Connector.Status.STARTING

    def stop_tracking(self) -> None:

        # TODO: Just ignore these existing requests, no need to wait for them or react to responses

        for detector_label in self.get_connected_detector_labels():
            live_detector_connection: LiveDetectorConnection = self.get_live_connection(
                connection_label=detector_label,
                live_connection_type=LiveDetectorConnection)
            if live_detector_connection is None:
                self.status_message_source.enqueue_status_message(
                    severity="error",
                    message=f"Failed to find LiveDetectorConnection with label {detector_label}.")
                continue
            if live_detector_connection.request_id is not None:
                self._pending_request_ids.append(live_detector_connection.request_id)
            request_series: MCastRequestSeries = MCastRequestSeries(
                series=[StopCaptureRequest()])
            self._pending_request_ids.append(self.request_series_push(
                connection_label=detector_label,
                request_series=request_series))

        for pose_solver_label in self.get_connected_pose_solver_labels():
            live_pose_solver_connection: LivePoseSolverConnection = self.get_live_connection(
                connection_label=pose_solver_label,
                live_connection_type=LivePoseSolverConnection)
            if live_pose_solver_connection is None:
                self.status_message_source.enqueue_status_message(
                    severity="error",
                    message=f"Failed to find LivePoseSolverConnection with label {pose_solver_label}.")
                continue
            if live_pose_solver_connection.request_id is not None:
                self._pending_request_ids.append(live_pose_solver_connection.request_id)
            request_series: MCastRequestSeries = MCastRequestSeries(series=[StopPoseSolverRequest()])
            self._pending_request_ids.append(self.request_series_push(
                connection_label=pose_solver_label,
                request_series=request_series))

        self._status = Connector.Status.STOPPING

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
        request_series: MCastRequestSeries
    ) -> uuid.UUID:
        if connection_label not in self._request_series_by_label:
            self._request_series_by_label[connection_label] = list()
        request_series_id: uuid.UUID = uuid.uuid4()
        self._request_series_by_label[connection_label].append((request_series, request_series_id))
        self._response_series_by_id[request_series_id] = None
        return request_series_id

    def response_series_pop(
        self,
        request_series_id: uuid.UUID
    ) -> MCastResponseSeries | None:
        """
        Only "pop" if there is a response (not None).
        Return value is the response series itself (or None)
        """
        if request_series_id not in self._response_series_by_id:
            raise ResponseSeriesNotExpected()

        if self._response_series_by_id[request_series_id] is None:
            return None

        response_series: MCastResponseSeries = self._response_series_by_id[request_series_id]
        self._response_series_by_id.pop(request_series_id)
        return response_series

    def supported_request_types(self) -> dict[type[MCastRequest], Callable[[dict], MCastResponse]]:
        return super().supported_request_types()

    # Right now this function doesn't update on its own - must be called externally
    def update_loop(self) -> None:
        if self.is_running():
            for detector_label in self.get_connected_detector_labels():
                live_detector_connection: LiveDetectorConnection = self.get_live_connection(
                    connection_label=detector_label,
                    live_connection_type=LiveDetectorConnection)
                if live_detector_connection is None:
                    self.status_message_source.enqueue_status_message(
                        severity="error",
                        message=f"Failed to find LiveDetectorConnection with label {detector_label}.")
                    continue
                if live_detector_connection.request_id is not None:
                    _, live_detector_connection.request_id = self.update_request(
                        request_id=live_detector_connection.request_id)
                if live_detector_connection.request_id is None:
                    live_detector_connection.request_id = self.request_series_push(
                        connection_label=detector_label,
                        request_series=MCastRequestSeries(series=[GetMarkerSnapshotsRequest()]))
            for pose_solver_label in self.get_connected_pose_solver_labels():
                live_pose_solver_connection: LivePoseSolverConnection = self.get_live_connection(
                    connection_label=pose_solver_label,
                    live_connection_type=LivePoseSolverConnection)
                if live_pose_solver_connection is None:
                    self.status_message_source.enqueue_status_message(
                        severity="error",
                        message=f"Failed to find LivePoseSolverConnection with label {pose_solver_label}.")
                    continue
                if live_pose_solver_connection.request_id is not None:
                    _, live_pose_solver_connection.request_id = self.update_request(
                        request_id=live_pose_solver_connection.request_id)
                if live_pose_solver_connection.request_id is None:
                    solver_request_list: list[MCastRequest] = list()
                    detector_labels: list[str] = self.get_connected_detector_labels()
                    for detector_label in detector_labels:
                        current_detector_frame: DetectorFrame = self.get_live_detector_frame(
                            detector_label=detector_label)
                        current_detector_frame_timestamp: datetime.datetime = current_detector_frame.timestamp_utc()
                        current_is_new: bool = False
                        if detector_label in live_pose_solver_connection.detector_timestamps:
                            old_detector_frame_timestamp = \
                                live_pose_solver_connection.detector_timestamps[detector_label]
                            if current_detector_frame_timestamp > old_detector_frame_timestamp:
                                current_is_new = True
                        else:
                            current_is_new = True
                        if current_is_new:
                            live_pose_solver_connection.detector_timestamps[detector_label] = \
                                current_detector_frame_timestamp
                            marker_request: AddMarkerCornersRequest = AddMarkerCornersRequest(
                                detected_marker_snapshots=current_detector_frame.detected_marker_snapshots,
                                rejected_marker_snapshots=current_detector_frame.rejected_marker_snapshots,
                                detector_label=detector_label,
                                detector_timestamp_utc_iso8601=current_detector_frame.timestamp_utc_iso8601)
                            solver_request_list.append(marker_request)
                    solver_request_list.append(GetPosesRequest())
                    request_series: MCastRequestSeries = MCastRequestSeries(series=solver_request_list)
                    live_pose_solver_connection.request_id = self.request_series_push(
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
                self.on_active_request_ids_processed()

    def update_request(
        self,
        request_id: uuid.UUID,
        task_description: Optional[str] = None,
        expected_response_count: Optional[int] = None
    ) -> (bool, uuid.UUID | None):
        """
        Returns a tuple of:
        - success at handling the response (False if no response has been received)
        - value that request_id shall take for subsequent iterations (None means a response series has been received)
        """

        response_series: MCastResponseSeries | None = self.response_series_pop(
            request_series_id=request_id)
        if response_series is None:
            return False, request_id  # try again next loop

        success: bool = self.handle_response_series(
            response_series=response_series,
            task_description=task_description,
            expected_response_count=expected_response_count)
        return success, None  # We've handled the request, request_id can be set to None

    async def do_update_frame_for_connection(
        self,
        connection: Connection
    ) -> None:
        if connection.live_connection.status == "disconnected" or connection.live_connection.status == "aborted":
            return

        if connection.live_connection.status == "disconnecting":
            if connection.live_connection.socket is not None:
                await connection.live_connection.socket.close()
                connection.live_connection.socket = None
            connection.live_connection.socket = None
            connection.live_connection.status = "disconnected"

        if connection.live_connection.status == "connecting":
            now_utc = datetime.datetime.utcnow()
            if now_utc >= connection.live_connection.next_attempt_timestamp_utc:
                connection.live_connection.attempt_count += 1
                uri: str = f"ws://{connection.static.ip_address}:{connection.static.port}/websocket"
                try:
                    connection.live_connection.socket = await connect(
                        uri=uri,
                        ping_timeout=None,
                        open_timeout=None,
                        close_timeout=None,
                        max_size=2**48)  # Default max_size may have trouble with large uncompressed images
                except ConnectionError as e:
                    if connection.live_connection.attempt_count >= LiveConnection.ATTEMPT_COUNT_MAXIMUM:
                        message = \
                            f"Failed to connect to {uri} with error: {str(e)}. "\
                            f"Connection is being aborted after {connection.live_connection.attempt_count} attempts."
                        self.add_status_message(severity="error", message=message)
                        connection.live_connection.status = "aborted"
                    else:
                        message: str = \
                            f"Failed to connect to {uri} with error: {str(e)}. "\
                            f"Will retry in {LiveConnection.ATTEMPT_TIME_GAP_SECONDS} seconds."
                        self.add_status_message(severity="warning", message=message)
                        connection.live_connection.next_attempt_timestamp_utc = now_utc + datetime.timedelta(
                            seconds=LiveConnection.ATTEMPT_TIME_GAP_SECONDS)
                    return
                message = f"Connected to {uri}."
                self.add_status_message(severity="info", message=message)
                connection.live_connection.status = "connected"
                connection.live_connection.attempt_count = 0

        if connection.live_connection.status == "connected":

            # TODO: Is this correct or even useful...?
            # if connection.dynamic.socket.closed:
            #     message = \
            #         f"Socket associated with {connection.static.label} appears to have been closed. "\
            #         f"Will attempt to reconnect."
            #     self.add_status_message(severity="warning", message=message)
            #     connection.dynamic.socket = None
            #     connection.dynamic.status = "connecting"
            #     connection.dynamic.attempt_count = 0
            #     return

            def response_series_converter(
                response_series_dict: dict
            ) -> MCastResponseSeries:
                series_list: list[MCastResponse] = self.parse_dynamic_series_list(
                    parsable_series_dict=response_series_dict,
                    supported_types=SUPPORTED_RESPONSE_TYPES)
                return MCastResponseSeries(series=series_list)

            # Handle manually-defined irregular tasks
            if connection.static.label in self._request_series_by_label:
                pairs: list[Tuple[MCastRequestSeries, uuid.UUID]] = \
                    self._request_series_by_label[connection.static.label]
                for pair in pairs:
                    response_series: MCastResponseSeries = \
                        await mcast_websocket_send_recv(
                            websocket=connection.live_connection.socket,
                            request_series=pair[0],
                            response_series_type=MCastResponseSeries,
                            response_series_converter=response_series_converter)
                    # TODO: This next line's logic may belong in the response_series_converter
                    response_series.responder = connection.static.label
                    self._response_series_by_id[pair[1]] = response_series
                self._request_series_by_label.pop(connection.static.label)

            # Regular every-frame stuff
            request_series: list[MCastRequest] = list()
            request_series.append(DequeueStatusMessagesRequest())

            response_series: MCastResponseSeries = await mcast_websocket_send_recv(
                websocket=connection.live_connection.socket,
                request_series=MCastRequestSeries(series=request_series),
                response_series_type=MCastResponseSeries,
                response_series_converter=response_series_converter)
            for response in response_series.series:
                if isinstance(response, DequeueStatusMessagesResponse):
                    for status_message in response.status_messages:
                        status_message_dict = status_message.dict()
                        status_message_dict["source_label"] = connection.static.label
                        self.add_status_message(**status_message_dict)

    async def do_update_frames_for_connections(
        self
    ) -> None:
        connections = list(self._connections.values())
        for connection in connections:
            await self.do_update_frame_for_connection(connection=connection)
