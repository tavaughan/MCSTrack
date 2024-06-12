from src.calibrator.api.get_calibration_result_request import GetCalibrationResultRequest
from src.calibrator.api.list_calibration_detector_resolutions_request import ListCalibrationDetectorResolutionsRequest
from src.calibrator.api.list_calibration_result_metadata_request import ListCalibrationResultMetadataRequest
from src.common.status_message_source import StatusMessageSource
from src.common.structures.detector_resolution import DetectorResolution
from src.common.structures.image_resolution import ImageResolution
from src.common.structures.intrinsic_parameters import IntrinsicParameters
from src.common.structures.marker_snapshot import MarkerSnapshot
from src.common.structures.pose import Pose
from src.detector.api.get_capture_properties_request import GetCapturePropertiesRequest
from src.detector.api.get_marker_snapshots_request import GetMarkerSnapshotsRequest
from src.detector.api.start_capture_request import StartCaptureRequest
from src.detector.api.stop_capture_request import StopCaptureRequest
from src.pose_solver.api.add_marker_corners_request import AddMarkerCornersRequest
from src.pose_solver.api.add_target_marker_request import AddTargetMarkerRequest
from src.pose_solver.api.get_poses_request import GetPosesRequest
from src.pose_solver.api.set_intrinsic_parameters_request import SetIntrinsicParametersRequest
from src.pose_solver.api.set_reference_marker_request import SetReferenceMarkerRequest
from src.pose_solver.api.start_pose_solver_request import StartPoseSolverRequest
from src.pose_solver.api.stop_pose_solver_request import StopPoseSolverRequest
from .exceptions import ResponseSeriesNotExpected
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
    mcast_websocket_send_recv
from src.common.structures import \
    COMPONENT_ROLE_LABEL_DETECTOR, \
    COMPONENT_ROLE_LABEL_POSE_SOLVER, \
    ComponentConnectionStatic
from src.connector.structures import \
    BaseComponentConnectionDynamic, \
    CalibratorComponentConnectionDynamic, \
    ConnectionTableRow, \
    DetectorComponentConnectionDynamic, \
    PoseSolverComponentConnectionDynamic
from src.calibrator.api import \
    AddCalibrationImageResponse, \
    CalibrateResponse, \
    GetCalibrationImageResponse, \
    GetCalibrationResultResponse, \
    ListCalibrationDetectorResolutionsResponse, \
    ListCalibrationImageMetadataResponse, \
    ListCalibrationResultMetadataResponse
from src.detector.api import \
    GetCaptureDeviceResponse, \
    GetCapturePropertiesResponse, \
    GetCaptureImageResponse, \
    GetDetectionParametersResponse, \
    GetMarkerSnapshotsResponse
from src.pose_solver.api import \
    AddTargetMarkerResponse, \
    GetPosesResponse
import datetime
import logging
from typing import Callable, Final, Optional, Tuple
import uuid
from websockets import \
    connect

logger = logging.getLogger(__name__)

# from src.gui.panels.pose_solver_panel import ACTIVE_PHASE_IDLE, ACTIVE_PHASE_STARTING_CAPTURE, ACTIVE_PHASE_STARTING_FINAL, ACTIVE_PHASE_STARTING_GET_INTRINSICS, ACTIVE_PHASE_STARTING_GET_RESOLUTIONS, ACTIVE_PHASE_STARTING_LIST_INTRINSICS, ACTIVE_PHASE_STOPPING
# Active means that something is happening that should prevent the user from interacting with the UI
ACTIVE_PHASE_IDLE: Final[int] = 0
ACTIVE_PHASE_STARTING_CAPTURE: Final[int] = 1
ACTIVE_PHASE_STARTING_GET_RESOLUTIONS: Final[int] = 2
ACTIVE_PHASE_STARTING_LIST_INTRINSICS: Final[int] = 3  # This and next phase to be combined with modified API
ACTIVE_PHASE_STARTING_GET_INTRINSICS: Final[int] = 4
ACTIVE_PHASE_STARTING_FINAL: Final[int] = 5
ACTIVE_PHASE_STOPPING: Final[int] = 6

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

    active_request_ids: list[uuid.UUID]
    status_message_source: StatusMessageSource

    class Connection:
        def __init__(
            self,
            static: ComponentConnectionStatic,
            dynamic: BaseComponentConnectionDynamic
        ):
            self.static = static
            self.dynamic = dynamic

        static: ComponentConnectionStatic
        dynamic: BaseComponentConnectionDynamic

    class PassiveDetectorRequest:
        request_id: uuid.UUID | None
        detected_marker_snapshots: list[MarkerSnapshot]
        rejected_marker_snapshots: list[MarkerSnapshot]
        marker_snapshot_timestamp: datetime.datetime
        def __init__(self):
            self.request_id = None
            self.detected_marker_snapshots = list()
            self.rejected_marker_snapshots = list()
            self.marker_snapshot_timestamp = datetime.datetime.min
    _passive_detecting_requests: dict[str, PassiveDetectorRequest]  # access by detector_label
    _passive_solving_request_id: uuid.UUID | None

    _serial_identifier: str
    _connections: dict[str, Connection]
    current_phase: int
    selected_pose_solver_label: str

    _request_series_by_label: dict[str, list[Tuple[MCastRequestSeries, uuid.UUID]]]

    # None indicates that no response has been received yet.
    _response_series_by_id: dict[uuid.UUID, MCastResponseSeries | None]

    # Variables assigned upon starting the pose solver
    _detector_calibration_labels: dict[str, str]
    _detector_resolutions: dict[str, ImageResolution]
    _calibrated_resolutions: list[DetectorResolution]
    _detector_intrinsics: dict[str, IntrinsicParameters]

    is_solving: bool
    tracked_target_poses: list[Pose]

    def __init__(
        self,
        serial_identifier: str,
        send_status_messages_to_logger: bool = False
    ):
        super().__init__(
            status_source_label=serial_identifier,
            send_status_messages_to_logger=send_status_messages_to_logger)
        
        self._passive_solving_request_id = None
        
        self.status_message_source = StatusMessageSource(
            source_label="connector",
            send_to_logger=True)
        self.active_request_ids = list()
        self._passive_detecting_requests = dict()

        self.is_solving = False
        self.tracked_target_poses = list()

        self._serial_identifier = serial_identifier
        self._connections = dict()
        self._request_series_by_label = dict()
        self._response_series_by_id = dict()
        self.current_phase = ACTIVE_PHASE_IDLE

        self._detector_resolutions = dict()
        self._calibrated_resolutions = list()
        self._detector_calibration_labels = dict()
        self._detector_intrinsics = dict()

    def add_connection(
        self,
        connection_static: ComponentConnectionStatic
    ) -> None:
        label = connection_static.label
        if label in self._connections:
            raise RuntimeError(f"Connection associated with {label} already exists.")
        connection_dynamic: BaseComponentConnectionDynamic
        if connection_static.role == COMPONENT_ROLE_LABEL_DETECTOR:
            connection_dynamic = DetectorComponentConnectionDynamic()
        elif connection_static.role == COMPONENT_ROLE_LABEL_POSE_SOLVER:
            connection_dynamic = PoseSolverComponentConnectionDynamic()
        else:
            raise NotImplementedError(f"Handling for {connection_static.role} not implemented")
        self._connections[label] = Connector.Connection(
            static=connection_static,
            dynamic=connection_dynamic)

    def begin_connecting(self, label: str) -> None:
        if label not in self._connections:
            message: str = f"label {label} is not in list. Returning."
            self.add_status_message(severity="error", message=message)
            return
        if self._connections[label].dynamic.status == "connected":
            message: str = f"label {label} is already connected. Returning."
            self.add_status_message(severity="warning", message=message)
            return
        self._connections[label].dynamic.status = "connecting"
        self._connections[label].dynamic.attempt_count = 0

    def begin_disconnecting(self, label: str) -> None:
        if label not in self._connections:
            message: str = f"label {label} is not in list. Returning."
            self.add_status_message(severity="error", message=message)
            return
        self._connections[label].dynamic.status = "disconnecting"
        self._connections[label].dynamic.attempt_count = 0
        self._connections[label].dynamic.socket = None

    def contains_connection_label(self, label: str) -> bool:
        return label in self._connections

    def get_connection_table_rows(self) -> list[ConnectionTableRow]:
        return_value: list[ConnectionTableRow] = list()
        for connection in self._connections.values():
            return_value.append(ConnectionTableRow(
                label=connection.static.label,
                role=connection.static.role,
                ip_address=str(connection.static.ip_address),
                port=int(connection.static.port),
                status=connection.dynamic.status))
        return return_value

    def get_connected_detector_labels(self) -> list[str]:
        return self.get_connected_role_labels(role=COMPONENT_ROLE_LABEL_DETECTOR)

    def get_connected_pose_solver_labels(self) -> list[str]:
        return self.get_connected_role_labels(role=COMPONENT_ROLE_LABEL_POSE_SOLVER)

    def get_connected_role_labels(self, role: str) -> list[str]:
        return_value: list[str] = list()
        for connection in self._connections.values():
            if connection.static.role == role and connection.dynamic.status == "connected":
                return_value.append(connection.static.label)
        return return_value
    
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
        self._detector_resolutions[detector_label] = ImageResolution(
            x_px=response.resolution_x_px,
            y_px=response.resolution_y_px)
        
    def handle_response_get_calibration_result(
        self,
        response: GetCalibrationResultResponse
    ) -> None:
        self._detector_intrinsics[response.intrinsic_calibration.detector_serial_identifier] = \
            response.intrinsic_calibration.calibrated_values

    def handle_response_get_marker_snapshots(
        self,
        response: GetMarkerSnapshotsResponse,
        detector_label: str
    ):
        if detector_label in self._passive_detecting_requests.keys():
            self._passive_detecting_requests[detector_label].detected_marker_snapshots = \
                response.detected_marker_snapshots
            self._passive_detecting_requests[detector_label].rejected_marker_snapshots = \
                response.rejected_marker_snapshots
            self._passive_detecting_requests[detector_label].marker_snapshot_timestamp = \
                datetime.datetime.utcnow()  # TODO: This should come from the detector

    def handle_response_get_poses(
        self,
        response: GetPosesResponse
    ) -> None:
        if not self.is_solving:
            return False
        self.tracked_target_poses.clear()
        for pose in response.target_poses:
            self.tracked_target_poses.append(pose)
        for pose in response.detector_poses:
            self.tracked_target_poses.append(pose)

    def handle_response_list_calibration_detector_resolutions(
        self,
        response: ListCalibrationDetectorResolutionsResponse
    ) -> None:
        self._calibrated_resolutions = response.detector_resolutions

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
        self._detector_calibration_labels[detector_label] = newest_result_id

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
        #return True

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
                self.handle_response_get_poses(response=response)
                success = True
            elif isinstance(response, ListCalibrationDetectorResolutionsResponse):
                self.handle_response_list_calibration_detector_resolutions(response=response)
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
                self.handle_unknown_response(response=response)
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

    def on_active_request_ids_processed(self) -> None:
        if self.current_phase == ACTIVE_PHASE_STARTING_CAPTURE:
            self.status_message_source.enqueue_status_message(
                severity="debug",
                message="ACTIVE_PHASE_STARTING_CAPTURE complete")
            calibrator_labels: list[str] = self.get_connected_detector_labels()
            request_series: MCastRequestSeries = MCastRequestSeries(
                series=[ListCalibrationDetectorResolutionsRequest()])
            self.active_request_ids.append(self.request_series_push(
                connection_label=calibrator_labels[0],
                request_series=request_series))
            detector_labels: list[str] = self.get_connected_detector_labels()
            for detector_label in detector_labels:
                request_series: MCastRequestSeries = MCastRequestSeries(
                    series=[GetCapturePropertiesRequest()])
                self.active_request_ids.append(self.request_series_push(
                    connection_label=detector_label,
                    request_series=request_series))
            self.current_phase = ACTIVE_PHASE_STARTING_GET_RESOLUTIONS
        elif self.current_phase == ACTIVE_PHASE_STARTING_GET_RESOLUTIONS:
            self.status_message_source.enqueue_status_message(
                severity="debug",
                message="ACTIVE_PHASE_STARTING_GET_RESOLUTIONS complete")
            requests: list[MCastRequest] = list()
            for detector_label, image_resolution in self._detector_resolutions.items():
                detector_resolution: DetectorResolution = DetectorResolution(
                    detector_serial_identifier=detector_label,
                    image_resolution=image_resolution)
                if detector_resolution in self._calibrated_resolutions:
                    requests.append(
                        ListCalibrationResultMetadataRequest(
                            detector_serial_identifier=detector_resolution.detector_serial_identifier,
                            image_resolution=image_resolution))
                else:
                    self.status_message_source.enqueue_status_message(
                        severity="error",
                        message=f"No calibration available for detector {detector_label} "
                                f"at resolution {str(image_resolution)}. No intrinsics will be set.")
            calibrator_labels: list[str] = self.get_connected_detector_labels()
            request_series: MCastRequestSeries = MCastRequestSeries(series=requests)
            self.active_request_ids.append(self.request_series_push(
                connection_label=calibrator_labels[0],
                request_series=request_series))
            self.current_phase = ACTIVE_PHASE_STARTING_LIST_INTRINSICS
        elif self.current_phase == ACTIVE_PHASE_STARTING_LIST_INTRINSICS:
            self.status_message_source.enqueue_status_message(
                severity="debug",
                message="ACTIVE_PHASE_STARTING_LIST_INTRINSICS complete")
            requests: list[MCastRequest] = list()
            for detector_label, result_identifier in self._detector_calibration_labels.items():
                requests.append(GetCalibrationResultRequest(result_identifier=result_identifier))
            calibrator_labels: list[str] = self.get_connected_detector_labels()
            request_series: MCastRequestSeries = MCastRequestSeries(series=requests)
            self.active_request_ids.append(self.request_series_push(
                connection_label=calibrator_labels[0],
                request_series=request_series))
            self.current_phase = ACTIVE_PHASE_STARTING_GET_INTRINSICS
        elif self.current_phase == ACTIVE_PHASE_STARTING_GET_INTRINSICS:
            self.status_message_source.enqueue_status_message(
                severity="debug",
                message="ACTIVE_PHASE_STARTING_GET_INTRINSICS complete")
            requests: list[MCastRequest] = list()
            for detector_label, intrinsic_parameters in self._detector_intrinsics.items():
                requests.append(SetIntrinsicParametersRequest(
                    detector_label=detector_label,
                    intrinsic_parameters=intrinsic_parameters))
            requests.append(StartPoseSolverRequest())
            request_series: MCastRequestSeries = MCastRequestSeries(series=requests)
            self.active_request_ids.append(self.request_series_push(
                connection_label=self.selected_pose_solver_label,
                request_series=request_series))
            self.current_phase = ACTIVE_PHASE_STARTING_FINAL
        elif self.current_phase == ACTIVE_PHASE_STARTING_FINAL:
            self.status_message_source.enqueue_status_message(
                severity="debug",
                message="ACTIVE_PHASE_STARTING_FINAL complete")
            for detector_label in self._detector_intrinsics.keys():
                self._passive_detecting_requests[detector_label] = self.PassiveDetectorRequest()
            self.is_solving = True
            self.current_phase = ACTIVE_PHASE_IDLE
        elif self.current_phase == ACTIVE_PHASE_STOPPING:
            self.current_phase = ACTIVE_PHASE_IDLE

    def start_tracking(self) -> None:
        calibrator_labels: list[str] = self.get_connected_detector_labels()
        if len(calibrator_labels) > 1:
            self.status_message_source.enqueue_status_message(
                severity="warning",
                message="Multiple calibrators are connected. "
                        "The first is being arbitrarily chosen for getting intrinsic parameters.")
        elif len(calibrator_labels) <= 0:
            self.status_message_source.enqueue_status_message(
                severity="error",
                message="No calibrators were found. Aborting tracking.")
            return
        self._detector_resolutions.clear()
        self._calibrated_resolutions.clear()
        self._detector_calibration_labels.clear()
        self._detector_intrinsics.clear()
        request_series: MCastRequestSeries = MCastRequestSeries(
            series=[ListCalibrationDetectorResolutionsRequest()])
        self.active_request_ids.append(self.request_series_push(
            connection_label=calibrator_labels[0],
            request_series=request_series))
        detector_labels: list[str] = self.get_connected_detector_labels()
        for detector_label in detector_labels:
            request_series: MCastRequestSeries = MCastRequestSeries(
                series=[StartCaptureRequest()])
            self.active_request_ids.append(self.request_series_push(
                connection_label=detector_label,
                request_series=request_series))
        self.current_phase = ACTIVE_PHASE_STARTING_CAPTURE

    def stop_tracking(self) -> None:
        detector_labels: list[str] = self.get_connected_detector_labels()
        for detector_label in detector_labels:
            request_series: MCastRequestSeries = MCastRequestSeries(
                series=[StopCaptureRequest()])
            self.active_request_ids.append(self.request_series_push(
                connection_label=detector_label,
                request_series=request_series))
        request_series: MCastRequestSeries = MCastRequestSeries(series=[StopPoseSolverRequest()])
        self.active_request_ids.append(self.request_series_push(
            connection_label=self.selected_pose_solver_label,
            request_series=request_series))

        # Finish up any running passive tasks before we allow controls again
        for detecting_request in self._passive_detecting_requests.values():
            if detecting_request.request_id is not None:
                self.active_request_ids.append(detecting_request.request_id)
        self._passive_detecting_requests.clear()
        if self._passive_solving_request_id is not None:
            self.active_request_ids.append(self._passive_solving_request_id)
        self._passive_solving_request_id = None

        self.is_solving = False
        self.current_phase = ACTIVE_PHASE_STOPPING

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
    
    def set_reference_target(self, marker_id, marker_diameter) -> None:
        request_series: MCastRequestSeries = MCastRequestSeries(series=[
            (SetReferenceMarkerRequest(
                marker_id=marker_id,
                marker_diameter=marker_diameter))])
        self.active_request_ids.append(self.request_series_push(
            connection_label=self.selected_pose_solver_label,
            request_series=request_series))
        
    def set_tracked_target(self, marker_id, marker_diameter) -> None:
        request_series: MCastRequestSeries = MCastRequestSeries(series=[
            (AddTargetMarkerRequest(
                marker_id=marker_id,
                marker_diameter=marker_diameter))])
        self.active_request_ids.append(self.request_series_push(
            connection_label=self.selected_pose_solver_label,
            request_series=request_series))
        
    # Right now this function doesn't update on its own
    # It needs the pose solver panel to call it every frame
    def update_loop(self) -> None:
        ui_needs_update: bool = False

        if self.is_solving:
            for detector_label, request_state in self._passive_detecting_requests.items():
                if request_state.request_id is not None:
                    _, request_state.request_id = self.update_request(request_id=request_state.request_id)
                if request_state.request_id is None:
                    if len(request_state.detected_marker_snapshots) > 0 or \
                       len(request_state.rejected_marker_snapshots) > 0:
                        detector_timestamp: str = request_state.marker_snapshot_timestamp.isoformat()
                        marker_request: AddMarkerCornersRequest = AddMarkerCornersRequest(
                            detected_marker_snapshots=request_state.detected_marker_snapshots,
                            rejected_marker_snapshots=request_state.rejected_marker_snapshots,
                            detector_label=detector_label,
                            detector_timestamp_utc_iso8601=detector_timestamp)
                        request_series: MCastRequestSeries = MCastRequestSeries(series=[marker_request])
                        request_state.request_id = self.request_series_push(
                            connection_label=self.selected_pose_solver_label,
                            request_series=request_series)
                        request_state.detected_marker_snapshots.clear()
                        request_state.rejected_marker_snapshots.clear()
                    else:
                        request_series: MCastRequestSeries = MCastRequestSeries(series=[GetMarkerSnapshotsRequest()])
                        request_state.request_id = self.request_series_push(
                            connection_label=detector_label,
                            request_series=request_series)
            if self._passive_solving_request_id is not None:
                _, self._passive_solving_request_id = \
                    self.update_request(request_id=self._passive_solving_request_id)
            if self._passive_solving_request_id is None:
                request_series: MCastRequestSeries = MCastRequestSeries(series=[GetPosesRequest()])
                self._passive_solving_request_id = self.request_series_push(
                    connection_label=self.selected_pose_solver_label,
                    request_series=request_series)

        # TODO: I think this can be moved to BasePanel class
        if len(self.active_request_ids) > 0:
            completed_request_ids: list[uuid.UUID] = list()
            for request_id in self.active_request_ids:
                _, remaining_request_id = self.update_request(request_id=request_id)
                if remaining_request_id is None:
                    completed_request_ids.append(request_id)
            for request_id in completed_request_ids:
                self.active_request_ids.remove(request_id)
            if len(self.active_request_ids) == 0:
                self.on_active_request_ids_processed(self.selected_pose_solver_label)

        return ui_needs_update
    
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
        if connection.dynamic.status == "disconnected" or connection.dynamic.status == "aborted":
            return

        if connection.dynamic.status == "disconnecting":
            if connection.dynamic.socket is not None:
                await connection.dynamic.socket.close()
                connection.dynamic.socket = None
            connection.dynamic.socket = None
            connection.dynamic.status = "disconnected"

        if connection.dynamic.status == "connecting":
            now_utc = datetime.datetime.utcnow()
            if now_utc >= connection.dynamic.next_attempt_timestamp_utc:
                connection.dynamic.attempt_count += 1
                uri: str = f"ws://{connection.static.ip_address}:{connection.static.port}/websocket"
                try:
                    connection.dynamic.socket = await connect(
                        uri=uri,
                        ping_timeout=None,
                        open_timeout=None,
                        close_timeout=None,
                        max_size=2**48)  # Default max_size might have trouble with some larger uncompressed images
                except ConnectionError as e:
                    if connection.dynamic.attempt_count >= BaseComponentConnectionDynamic.ATTEMPT_COUNT_MAXIMUM:
                        message = \
                            f"Failed to connect to {uri} with error: {str(e)}. "\
                            f"Connection is being aborted after {connection.dynamic.attempt_count} attempts."
                        self.add_status_message(severity="error", message=message)
                        connection.dynamic.status = "aborted"
                    else:
                        message: str = \
                            f"Failed to connect to {uri} with error: {str(e)}. "\
                            f"Will retry in {BaseComponentConnectionDynamic.ATTEMPT_TIME_GAP_SECONDS} seconds."
                        self.add_status_message(severity="warning", message=message)
                        connection.dynamic.next_attempt_timestamp_utc = now_utc + datetime.timedelta(
                            seconds=BaseComponentConnectionDynamic.ATTEMPT_TIME_GAP_SECONDS)
                    return
                message = f"Connected to {uri}."
                self.add_status_message(severity="info", message=message)
                connection.dynamic.status = "connected"
                connection.dynamic.attempt_count = 0

        if connection.dynamic.status == "connected":

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
                            websocket=connection.dynamic.socket,
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
                websocket=connection.dynamic.socket,
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
