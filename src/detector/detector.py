from .api import \
    CalibrationCalculateRequest, \
    CalibrationDeleteStagedRequest, \
    CalibrationImageAddRequest, \
    CalibrationImageGetRequest, \
    CalibrationImageMetadataListRequest, \
    CalibrationImageMetadataUpdateRequest, \
    CalibrationResultGetRequest, \
    CalibrationResolutionListRequest, \
    CalibrationResultMetadataListRequest, \
    CalibrationResultMetadataUpdateRequest, \
    CameraImageGetRequest, \
    CameraImageGetResponse, \
    CameraParametersGetRequest, \
    CameraParametersGetResponse, \
    CameraParametersSetRequest, \
    DetectorFrameGetRequest, \
    DetectorFrameGetResponse, \
    DetectorStartRequest, \
    DetectorStopRequest, \
    MarkerParametersGetRequest, \
    MarkerParametersGetResponse, \
    MarkerParametersSetRequest
from .calibrator import Calibrator
from .exceptions import UpdateCaptureError
from .interfaces import \
    AbstractMarkerInterface, \
    AbstractCameraInterface
from .structures import \
    DetectorConfiguration
from src.common import \
    EmptyResponse, \
    ErrorResponse, \
    MCTComponent, \
    MCTRequest, \
    MCTResponse
from src.common.structures import \
    CaptureStatus, \
    MarkerStatus
import logging
from typing import Callable


logger = logging.getLogger(__name__)


class Detector(MCTComponent):

    _detector_configuration: DetectorConfiguration
    _calibrator: Calibrator

    _camera_interface: AbstractCameraInterface
    _marker_interface: AbstractMarkerInterface

    _frame_count: int

    def __init__(
        self,
        detector_configuration: DetectorConfiguration,
        marker_interface: AbstractMarkerInterface,
        camera_interface: AbstractCameraInterface
    ):
        super().__init__(
            status_source_label=detector_configuration.serial_identifier,
            send_status_messages_to_logger=True)
        
        self._detector_configuration = detector_configuration
        self._calibrator = Calibrator(
            configuration=detector_configuration.calibrator_configuration,
            status_message_source=self.get_status_message_source())
        self._frame_count = 0

        self._camera_interface = camera_interface
        self._marker_interface = marker_interface

    def __del__(self):
        self._camera_interface.__del__()

    async def internal_update(self):
        if self._camera_interface._capture_status.status == CaptureStatus.Status.RUNNING:
            self.internal_update_capture()
        if self._marker_interface.marker_status.status == MarkerStatus.Status.RUNNING and \
           self._camera_interface._captured_timestamp_utc > self._marker_interface.marker_timestamp_utc:
            self.internal_update_marker_corners()
        self._frame_count += 1
        if self._frame_count % 1000 == 0:
            print(f"Update count: {self._frame_count}")

    def supported_request_types(self) -> dict[type[MCTRequest], Callable[[dict], MCTResponse]]:
        return_value: dict[type[MCTRequest], Callable[[dict], MCTResponse]] = super().supported_request_types()
        return_value.update({

            DetectorFrameGetRequest: self.get_marker_snapshots,
            DetectorStartRequest: self.start_capture,
            DetectorStopRequest: self.stop_capture,

            CameraImageGetRequest: self.get_capture_image,
            CameraParametersGetRequest: self.get_capture_properties,
            CameraParametersSetRequest: self.set_capture_properties,

            MarkerParametersGetRequest: self.get_detection_parameters,
            MarkerParametersSetRequest: self.set_detection_parameters,

            CalibrationCalculateRequest: self._calibrator.calibrate,
            CalibrationDeleteStagedRequest: self._calibrator.delete_staged,
            CalibrationImageAddRequest: self._calibrator.add_calibration_image,
            CalibrationImageGetRequest: self._calibrator.get_calibration_image,
            CalibrationImageMetadataListRequest: self._calibrator.list_calibration_image_metadata_list,
            CalibrationImageMetadataUpdateRequest: self._calibrator.update_calibration_image_metadata,
            CalibrationResolutionListRequest: self._calibrator.list_calibration_detector_resolutions,
            CalibrationResultGetRequest: self._calibrator.get_calibration_result,
            CalibrationResultMetadataListRequest: self._calibrator.list_calibration_result_metadata_list,
            CalibrationResultMetadataUpdateRequest: self._calibrator.update_calibration_result_metadata})
        return return_value
    
    # Camera
    def internal_update_capture(self):
        try:
            self._camera_interface.internal_update_capture()
        except UpdateCaptureError as e:
            self.add_status_message(
                severity=e.severity,
                message=e.message)

    def set_capture_properties(self, **kwargs) -> EmptyResponse | ErrorResponse:
        return self._camera_interface.set_capture_properties(**kwargs)

    def get_capture_properties(self, **_kwargs) -> CameraParametersGetResponse | ErrorResponse:
        return self._camera_interface.get_capture_properties(**_kwargs)

    def get_capture_image(self, **kwargs) -> CameraImageGetResponse:
        return self._camera_interface.get_capture_image(**kwargs)

    def start_capture(self, **kwargs) -> MCTResponse:
        return self._camera_interface.start_capture(**kwargs)

    def stop_capture(self, **kwargs) -> MCTResponse:
        return self._camera_interface.stop_capture(**kwargs)
    
    # Marker
    def set_detection_parameters(self, **kwargs) -> EmptyResponse | ErrorResponse:
        return self._marker_interface.set_detection_parameters(**kwargs)

    def get_detection_parameters(self, **_kwargs) -> MarkerParametersGetResponse | ErrorResponse:
        return self._marker_interface.get_detection_parameters(**_kwargs)

    def get_marker_snapshots(self, **kwargs) -> DetectorFrameGetResponse:
        return self._marker_interface.get_marker_snapshots(**kwargs)

    def internal_update_marker_corners(self):
        return self._marker_interface.internal_update_marker_corners(self._camera_interface._captured_image)
