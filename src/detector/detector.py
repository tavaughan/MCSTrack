from .api import \
    CalibrationCalculateRequest, \
    CalibrationCalculateResponse, \
    CalibrationDeleteStagedRequest, \
    CalibrationImageAddRequest, \
    CalibrationImageAddResponse, \
    CalibrationImageGetRequest, \
    CalibrationImageGetResponse, \
    CalibrationImageMetadataListRequest, \
    CalibrationImageMetadataListResponse, \
    CalibrationImageMetadataUpdateRequest, \
    CalibrationResolutionListRequest, \
    CalibrationResolutionListResponse, \
    CalibrationResultGetRequest, \
    CalibrationResultGetResponse, \
    CalibrationResultGetActiveRequest, \
    CalibrationResultGetActiveResponse, \
    CalibrationResultMetadataListRequest, \
    CalibrationResultMetadataListResponse, \
    CalibrationResultMetadataUpdateRequest, \
    CameraImageGetRequest, \
    CameraImageGetResponse, \
    CameraParametersGetRequest, \
    CameraParametersGetResponse, \
    CameraParametersSetRequest, \
    CameraParametersSetResponse, \
    CameraResolutionGetRequest, \
    CameraResolutionGetResponse, \
    DetectorFrameGetRequest, \
    DetectorFrameGetResponse, \
    DetectorStartRequest, \
    DetectorStopRequest, \
    MarkerParametersGetRequest, \
    MarkerParametersGetResponse, \
    MarkerParametersSetRequest
from .calibrator import Calibrator
from .exceptions import \
    MCTDetectorRuntimeError
from .interfaces import \
    AbstractMarker, \
    AbstractCamera
from .structures import \
    CalibrationImageMetadata, \
    CalibrationResultMetadata, \
    CameraStatus, \
    DetectorConfiguration, \
    MarkerStatus
from src.common import \
    EmptyResponse, \
    ErrorResponse, \
    get_kwarg, \
    MCTComponent, \
    MCTRequest, \
    MCTResponse
from src.common.structures import \
    DetectorFrame, \
    DetectionParameters, \
    ImageResolution, \
    IntrinsicCalibration, \
    KeyValueMetaAbstract
import logging
from typing import Callable


logger = logging.getLogger(__name__)


class Detector(MCTComponent):

    _detector_configuration: DetectorConfiguration

    _calibrator: Calibrator
    _camera: AbstractCamera
    _marker: AbstractMarker

    _frame_count: int

    def __init__(
        self,
        detector_configuration: DetectorConfiguration,
        camera_type: type[AbstractCamera],
        marker_type: type[AbstractMarker]
    ):
        super().__init__(
            status_source_label="detector",
            send_status_messages_to_logger=True)
        
        self._detector_configuration = detector_configuration
        self._calibrator = Calibrator(
            configuration=detector_configuration.calibrator_configuration,
            status_message_source=self.get_status_message_source())
        self._camera = camera_type(
            configuration=detector_configuration.camera_configuration,
            status_message_source=self.get_status_message_source())
        self._marker = marker_type(
            configuration=detector_configuration.marker_configuration,
            status_message_source=self.get_status_message_source())
        self._frame_count = 0

    def __del__(self):
        self._camera.__del__()

    def calibration_calculate(self, **kwargs) -> CalibrationCalculateResponse | ErrorResponse:
        request: CalibrationCalculateRequest = get_kwarg(
            kwargs=kwargs,
            key="request",
            arg_type=CalibrationCalculateRequest)
        result_identifier: str
        intrinsic_calibration: IntrinsicCalibration
        try:
            result_identifier, intrinsic_calibration = self._calibrator.calculate(request.image_resolution)
        except MCTDetectorRuntimeError as e:
            return ErrorResponse(message=e.message)
        return CalibrationCalculateResponse(
            result_identifier=result_identifier,
            intrinsic_calibration=intrinsic_calibration)

    def calibration_delete_staged(self, **_kwargs) -> EmptyResponse | ErrorResponse:
        try:
            self._calibrator.delete_staged()
        except MCTDetectorRuntimeError as e:
            return ErrorResponse(message=e.message)
        return EmptyResponse()

    def calibration_image_add(self, **_kwargs) -> CalibrationImageAddResponse | ErrorResponse:
        try:
            image_base64: str = self._camera.get_encoded_image(image_format=".png")
            image_identifier: str = self._calibrator.add_image(image_base64=image_base64)
        except MCTDetectorRuntimeError as e:
            return ErrorResponse(message=e.message)
        return CalibrationImageAddResponse(image_identifier=image_identifier)

    def calibration_image_get(self, **kwargs) -> CalibrationImageGetResponse | ErrorResponse:
        request: CalibrationImageGetRequest = get_kwarg(
            kwargs=kwargs,
            key="request",
            arg_type=CalibrationImageGetRequest)
        image_base64: str
        try:
            image_base64 = self._calibrator.get_image(image_identifier=request.image_identifier)
        except MCTDetectorRuntimeError as e:
            return ErrorResponse(message=e.message)
        return CalibrationImageGetResponse(image_base64=image_base64)

    def calibration_image_metadata_list(self, **kwargs) -> CalibrationImageMetadataListResponse | ErrorResponse:
        request: CalibrationImageMetadataListRequest = get_kwarg(
            kwargs=kwargs,
            key="request",
            arg_type=CalibrationImageMetadataListRequest)
        image_metadata_list: list[CalibrationImageMetadata]
        try:
            image_metadata_list = self._calibrator.list_image_metadata(
                image_resolution=request.image_resolution)
        except MCTDetectorRuntimeError as e:
            return ErrorResponse(message=e.message)
        return CalibrationImageMetadataListResponse(metadata_list=image_metadata_list)

    def calibration_image_metadata_update(self, **kwargs) -> EmptyResponse | ErrorResponse:
        request: CalibrationImageMetadataUpdateRequest = get_kwarg(
            kwargs=kwargs,
            key="request",
            arg_type=CalibrationImageMetadataUpdateRequest)
        try:
            self._calibrator.update_image_metadata(
                image_identifier=request.image_identifier,
                image_state=request.image_state,
                image_label=request.image_label)
        except MCTDetectorRuntimeError as e:
            return ErrorResponse(message=e.message)
        return EmptyResponse()

    def calibration_resolution_list(self, **_kwargs) -> CalibrationResolutionListResponse | ErrorResponse:
        resolutions: list[ImageResolution]
        try:
            resolutions = self._calibrator.list_resolutions()
        except MCTDetectorRuntimeError as e:
            return ErrorResponse(message=e.message)
        return CalibrationResolutionListResponse(resolutions=resolutions)

    def calibration_result_get(self, **kwargs) -> CalibrationResultGetResponse | ErrorResponse:
        request: CalibrationResultGetRequest = get_kwarg(
            kwargs=kwargs,
            key="request",
            arg_type=CalibrationResultGetRequest)
        intrinsic_calibration: IntrinsicCalibration
        try:
            intrinsic_calibration = self._calibrator.get_result(result_identifier=request.result_identifier)
        except MCTDetectorRuntimeError as e:
            return ErrorResponse(message=e.message)
        return CalibrationResultGetResponse(intrinsic_calibration=intrinsic_calibration)

    def calibration_result_get_active(self, **_kwargs) -> CalibrationResultGetActiveResponse | ErrorResponse:
        intrinsic_calibration: IntrinsicCalibration | None
        try:
            image_resolution: ImageResolution = self._camera.get_resolution()
            intrinsic_calibration = self._calibrator.get_result_active(image_resolution=image_resolution)
        except MCTDetectorRuntimeError as e:
            return ErrorResponse(message=e.message)
        return CalibrationResultGetActiveResponse(intrinsic_calibration=intrinsic_calibration)

    def calibration_result_metadata_list(self, **kwargs) -> CalibrationResultMetadataListResponse | ErrorResponse:
        request: CalibrationResultMetadataListRequest = get_kwarg(
            kwargs=kwargs,
            key="request",
            arg_type=CalibrationResultMetadataListRequest)
        result_metadata_list: list[CalibrationResultMetadata]
        try:
            result_metadata_list = self._calibrator.list_result_metadata(
                image_resolution=request.image_resolution)
        except MCTDetectorRuntimeError as e:
            return ErrorResponse(message=e.message)
        return CalibrationResultMetadataListResponse(metadata_list=result_metadata_list)

    def calibration_result_metadata_update(self, **kwargs) -> EmptyResponse | ErrorResponse:
        request: CalibrationResultMetadataUpdateRequest = get_kwarg(
            kwargs=kwargs,
            key="request",
            arg_type=CalibrationResultMetadataUpdateRequest)
        try:
            self._calibrator.update_result_metadata(
                result_identifier=request.result_identifier,
                result_state=request.result_state,
                result_label=request.result_label)
        except MCTDetectorRuntimeError as e:
            return ErrorResponse(message=e.message)
        return EmptyResponse()

    def camera_image_get(self, **kwargs) -> CameraImageGetResponse | ErrorResponse:
        request: CameraImageGetRequest = get_kwarg(
            kwargs=kwargs,
            key="request",
            arg_type=CameraImageGetRequest)
        encoded_image_base64: str
        try:
            encoded_image_base64 = self._camera.get_encoded_image(image_format=request.format)
        except MCTDetectorRuntimeError as e:
            return ErrorResponse(message=e.message)
        return CameraImageGetResponse(
            format=request.format,
            image_base64=encoded_image_base64)

    def camera_parameters_get(self, **_kwargs) -> CameraParametersGetResponse | ErrorResponse:
        parameters: list[KeyValueMetaAbstract]
        try:
            parameters = self._camera.get_parameters()
        except MCTDetectorRuntimeError as e:
            return ErrorResponse(message=e.message)
        return CameraParametersGetResponse(parameters=parameters)

    def camera_parameters_set(self, **kwargs) -> CameraParametersSetResponse | ErrorResponse:
        request: CameraParametersSetRequest = get_kwarg(
            kwargs=kwargs,
            key="request",
            arg_type=CameraParametersSetRequest)
        new_resolution: ImageResolution
        try:
            self._camera.set_parameters(parameters=request.parameters)
            new_resolution = self._camera.get_resolution()
        except MCTDetectorRuntimeError as e:
            return ErrorResponse(message=e.message)
        return CameraParametersSetResponse(resolution=new_resolution)

    def camera_resolution_get(self, **_kwargs) -> CameraResolutionGetResponse | ErrorResponse:
        image_resolution: ImageResolution
        try:
            image_resolution = self._camera.get_resolution()
        except MCTDetectorRuntimeError as e:
            return ErrorResponse(message=e.message)
        return CameraResolutionGetResponse(resolution=image_resolution)

    def detector_frame_get(self, **kwargs) -> DetectorFrameGetResponse | ErrorResponse:
        request: DetectorFrameGetRequest = get_kwarg(
            kwargs=kwargs,
            key="request",
            arg_type=DetectorFrameGetRequest)
        detector_frame: DetectorFrame
        try:
            detector_frame = DetectorFrame(
                detected_marker_snapshots=list(),
                rejected_marker_snapshots=list(),
                timestamp_utc_iso8601=self._marker.get_changed_timestamp().isoformat())
            if request.include_detected:
                detector_frame.detected_marker_snapshots = self._marker.get_markers_detected()
            if request.include_rejected:
                detector_frame.rejected_marker_snapshots = self._marker.get_markers_rejected()
        except MCTDetectorRuntimeError as e:
            return ErrorResponse(message=e.message)
        return DetectorFrameGetResponse(frame=detector_frame)

    def detector_start(self, **_kwargs) -> EmptyResponse | ErrorResponse:
        try:
            self._camera.start()
        except MCTDetectorRuntimeError as e:
            return ErrorResponse(message=e.message)
        return EmptyResponse()

    def detector_stop(self, **_kwargs) -> EmptyResponse | ErrorResponse:
        try:
            self._camera.stop()
        except MCTDetectorRuntimeError as e:
            return ErrorResponse(message=e.message)
        return EmptyResponse()

    def marker_parameters_get(self, **_kwargs) -> MarkerParametersGetResponse | ErrorResponse:
        parameters: DetectionParameters
        try:
            parameters = self._marker.get_parameters()
        except MCTDetectorRuntimeError as e:
            return ErrorResponse(message=e.message)
        return MarkerParametersGetResponse(parameters=parameters)

    def marker_parameters_set(self, **kwargs) -> EmptyResponse | ErrorResponse:
        request: MarkerParametersSetRequest = get_kwarg(
            kwargs=kwargs,
            key="request",
            arg_type=MarkerParametersSetRequest)
        try:
            self._marker.set_parameters(parameters=request.parameters)
        except MCTDetectorRuntimeError as e:
            return ErrorResponse(message=e.message)
        return EmptyResponse()

    def supported_request_types(self) -> dict[type[MCTRequest], Callable[[dict], MCTResponse]]:
        return_value: dict[type[MCTRequest], Callable[[dict], MCTResponse]] = super().supported_request_types()
        return_value.update({
            DetectorFrameGetRequest: self.detector_frame_get,
            DetectorStartRequest: self.detector_start,
            DetectorStopRequest: self.detector_stop,
            CalibrationCalculateRequest: self.calibration_calculate,
            CalibrationDeleteStagedRequest: self.calibration_delete_staged,
            CalibrationImageAddRequest: self.calibration_image_add,
            CalibrationImageGetRequest: self.calibration_image_get,
            CalibrationImageMetadataListRequest: self.calibration_image_metadata_list,
            CalibrationImageMetadataUpdateRequest: self.calibration_image_metadata_update,
            CalibrationResolutionListRequest: self.calibration_resolution_list,
            CalibrationResultGetRequest: self.calibration_result_get,
            CalibrationResultGetActiveRequest: self.calibration_result_get_active,
            CalibrationResultMetadataListRequest: self.calibration_result_metadata_list,
            CalibrationResultMetadataUpdateRequest: self.calibration_result_metadata_update,
            CameraImageGetRequest: self.camera_image_get,
            CameraParametersGetRequest: self.camera_parameters_get,
            CameraParametersSetRequest: self.camera_parameters_set,
            CameraResolutionGetRequest: self.camera_resolution_get,
            MarkerParametersGetRequest: self.marker_parameters_get,
            MarkerParametersSetRequest: self.marker_parameters_set})
        return return_value

    async def update(self):
        if self._camera.get_status() == CameraStatus.RUNNING:
            try:
                self._camera.update()
            except MCTDetectorRuntimeError as e:
                self.add_status_message(severity="error", message=e.message)
        if self._marker.get_status() == MarkerStatus.RUNNING and \
           self._camera.get_changed_timestamp() > self._marker.get_changed_timestamp():
            self._marker.update(self._camera.get_image())
        self._frame_count += 1
        if self._frame_count % 1000 == 0:
            print(f"Update count: {self._frame_count}")
