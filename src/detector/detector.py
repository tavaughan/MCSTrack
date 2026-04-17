from .api import \
    DetectorFrameGetRequest, \
    DetectorFrameGetResponse, \
    DetectorParametersGetRequest, \
    DetectorParametersGetResponse, \
    DetectorParametersSetRequest, \
    DetectorParametersSetResponse, \
    DetectorQueryRequest, \
    DetectorQueryResponse, \
    DetectorStartRequest, \
    DetectorStopRequest, \
    IntrinsicCalibrationCalculateRequest, \
    IntrinsicCalibrationCalculateResponse, \
    IntrinsicCalibrationDeleteStagedRequest, \
    IntrinsicCalibrationImageAddRequest, \
    IntrinsicCalibrationImageAddResponse, \
    IntrinsicCalibrationImageGetRequest, \
    IntrinsicCalibrationImageGetResponse, \
    IntrinsicCalibrationMetadataListRequest, \
    IntrinsicCalibrationMetadataListResponse, \
    IntrinsicCalibrationImageMetadataUpdateRequest, \
    IntrinsicCalibrationResolutionListRequest, \
    IntrinsicCalibrationResolutionListResponse, \
    IntrinsicCalibrationResultGetRequest, \
    IntrinsicCalibrationResultGetResponse, \
    IntrinsicCalibrationResultGetActiveRequest, \
    IntrinsicCalibrationResultGetActiveResponse, \
    IntrinsicCalibrationResultMetadataUpdateRequest
from src.common import \
    Annotator, \
    Camera, \
    CalibrationErrorReason, \
    DetectorFrame, \
    EmptyResponse, \
    ErrorResponse, \
    ImageFormat, \
    ImageResolution, \
    ImageUtils, \
    IntrinsicCalibration, \
    IntrinsicCalibrator, \
    KeyValueMetaAbstract, \
    MCTCalibrationError, \
    MCTCameraRuntimeError, \
    MCTComponent, \
    MCTAnnotatorRuntimeError, \
    MCTRequest, \
    MCTResponse, \
    SeverityLabel
import logging
from typing import Callable, Final
from pydantic import BaseModel, Field


logger = logging.getLogger(__name__)


_ROLE_LABEL: Final[str] = "detector"


class _ConfigurationSection(BaseModel):
    implementation: str = Field()
    configuration: dict = Field()


# noinspection DuplicatedCode
class Detector(MCTComponent):

    class Configuration(BaseModel):
        """
        Top-level schema for Detector initialization data
        """
        detector_label: str = Field()
        intrinsic_calibrator: _ConfigurationSection = Field()
        camera: _ConfigurationSection = Field()
        annotator: _ConfigurationSection = Field()

    _configuration: Configuration

    _calibrator: IntrinsicCalibrator
    _camera: Camera
    _annotator: Annotator

    _frame_count: int

    def __init__(
        self,
        configuration: Configuration,
        camera_type: type[Camera],
        annotator_type: type[Annotator],
        intrinsic_calibrator_type: type[IntrinsicCalibrator]
    ):
        super().__init__(
            status_source_label="detector",
            send_status_messages_to_logger=True)
        
        self._configuration = configuration
        self._calibrator = intrinsic_calibrator_type(
            configuration=intrinsic_calibrator_type.Configuration(**configuration.intrinsic_calibrator.configuration),
            status_message_source=self._status_message_source)
        self._camera = camera_type(
            configuration=camera_type.Configuration(**configuration.camera.configuration),
            status_message_source=self.get_status_message_source())
        self._annotator = annotator_type(
            configuration=annotator_type.Configuration(**configuration.annotator.configuration),
            status_message_source=self.get_status_message_source())
        self._frame_count = 0

    def __del__(self):
        self._camera.__del__()

    def calibration_calculate(
        self,
        **kwargs
    ) -> IntrinsicCalibrationCalculateResponse | ErrorResponse:
        request: IntrinsicCalibrationCalculateRequest = self.get_kwarg(
            kwargs=kwargs,
            key="request",
            arg_type=IntrinsicCalibrationCalculateRequest)
        result_identifier: str
        intrinsic_calibration: IntrinsicCalibration
        try:
            result_identifier, intrinsic_calibration = self._calibrator.calculate(
                image_resolution=request.image_resolution)
        except MCTCalibrationError as e:
            logger.error(e.private_message)
            return ErrorResponse(message=e.public_message)
        return IntrinsicCalibrationCalculateResponse(
            result_identifier=result_identifier,
            intrinsic_calibration=intrinsic_calibration)

    def calibration_delete_staged(
        self,
        **_kwargs
    ) -> EmptyResponse | ErrorResponse:
        try:
            self._calibrator.delete_staged()
        except MCTCalibrationError as e:
            logger.error(e.private_message)
            return ErrorResponse(message=e.public_message)
        return EmptyResponse()

    def calibration_image_add(
        self,
        **_kwargs
    ) -> IntrinsicCalibrationImageAddResponse | ErrorResponse:
        try:
            image_base64: str
            image_base64, _ = self._camera.get_encoded_image(
                image_format=ImageFormat.FORMAT_PNG,
                requested_resolution=None)
            image_identifier: str = self._calibrator.add_image(image_base64=image_base64)
        except MCTCalibrationError as e:
            logger.error(e.private_message)
            return ErrorResponse(message=e.public_message)
        return IntrinsicCalibrationImageAddResponse(image_identifier=image_identifier)

    def calibration_image_get(
        self,
        **kwargs
    ) -> IntrinsicCalibrationImageGetResponse | ErrorResponse:
        request: IntrinsicCalibrationImageGetRequest = self.get_kwarg(
            kwargs=kwargs,
            key="request",
            arg_type=IntrinsicCalibrationImageGetRequest)
        image_base64: str
        try:
            image_base64 = self._calibrator.get_image_by_identifier(identifier=request.image_identifier)
        except MCTCalibrationError as e:
            logger.error(e.private_message)
            return ErrorResponse(message=e.public_message)
        return IntrinsicCalibrationImageGetResponse(image_base64=image_base64)

    def calibration_image_metadata_update(
        self,
        **kwargs
    ) -> EmptyResponse | ErrorResponse:
        request: IntrinsicCalibrationImageMetadataUpdateRequest = self.get_kwarg(
            kwargs=kwargs,
            key="request",
            arg_type=IntrinsicCalibrationImageMetadataUpdateRequest)
        try:
            self._calibrator.update_image_metadata(
                image_identifier=request.image_identifier,
                image_state=request.image_state,
                image_label=request.image_label)
        except MCTCalibrationError as e:
            logger.error(e.private_message)
            return ErrorResponse(message=e.public_message)
        return EmptyResponse()

    def calibration_metadata_list(
        self,
        **kwargs
    ) -> IntrinsicCalibrationMetadataListResponse | ErrorResponse:
        request: IntrinsicCalibrationMetadataListRequest = self.get_kwarg(
            kwargs=kwargs,
            key="request",
            arg_type=IntrinsicCalibrationMetadataListRequest)
        image_metadata_list: list[IntrinsicCalibrator.ImageMetadata]
        result_metadata_list: list[IntrinsicCalibrator.ResultMetadata]
        try:
            image_metadata_list = self._calibrator.list_image_metadata_by_image_resolution(
                image_resolution=request.image_resolution)
            result_metadata_list = self._calibrator.list_result_metadata_by_image_resolution(
                image_resolution=request.image_resolution)
        except MCTCalibrationError as e:
            logger.error(e.private_message)
            return ErrorResponse(message=e.public_message)
        return IntrinsicCalibrationMetadataListResponse(
            image_metadata_list=image_metadata_list,
            result_metadata_list=result_metadata_list)

    def calibration_resolution_list(
        self,
        **_kwargs
    ) -> IntrinsicCalibrationResolutionListResponse | ErrorResponse:
        resolutions: list[ImageResolution]
        try:
            resolutions = self._calibrator.list_resolutions()
        except MCTCalibrationError as e:
            logger.error(e.private_message)
            return ErrorResponse(message=e.public_message)
        return IntrinsicCalibrationResolutionListResponse(resolutions=resolutions)

    def calibration_result_get(
        self,
        **kwargs
    ) -> IntrinsicCalibrationResultGetResponse | ErrorResponse:
        request: IntrinsicCalibrationResultGetRequest = self.get_kwarg(
            kwargs=kwargs,
            key="request",
            arg_type=IntrinsicCalibrationResultGetRequest)
        intrinsic_calibration: IntrinsicCalibration
        try:
            intrinsic_calibration = self._calibrator.get_result(result_identifier=request.result_identifier)
        except MCTCalibrationError as e:
            logger.error(e.private_message)
            return ErrorResponse(message=e.public_message)
        return IntrinsicCalibrationResultGetResponse(intrinsic_calibration=intrinsic_calibration)

    def calibration_result_get_active(
        self,
        **_kwargs
    ) -> IntrinsicCalibrationResultGetActiveResponse | ErrorResponse:
        intrinsic_calibration: IntrinsicCalibration | None
        try:
            image_resolution: ImageResolution = self._camera.get_resolution()
            intrinsic_calibration = self._calibrator.get_result_active_by_image_resolution(image_resolution=image_resolution)
        except MCTCalibrationError as e:
            if e.reason != CalibrationErrorReason.DATA_NOT_FOUND:
                logger.error(e.private_message)
                return ErrorResponse(message=e.public_message)
            else:
                return IntrinsicCalibrationResultGetActiveResponse(intrinsic_calibration=None)
        return IntrinsicCalibrationResultGetActiveResponse(intrinsic_calibration=intrinsic_calibration)

    def calibration_result_metadata_update(
        self,
        **kwargs
    ) -> EmptyResponse | ErrorResponse:
        request: IntrinsicCalibrationResultMetadataUpdateRequest = self.get_kwarg(
            kwargs=kwargs,
            key="request",
            arg_type=IntrinsicCalibrationResultMetadataUpdateRequest)
        try:
            self._calibrator.update_result_metadata(
                result_identifier=request.result_identifier,
                result_state=request.result_state,
                result_label=request.result_label)
        except MCTCalibrationError as e:
            logger.error(e.private_message)
            return ErrorResponse(message=e.public_message)
        return EmptyResponse()

    def detector_frame_get(
        self,
        **kwargs
    ) -> DetectorFrameGetResponse | ErrorResponse:
        request: DetectorFrameGetRequest = self.get_kwarg(
            kwargs=kwargs,
            key="request",
            arg_type=DetectorFrameGetRequest)
        detector_frame: DetectorFrame
        try:
            detector_frame = DetectorFrame(
                annotations=list(),
                timestamp_utc_iso8601=self._annotator.get_changed_timestamp().isoformat(),
                image_resolution=self._camera.get_resolution())
            if request.include_detected:
                detector_frame.annotations += self._annotator.get_markers_detected()
            if request.include_rejected:
                detector_frame.annotations += self._annotator.get_markers_rejected()
            if request.include_image:
                detector_frame.image_base64 = ImageUtils.image_to_base64(
                    image_data=self._camera.get_image(),
                    image_format=request.image_format)
        except (MCTCameraRuntimeError, MCTAnnotatorRuntimeError) as e:
            return ErrorResponse(message=e.message)
        return DetectorFrameGetResponse(frame=detector_frame)

    def detector_parameters_get(
        self,
        **_kwargs
    ) -> DetectorParametersGetResponse | ErrorResponse:
        camera_resolution: ImageResolution
        camera_parameters: list[KeyValueMetaAbstract]
        annotator_parameters: list[KeyValueMetaAbstract]
        try:
            camera_resolution = self._camera.get_resolution()
            camera_parameters = self._camera.get_parameters()
            annotator_parameters = self._annotator.get_parameters()
        except (MCTAnnotatorRuntimeError, MCTCameraRuntimeError) as e:
            return ErrorResponse(message=e.message)
        return DetectorParametersGetResponse(
            camera_resolution=camera_resolution,
            camera_parameters=camera_parameters,
            annotator_parameters=annotator_parameters)

    def detector_parameters_set(
        self,
        **kwargs
    ) -> DetectorParametersSetResponse | ErrorResponse:
        request: DetectorParametersSetRequest = self.get_kwarg(
            kwargs=kwargs,
            key="request",
            arg_type=DetectorParametersSetRequest)
        new_camera_resolution: ImageResolution
        new_camera_parameters: list[KeyValueMetaAbstract]
        new_annotator_parameters: list[KeyValueMetaAbstract]
        try:
            if request.camera_parameters is not None:
                self._camera.set_parameters(parameters=request.camera_parameters)
            if request.annotator_parameters is not None:
                self._annotator.set_parameters(parameters=request.annotator_parameters)
            new_camera_resolution = self._camera.get_resolution()
            new_camera_parameters = self._camera.get_parameters()
            new_annotator_parameters = self._annotator.get_parameters()
        except (MCTAnnotatorRuntimeError, MCTCameraRuntimeError) as e:
            return ErrorResponse(message=e.message)
        return DetectorParametersSetResponse(
            camera_resolution=new_camera_resolution,
            camera_parameters=new_camera_parameters,
            annotator_parameters=new_annotator_parameters)

    def detector_query(
        self,
        **_kwargs
    ) -> DetectorQueryResponse:
        return DetectorQueryResponse(
            annotator_status=self._annotator.get_status(),
            camera_status=self._camera.get_status())

    def detector_start(
        self,
        **_kwargs
    ) -> EmptyResponse | ErrorResponse:
        try:
            self._camera.start()
        except MCTCameraRuntimeError as e:
            return ErrorResponse(message=e.message)
        return EmptyResponse()

    def detector_stop(
        self,
        **_kwargs
    ) -> EmptyResponse | ErrorResponse:
        try:
            self._camera.stop()
        except MCTCameraRuntimeError as e:
            return ErrorResponse(message=e.message)
        return EmptyResponse()

    @staticmethod
    def get_role_label():
        return _ROLE_LABEL

    def supported_request_methods(self) -> dict[type[MCTRequest], Callable[[dict], MCTResponse]]:
        return_value: dict[type[MCTRequest], Callable[[dict], MCTResponse]] = super().supported_request_methods()
        return_value.update({
            DetectorFrameGetRequest: self.detector_frame_get,
            DetectorParametersGetRequest: self.detector_parameters_get,
            DetectorParametersSetRequest: self.detector_parameters_set,
            DetectorQueryRequest: self.detector_query,
            DetectorStartRequest: self.detector_start,
            DetectorStopRequest: self.detector_stop,
            IntrinsicCalibrationCalculateRequest: self.calibration_calculate,
            IntrinsicCalibrationDeleteStagedRequest: self.calibration_delete_staged,
            IntrinsicCalibrationImageAddRequest: self.calibration_image_add,
            IntrinsicCalibrationImageGetRequest: self.calibration_image_get,
            IntrinsicCalibrationMetadataListRequest: self.calibration_metadata_list,
            IntrinsicCalibrationImageMetadataUpdateRequest: self.calibration_image_metadata_update,
            IntrinsicCalibrationResolutionListRequest: self.calibration_resolution_list,
            IntrinsicCalibrationResultGetRequest: self.calibration_result_get,
            IntrinsicCalibrationResultGetActiveRequest: self.calibration_result_get_active,
            IntrinsicCalibrationResultMetadataUpdateRequest: self.calibration_result_metadata_update})
        return return_value

    async def update(self):
        if self.time_sync_active:
            return

        if self._camera.get_status() == Camera.Status.RUNNING:
            try:
                self._camera.update()
            except MCTCameraRuntimeError as e:
                self.add_status_message(
                    severity=SeverityLabel.ERROR,
                    message=f"Exception occurred in Camera update: {e.message}")
        if self._annotator.get_status() == Annotator.Status.RUNNING and \
           self._camera.get_changed_timestamp() > self._annotator.get_changed_timestamp():
            try:
                self._annotator.update(self._camera.get_image())
            except MCTAnnotatorRuntimeError as e:
                self.add_status_message(
                    severity=SeverityLabel.ERROR,
                    message=f"Exception occurred in Annotator update: {e.message}")
        # self._frame_count += 1
        # if self._frame_count % 1000 == 0:
        #     print(f"Update count: {self._frame_count}")
