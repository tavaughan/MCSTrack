from .api import \
    ExtrinsicCalibrationCalculateRequest, \
    ExtrinsicCalibrationCalculateResponse, \
    ExtrinsicCalibrationDeleteStagedRequest, \
    ExtrinsicCalibrationImageAddRequest, \
    ExtrinsicCalibrationImageAddResponse, \
    ExtrinsicCalibrationImageGetRequest, \
    ExtrinsicCalibrationImageGetResponse, \
    ExtrinsicCalibrationImageMetadataUpdateRequest, \
    ExtrinsicCalibrationMetadataListRequest, \
    ExtrinsicCalibrationMetadataListResponse, \
    ExtrinsicCalibrationResultGetActiveRequest, \
    ExtrinsicCalibrationResultGetActiveResponse, \
    ExtrinsicCalibrationResultGetRequest, \
    ExtrinsicCalibrationResultGetResponse, \
    ExtrinsicCalibrationResultMetadataUpdateRequest, \
    PoseSolverDetectorFrameAddRequest, \
    PoseSolverExtrinsicClearRequest, \
    PoseSolverExtrinsicSetRequest, \
    PoseSolverPosesGetRequest, \
    PoseSolverPosesGetResponse, \
    PoseSolverTargetAddRequest, \
    PoseSolverTargetsClearRequest, \
    PoseSolverTargetsSetRequest, \
    MixerFrameGetRequest, \
    MixerFrameGetResponse, \
    MixerIntrinsicUpdateRequest, \
    MixerQueryRequest, \
    MixerQueryResponse, \
    MixerStartRequest, \
    MixerStopRequest
from src.common import \
    CalibrationErrorReason, \
    EmptyResponse, \
    ErrorResponse, \
    ExtrinsicCalibration, \
    ExtrinsicCalibrator, \
    MCTCalibrationError, \
    MCTComponent, \
    MCTRequest, \
    MCTResponse, \
    MixerFrame, \
    Pose, \
    PoseSolver, \
    PoseSolverException
from enum import StrEnum
import logging
from pydantic import BaseModel, Field
from typing import Callable, Final


logger = logging.getLogger(__name__)


_ROLE_LABEL: Final[str] = "mixer"


class _ConfigurationSection(BaseModel):
    implementation: str = Field()
    configuration: dict = Field()


# noinspection DuplicatedCode
class Mixer(MCTComponent):

    class Configuration(BaseModel):
        mixer_label: str = Field()
        pose_solver: _ConfigurationSection = Field()
        extrinsic_calibrator: _ConfigurationSection = Field()

    class Status(StrEnum):
        STOPPED = "stopped"
        RUNNING = "running"
        FAILURE = "failure"

    _status: Status

    _configuration: Configuration
    _pose_solver: PoseSolver
    _extrinsic_calibrator: ExtrinsicCalibrator

    def __init__(
        self,
        configuration: Configuration,
        pose_solver_type: type[PoseSolver],
        extrinsic_calibrator_type: type[ExtrinsicCalibrator]
    ):
        super().__init__(
            status_source_label=configuration.mixer_label,
            send_status_messages_to_logger=True)

        self._configuration = configuration
        self._pose_solver = pose_solver_type(
            configuration=pose_solver_type.Configuration(**configuration.pose_solver.configuration),
            status_message_source=self._status_message_source)
        self._extrinsic_calibrator = extrinsic_calibrator_type(
            configuration=extrinsic_calibrator_type.Configuration(**configuration.extrinsic_calibrator.configuration),
            status_message_source=self._status_message_source)

        self._status = Mixer.Status.STOPPED

    def extrinsic_calibrator_calculate(
        self,
        **_kwargs
    ) -> ExtrinsicCalibrationCalculateResponse | ErrorResponse:
        result_identifier: str
        extrinsic_calibration: ExtrinsicCalibration
        try:
            result_identifier, extrinsic_calibration = self._extrinsic_calibrator.calculate()
        except MCTCalibrationError as e:
            logger.error(e.private_message)
            return ErrorResponse(message=e.public_message)
        return ExtrinsicCalibrationCalculateResponse(
            result_identifier=result_identifier,
            extrinsic_calibration=extrinsic_calibration)

    def extrinsic_calibrator_delete_staged(
        self,
        **_kwargs
    ) -> EmptyResponse | ErrorResponse:
        try:
            self._extrinsic_calibrator.delete_staged()
        except MCTCalibrationError as e:
            logger.error(e.private_message)
            return ErrorResponse(message=e.public_message)
        return EmptyResponse()

    def extrinsic_calibrator_image_add(
        self,
        **kwargs
    ) -> ExtrinsicCalibrationImageAddResponse | ErrorResponse:
        request: ExtrinsicCalibrationImageAddRequest = self.get_kwarg(
            kwargs=kwargs,
            key="request",
            arg_type=ExtrinsicCalibrationImageAddRequest)
        image_identifier: str
        try:
            image_identifier = self._extrinsic_calibrator.add_image(
                image_base64=request.image_base64,
                detector_label=request.detector_label,
                timestamp_utc_iso8601=request.timestamp_utc_iso8601)
        except MCTCalibrationError as e:
            logger.error(e.private_message)
            return ErrorResponse(message=e.public_message)
        return ExtrinsicCalibrationImageAddResponse(image_identifier=image_identifier)

    def extrinsic_calibrator_image_get(
        self,
        **kwargs
    ) -> ExtrinsicCalibrationImageGetResponse | ErrorResponse:
        request: ExtrinsicCalibrationImageGetRequest = self.get_kwarg(
            kwargs=kwargs,
            key="request",
            arg_type=ExtrinsicCalibrationImageGetRequest)
        image_base64: str
        try:
            image_base64 = self._extrinsic_calibrator.get_image_by_identifier(identifier=request.image_identifier)
        except MCTCalibrationError as e:
            logger.error(e.private_message)
            return ErrorResponse(message=e.public_message)
        return ExtrinsicCalibrationImageGetResponse(image_base64=image_base64)

    def extrinsic_calibrator_image_metadata_list(
        self,
        **_kwargs
    ) -> ExtrinsicCalibrationMetadataListResponse | ErrorResponse:
        image_metadata_list: list[ExtrinsicCalibrator.ImageMetadata]
        result_metadata_list: list[ExtrinsicCalibrator.ResultMetadata]
        try:
            image_metadata_list = self._extrinsic_calibrator.list_image_metadata()
            result_metadata_list = self._extrinsic_calibrator.list_result_metadata()
        except MCTCalibrationError as e:
            logger.error(e.private_message)
            return ErrorResponse(message=e.public_message)
        return ExtrinsicCalibrationMetadataListResponse(
            image_metadata_list=image_metadata_list,
            result_metadata_list=result_metadata_list)

    def extrinsic_calibrator_image_metadata_update(
        self,
        **kwargs
    ) -> EmptyResponse | ErrorResponse:
        request: ExtrinsicCalibrationImageMetadataUpdateRequest = self.get_kwarg(
            kwargs=kwargs,
            key="request",
            arg_type=ExtrinsicCalibrationImageMetadataUpdateRequest)
        try:
            self._extrinsic_calibrator.update_image_metadata(
                image_identifier=request.image_identifier,
                image_state=request.image_state,
                image_label=request.image_label)
        except MCTCalibrationError as e:
            logger.error(e.private_message)
            return ErrorResponse(message=e.public_message)
        return EmptyResponse()

    def extrinsic_calibrator_result_get_active(
        self,
        **_kwargs
    ) -> ExtrinsicCalibrationResultGetActiveResponse | ErrorResponse:
        calibration: ExtrinsicCalibration
        try:
            calibration = self._extrinsic_calibrator.get_result_active()
        except MCTCalibrationError as e:
            if e.reason != CalibrationErrorReason.DATA_NOT_FOUND:
                logger.error(e.private_message)
                return ErrorResponse(message=e.public_message)
            else:
                return ExtrinsicCalibrationResultGetActiveResponse(extrinsic_calibration=None)
        return ExtrinsicCalibrationResultGetActiveResponse(extrinsic_calibration=calibration)

    def extrinsic_calibrator_result_get(
        self,
        **kwargs
    ) -> ExtrinsicCalibrationResultGetResponse | ErrorResponse:
        request: ExtrinsicCalibrationResultGetRequest = self.get_kwarg(
            kwargs=kwargs,
            key="request",
            arg_type=ExtrinsicCalibrationResultGetRequest)
        calibration: ExtrinsicCalibration
        try:
            calibration = self._extrinsic_calibrator.get_result(result_identifier=request.result_identifier)
        except MCTCalibrationError as e:
            logger.error(e.private_message)
            return ErrorResponse(message=e.public_message)
        return ExtrinsicCalibrationResultGetResponse(extrinsic_calibration=calibration)

    def extrinsic_calibrator_result_metadata_update(
        self,
        **kwargs
    ) -> EmptyResponse | ErrorResponse:
        request: ExtrinsicCalibrationResultMetadataUpdateRequest = self.get_kwarg(
            kwargs=kwargs,
            key="request",
            arg_type=ExtrinsicCalibrationResultMetadataUpdateRequest)
        try:
            self._extrinsic_calibrator.update_result_metadata(
                result_identifier=request.result_identifier,
                result_state=request.result_state,
                result_label=request.result_label)
        except MCTCalibrationError as e:
            logger.error(e.private_message)
            return ErrorResponse(message=e.public_message)
        return EmptyResponse()

    @staticmethod
    def get_role_label():
        return _ROLE_LABEL

    def mixer_frame_get(self, **_kwargs) -> MixerFrameGetResponse | ErrorResponse:
        detector_poses, target_poses = self._pose_solver.get_poses()
        return MixerFrameGetResponse(
            frame=MixerFrame(
                detector_poses=detector_poses,
                target_poses=target_poses,
                timestamp_utc_iso8601=self._pose_solver.get_detector_frame_timestamp().isoformat()))

    def mixer_query(self, **_kwargs) -> MixerQueryResponse:
        return MixerQueryResponse(mixer_status=self._status)

    def mixer_start(self, **_kwargs) -> EmptyResponse:
        self._status = Mixer.Status.RUNNING
        return EmptyResponse()

    def mixer_stop(self, **_kwargs) -> EmptyResponse:
        self._status = Mixer.Status.STOPPED
        return EmptyResponse()

    def mixer_update_intrinsic_parameters(
        self,
        **kwargs
    ) -> EmptyResponse | ErrorResponse:
        request: MixerIntrinsicUpdateRequest = self.get_kwarg(
            kwargs=kwargs,
            key="request",
            arg_type=MixerIntrinsicUpdateRequest)
        self._pose_solver.set_intrinsic_parameters(
            detector_label=request.detector_label,
            intrinsic_parameters=request.intrinsic_parameters)
        self._extrinsic_calibrator.intrinsic_parameters_update(
            detector_label=request.detector_label,
            intrinsic_parameters=request.intrinsic_parameters)
        return EmptyResponse()

    def pose_solver_detector_frame_add(self, **kwargs) -> EmptyResponse | ErrorResponse:
        request: PoseSolverDetectorFrameAddRequest = self.get_kwarg(
            kwargs=kwargs,
            key="request",
            arg_type=PoseSolverDetectorFrameAddRequest)
        try:
            self._pose_solver.add_detector_frame(
                detector_label=request.detector_label,
                frame_annotations=request.detector_frame.annotations,
                frame_timestamp_utc=request.detector_frame.timestamp_utc)
        except PoseSolverException as e:
            return ErrorResponse(message=e.message)
        return EmptyResponse()

    def pose_solver_target_add(self, **kwargs) -> EmptyResponse | ErrorResponse:
        request: PoseSolverTargetAddRequest = self.get_kwarg(
            kwargs=kwargs,
            key="request",
            arg_type=PoseSolverTargetAddRequest)
        try:
            self._pose_solver.add_target(target=request.target)
        except PoseSolverException as e:
            return ErrorResponse(message=e.message)
        return EmptyResponse()

    def pose_solver_targets_clear(self, **_kwargs) -> EmptyResponse | ErrorResponse:
        try:
            self._pose_solver.clear_targets()
        except PoseSolverException as e:
            return ErrorResponse(message=e.message)
        return EmptyResponse()

    def pose_solver_poses_get(self, **_kwargs) -> PoseSolverPosesGetResponse | ErrorResponse:
        detector_poses: list[Pose]
        target_poses: list[Pose]
        try:
            detector_poses, target_poses = self._pose_solver.get_poses()
        except PoseSolverException as e:
            return ErrorResponse(message=e.message)
        return PoseSolverPosesGetResponse(
            detector_poses=detector_poses,
            target_poses=target_poses)

    def pose_solver_extrinsic_clear(self, **_kwargs) -> EmptyResponse | ErrorResponse:
        self._pose_solver.clear_extrinsic_matrices()
        return EmptyResponse()

    def pose_solver_extrinsic_set(self, **kwargs) -> EmptyResponse | ErrorResponse:
        request: PoseSolverExtrinsicSetRequest = self.get_kwarg(
            kwargs=kwargs,
            key="request",
            arg_type=PoseSolverExtrinsicSetRequest)
        try:
            self._pose_solver.set_extrinsic_matrix(
                detector_label=request.detector_label,
                transform_to_reference=request.transform_to_reference)
        except PoseSolverException as e:
            return ErrorResponse(message=e.message)
        return EmptyResponse()

    def pose_solver_set_targets(self, **kwargs) -> EmptyResponse | ErrorResponse:
        request: PoseSolverTargetsSetRequest = self.get_kwarg(
            kwargs=kwargs,
            key="request",
            arg_type=PoseSolverTargetsSetRequest)
        try:
            self._pose_solver.set_targets(targets=request.targets)
        except PoseSolverException as e:
            return ErrorResponse(message=e.message)
        return EmptyResponse()

    def supported_request_methods(self) -> dict[type[MCTRequest], Callable[[dict], MCTResponse]]:
        return_value: dict[type[MCTRequest], Callable[[dict], MCTResponse]] = super().supported_request_methods()
        return_value.update({
            ExtrinsicCalibrationCalculateRequest: self.extrinsic_calibrator_calculate,
            ExtrinsicCalibrationDeleteStagedRequest: self.extrinsic_calibrator_delete_staged,
            ExtrinsicCalibrationImageAddRequest: self.extrinsic_calibrator_image_add,
            ExtrinsicCalibrationImageGetRequest: self.extrinsic_calibrator_image_get,
            ExtrinsicCalibrationMetadataListRequest: self.extrinsic_calibrator_image_metadata_list,
            ExtrinsicCalibrationImageMetadataUpdateRequest: self.extrinsic_calibrator_image_metadata_update,
            ExtrinsicCalibrationResultGetActiveRequest: self.extrinsic_calibrator_result_get_active,
            ExtrinsicCalibrationResultGetRequest: self.extrinsic_calibrator_result_get,
            ExtrinsicCalibrationResultMetadataUpdateRequest: self.extrinsic_calibrator_result_metadata_update,
            MixerFrameGetRequest: self.mixer_frame_get,
            MixerIntrinsicUpdateRequest: self.mixer_update_intrinsic_parameters,
            MixerQueryRequest: self.mixer_query,
            MixerStartRequest: self.mixer_start,
            MixerStopRequest: self.mixer_stop,
            PoseSolverDetectorFrameAddRequest: self.pose_solver_detector_frame_add,
            PoseSolverExtrinsicClearRequest: self.pose_solver_extrinsic_clear,
            PoseSolverExtrinsicSetRequest: self.pose_solver_extrinsic_set,
            PoseSolverPosesGetRequest: self.pose_solver_poses_get,
            PoseSolverTargetAddRequest: self.pose_solver_target_add,
            PoseSolverTargetsClearRequest: self.pose_solver_targets_clear,
            PoseSolverTargetsSetRequest: self.pose_solver_set_targets})
        return return_value

    async def update(self):
        if self.time_sync_active:
            return
        if self._status == Mixer.Status.RUNNING:
            try:
                self._pose_solver.update()
            except Exception as e:
                logger.error(e)
