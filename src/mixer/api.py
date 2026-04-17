from src.common import \
    DequeueStatusMessagesResponse, \
    DetectorFrame, \
    EmptyResponse, \
    ErrorResponse, \
    ExtrinsicCalibration, \
    ExtrinsicCalibrator, \
    IntrinsicParameters, \
    Matrix4x4, \
    MCTRequest, \
    MCTResponse, \
    MixerFrame, \
    Pose, \
    Target, \
    TimestampGetResponse
from pydantic import Field


class ExtrinsicCalibrationCalculateRequest(MCTRequest):
    @staticmethod
    def type_identifier() -> str:
        return "mixer_extrinsic_calibration_calculate"

    parsable_type: str = Field(default=type_identifier())


class ExtrinsicCalibrationCalculateResponse(MCTResponse):
    @staticmethod
    def type_identifier() -> str:
        return "mixer_extrinsic_calibration_calculate"

    parsable_type: str = Field(default=type_identifier())

    result_identifier: str = Field()
    extrinsic_calibration: ExtrinsicCalibration = Field()


class ExtrinsicCalibrationDeleteStagedRequest(MCTRequest):
    @staticmethod
    def type_identifier() -> str:
        return "mixer_extrinsic_calibration_delete_staged"

    parsable_type: str = Field(default=type_identifier())


class ExtrinsicCalibrationImageAddRequest(MCTRequest):
    @staticmethod
    def type_identifier() -> str:
        return "mixer_extrinsic_calibration_image_add"

    parsable_type: str = Field(default=type_identifier())

    image_base64: str = Field()
    detector_label: str = Field()
    timestamp_utc_iso8601: str = Field()


class ExtrinsicCalibrationImageAddResponse(MCTResponse):
    @staticmethod
    def type_identifier() -> str:
        return "mixer_extrinsic_calibration_image_add"

    parsable_type: str = Field(default=type_identifier())

    image_identifier: str = Field()


class ExtrinsicCalibrationImageGetRequest(MCTRequest):
    @staticmethod
    def type_identifier() -> str:
        return "mixer_extrinsic_calibration_image_get"

    parsable_type: str = Field(default=type_identifier())

    image_identifier: str = Field()


class ExtrinsicCalibrationImageGetResponse(MCTResponse):
    @staticmethod
    def type_identifier() -> str:
        return "mixer_extrinsic_calibration_image_get"

    parsable_type: str = Field(default=type_identifier())

    image_base64: str = Field()


class ExtrinsicCalibrationImageMetadataUpdateRequest(MCTRequest):
    @staticmethod
    def type_identifier() -> str:
        return "mixer_extrinsic_calibration_image_metadata_update"

    parsable_type: str = Field(default=type_identifier())

    image_identifier: str = Field()
    image_state: ExtrinsicCalibrator.ImageState = Field()
    image_label: str | None = Field(default=None)


class ExtrinsicCalibrationMetadataListRequest(MCTRequest):
    @staticmethod
    def type_identifier() -> str:
        return "mixer_extrinsic_calibration_image_metadata_list"

    parsable_type: str = Field(default=type_identifier())


class ExtrinsicCalibrationMetadataListResponse(MCTResponse):
    @staticmethod
    def type_identifier() -> str:
        return "mixer_extrinsic_calibration_image_metadata_list"

    parsable_type: str = Field(default=type_identifier())

    image_metadata_list: list[ExtrinsicCalibrator.ImageMetadata] = Field(default_factory=list)
    result_metadata_list: list[ExtrinsicCalibrator.ResultMetadata] = Field(default_factory=list)


class ExtrinsicCalibrationResultGetRequest(MCTRequest):
    @staticmethod
    def type_identifier() -> str:
        return "mixer_extrinsic_calibration_result_get"

    parsable_type: str = Field(default=type_identifier())

    result_identifier: str = Field()


class ExtrinsicCalibrationResultGetResponse(MCTResponse):
    @staticmethod
    def type_identifier() -> str:
        return "mixer_extrinsic_calibration_result_get"

    parsable_type: str = Field(default=type_identifier())

    extrinsic_calibration: ExtrinsicCalibration = Field()


class ExtrinsicCalibrationResultGetActiveRequest(MCTRequest):
    @staticmethod
    def type_identifier() -> str:
        return "mixer_extrinsic_calibration_result_active_get"

    parsable_type: str = Field(default=type_identifier())


class ExtrinsicCalibrationResultGetActiveResponse(MCTResponse):
    @staticmethod
    def type_identifier() -> str:
        return "mixer_extrinsic_calibration_result_active_get"

    parsable_type: str = Field(default=type_identifier())

    extrinsic_calibration: ExtrinsicCalibration | None = Field()


class ExtrinsicCalibrationResultMetadataUpdateRequest(MCTRequest):
    @staticmethod
    def type_identifier() -> str:
        return "mixer_extrinsic_calibration_result_metadata_update"

    parsable_type: str = Field(default=type_identifier())

    result_identifier: str = Field()
    result_state: ExtrinsicCalibrator.ResultState = Field()
    result_label: str | None = Field(default=None)


class PoseSolverDetectorFrameAddRequest(MCTRequest):
    @staticmethod
    def type_identifier() -> str:
        return "mixer_pose_solver_add_marker_corners"

    # noinspection PyTypeHints
    parsable_type: str = Field(default=type_identifier())

    detector_label: str = Field()
    detector_frame: DetectorFrame = Field()


class PoseSolverExtrinsicClearRequest(MCTRequest):
    @staticmethod
    def type_identifier() -> str:
        return "pose_solver_extrinsic_clear"

    # noinspection PyTypeHints
    parsable_type: str = Field(default=type_identifier())


class PoseSolverExtrinsicSetRequest(MCTRequest):
    @staticmethod
    def type_identifier() -> str:
        return "pose_solver_extrinsic_set"

    # noinspection PyTypeHints
    parsable_type: str = Field(default=type_identifier())

    detector_label: str = Field()
    transform_to_reference: Matrix4x4 = Field()


class PoseSolverPosesGetRequest(MCTRequest):
    @staticmethod
    def type_identifier() -> str:
        return "mixer_pose_solver_poses_get"

    # noinspection PyTypeHints
    parsable_type: str = Field(default=type_identifier())


class PoseSolverPosesGetResponse(MCTResponse):
    @staticmethod
    def type_identifier() -> str:
        return "pose_solver_poses_get"

    # noinspection PyTypeHints
    parsable_type: str = Field(default=type_identifier())

    detector_poses: list[Pose]
    target_poses: list[Pose]


class PoseSolverTargetAddRequest(MCTRequest):
    @staticmethod
    def type_identifier() -> str:
        return "pose_solver_add_target"

    # noinspection PyTypeHints
    parsable_type: str = Field(default=type_identifier())

    target: Target = Field()


class PoseSolverTargetAddResponse(MCTResponse):
    @staticmethod
    def type_identifier() -> str:
        return "pose_solver_add_target"

    # noinspection PyTypeHints
    parsable_type: str = Field(default=type_identifier())

    target_id: str = Field()


class PoseSolverTargetsClearRequest(MCTRequest):
    @staticmethod
    def type_identifier() -> str:
        return "pose_solver_target_clear"

    # noinspection PyTypeHints
    parsable_type: str = Field(default=type_identifier())


class PoseSolverTargetsSetRequest(MCTRequest):
    @staticmethod
    def type_identifier() -> str:
        return "pose_solver_set_targets"

    # noinspection PyTypeHints
    parsable_type: str = Field(default=type_identifier())

    targets: list[Target] = Field()


class MixerFrameGetRequest(MCTRequest):
    @staticmethod
    def type_identifier() -> str:
        return "mixer_frame_get"

    # noinspection PyTypeHints
    parsable_type: str = Field(default=type_identifier())


class MixerFrameGetResponse(MCTResponse):
    @staticmethod
    def type_identifier() -> str:
        return "mixer_frame_get"

    # noinspection PyTypeHints
    parsable_type: str = Field(default=type_identifier())

    frame: MixerFrame = Field()


class MixerIntrinsicUpdateRequest(MCTRequest):
    @staticmethod
    def type_identifier() -> str:
        return "mixer_intrinsic_update_parameters"

    parsable_type: str = Field(default=type_identifier())

    detector_label: str = Field()
    intrinsic_parameters: IntrinsicParameters = Field()


class MixerQueryRequest(MCTRequest):
    @staticmethod
    def type_identifier() -> str:
        return "mixer_query"

    # noinspection PyTypeHints
    parsable_type: str = Field(default=type_identifier())


class MixerQueryResponse(MCTResponse):
    @staticmethod
    def type_identifier() -> str:
        return "mixer_query"

    # noinspection PyTypeHints
    parsable_type: str = Field(default=type_identifier())

    mixer_status: str = Field()


class MixerStartRequest(MCTRequest):
    @staticmethod
    def type_identifier() -> str:
        return "mixer_start"

    # noinspection PyTypeHints
    parsable_type: str = Field(default=type_identifier())


class MixerStopRequest(MCTRequest):
    @staticmethod
    def type_identifier() -> str:
        return "mixer_stop"

    # noinspection PyTypeHints
    parsable_type: str = Field(default=type_identifier())


MIXER_RESPONSE_TYPES: list[type[MCTResponse]] = [
    DequeueStatusMessagesResponse,
    EmptyResponse,
    ErrorResponse,
    ExtrinsicCalibrationCalculateResponse,
    ExtrinsicCalibrationImageAddResponse,
    ExtrinsicCalibrationImageGetResponse,
    ExtrinsicCalibrationMetadataListResponse,
    ExtrinsicCalibrationResultGetResponse,
    ExtrinsicCalibrationResultGetActiveResponse,
    PoseSolverPosesGetResponse,
    PoseSolverTargetAddResponse,
    MixerFrameGetResponse,
    MixerQueryResponse,
    TimestampGetResponse]
