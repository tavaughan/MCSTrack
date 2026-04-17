from src.common import \
    DequeueStatusMessagesResponse, \
    DetectorFrame, \
    EmptyResponse, \
    ErrorResponse, \
    ImageFormat, \
    ImageResolution, \
    IntrinsicCalibration, \
    IntrinsicCalibrator, \
    KeyValueMetaAny, \
    KeyValueSimpleAny, \
    MCTRequest, \
    MCTResponse, \
    TimestampGetResponse
from pydantic import Field, SerializeAsAny
from typing import Optional


# TODO: Remove
class CameraImageGetRequest(MCTRequest):
    """
    Deprecated - use DetectorFrameGetRequest instead
    """
    @staticmethod
    def type_identifier() -> str:
        return "detector_camera_image_get"

    parsable_type: str = Field(default=type_identifier())

    format: ImageFormat = Field()
    requested_resolution: ImageResolution | None = Field(default=None)


# TODO: Remove
class CameraImageGetResponse(MCTResponse):
    """
    Deprecated - use DetectorFrameGetResponse instead
    """
    @staticmethod
    def type_identifier() -> str:
        return "detector_camera_image_get"

    parsable_type: str = Field(default=type_identifier())

    format: ImageFormat = Field()
    image_base64: str = Field()
    original_resolution: ImageResolution = Field()


class DetectorFrameGetRequest(MCTRequest):
    @staticmethod
    def type_identifier() -> str:
        return "detector_frame_get"

    parsable_type: str = Field(default=type_identifier())

    include_detected: bool = Field(default=True)
    include_rejected: bool = Field(default=True)
    include_image: bool = Field(default=False)
    image_format: ImageFormat = Field(default=ImageFormat.FORMAT_JPG)
    image_resolution: ImageResolution | None = Field(default=None)  # None = that of the Detector's camera


class DetectorFrameGetResponse(MCTResponse):
    @staticmethod
    def type_identifier() -> str:
        return "detector_frame_get"

    parsable_type: str = Field(default=type_identifier())

    frame: DetectorFrame = Field()


class DetectorParametersGetRequest(MCTRequest):
    @staticmethod
    def type_identifier() -> str:
        return "detector_parameters_get"

    parsable_type: str = Field(default=type_identifier())


class DetectorParametersGetResponse(MCTResponse):
    @staticmethod
    def type_identifier() -> str:
        return "detector_parameters_get"

    parsable_type: str = Field(default=type_identifier())

    camera_resolution: ImageResolution = Field()
    camera_parameters: list[SerializeAsAny[KeyValueMetaAny]] = Field()
    annotator_parameters: list[SerializeAsAny[KeyValueMetaAny]] = Field()


class DetectorParametersSetRequest(MCTRequest):
    @staticmethod
    def type_identifier() -> str:
        return "detector_parameters_set"

    parsable_type: str = Field(default=type_identifier())

    camera_resolution: ImageResolution | None = Field(default=None)
    camera_parameters: list[SerializeAsAny[KeyValueSimpleAny]] | None = Field(default=None)
    annotator_parameters: list[SerializeAsAny[KeyValueSimpleAny]] | None = Field(default=None)


class DetectorParametersSetResponse(MCTResponse):
    @staticmethod
    def type_identifier() -> str:
        return "detector_parameters_set"

    parsable_type: str = Field(default=type_identifier())

    camera_resolution: ImageResolution = Field()
    camera_parameters: list[SerializeAsAny[KeyValueMetaAny]] = Field()
    annotator_parameters: list[SerializeAsAny[KeyValueMetaAny]] = Field()


class DetectorQueryRequest(MCTRequest):
    @staticmethod
    def type_identifier() -> str:
        return "detector_status"

    parsable_type: str = Field(default=type_identifier())


class DetectorQueryResponse(MCTResponse):
    @staticmethod
    def type_identifier() -> str:
        return "detector_status"

    parsable_type: str = Field(default=type_identifier())

    annotator_status: str = Field()
    camera_status: str = Field()


class DetectorStartRequest(MCTRequest):
    @staticmethod
    def type_identifier() -> str:
        return "detector_start"

    parsable_type: str = Field(default=type_identifier())


class DetectorStopRequest(MCTRequest):
    @staticmethod
    def type_identifier() -> str:
        return "detector_stop"

    parsable_type: str = Field(default=type_identifier())


class IntrinsicCalibrationCalculateRequest(MCTRequest):
    @staticmethod
    def type_identifier() -> str:
        return "detector_intrinsic_calibration_calculate"

    parsable_type: str = Field(default=type_identifier())

    image_resolution: ImageResolution = Field()


class IntrinsicCalibrationCalculateResponse(MCTResponse):
    @staticmethod
    def type_identifier() -> str:
        return "detector_intrinsic_calibration_calculate"

    parsable_type: str = Field(default=type_identifier())

    result_identifier: str = Field()
    intrinsic_calibration: IntrinsicCalibration = Field()


class IntrinsicCalibrationDeleteStagedRequest(MCTRequest):
    @staticmethod
    def type_identifier() -> str:
        return "detector_intrinsic_calibration_delete_staged"

    parsable_type: str = Field(default=type_identifier())


class IntrinsicCalibrationImageAddRequest(MCTRequest):
    @staticmethod
    def type_identifier() -> str:
        return "detector_intrinsic_calibration_image_add"

    parsable_type: str = Field(default=type_identifier())


class IntrinsicCalibrationImageAddResponse(MCTResponse):
    @staticmethod
    def type_identifier() -> str:
        return "detector_intrinsic_calibration_image_add"

    parsable_type: str = Field(default=type_identifier())

    image_identifier: str = Field()


class IntrinsicCalibrationImageGetRequest(MCTRequest):
    @staticmethod
    def type_identifier() -> str:
        return "detector_intrinsic_calibration_image_get"

    parsable_type: str = Field(default=type_identifier())

    image_identifier: str = Field()


class IntrinsicCalibrationImageGetResponse(MCTResponse):
    @staticmethod
    def type_identifier() -> str:
        return "detector_intrinsic_calibration_image_get"

    parsable_type: str = Field(default=type_identifier())

    image_base64: str = Field()


class IntrinsicCalibrationImageMetadataUpdateRequest(MCTRequest):
    @staticmethod
    def type_identifier() -> str:
        return "detector_intrinsic_calibration_image_metadata_update"

    parsable_type: str = Field(default=type_identifier())

    image_identifier: str = Field()
    image_state: IntrinsicCalibrator.ImageState = Field()
    image_label: str | None = Field(default=None)


class IntrinsicCalibrationMetadataListRequest(MCTRequest):
    @staticmethod
    def type_identifier() -> str:
        return "detector_intrinsic_calibration_metadata_list"

    parsable_type: str = Field(default=type_identifier())

    image_resolution: ImageResolution = Field()


class IntrinsicCalibrationMetadataListResponse(MCTResponse):
    @staticmethod
    def type_identifier() -> str:
        return "detector_intrinsic_calibration_metadata_list"

    parsable_type: str = Field(default=type_identifier())
    image_metadata_list: list[IntrinsicCalibrator.ImageMetadata] = Field(default_factory=list)
    result_metadata_list: list[IntrinsicCalibrator.ResultMetadata] = Field(default_factory=list)


class IntrinsicCalibrationResolutionListRequest(MCTRequest):
    @staticmethod
    def type_identifier() -> str:
        return "detector_intrinsic_calibration_detector_resolutions_list"

    parsable_type: str = Field(default=type_identifier())


class IntrinsicCalibrationResolutionListResponse(MCTResponse):
    @staticmethod
    def type_identifier() -> str:
        return "detector_intrinsic_calibration_detector_resolutions_list"

    parsable_type: str = Field(default=type_identifier())

    resolutions: list[ImageResolution] = Field()


class IntrinsicCalibrationResultGetRequest(MCTRequest):
    @staticmethod
    def type_identifier() -> str:
        return "detector_intrinsic_calibration_result_get"

    parsable_type: str = Field(default=type_identifier())

    result_identifier: str = Field()


class IntrinsicCalibrationResultGetResponse(MCTResponse):
    @staticmethod
    def type_identifier() -> str:
        return "detector_intrinsic_calibration_result_get"

    parsable_type: str = Field(default=type_identifier())

    intrinsic_calibration: IntrinsicCalibration = Field()


class IntrinsicCalibrationResultGetActiveRequest(MCTRequest):
    @staticmethod
    def type_identifier() -> str:
        return "detector_intrinsic_calibration_result_active_get"

    parsable_type: str = Field(default=type_identifier())


class IntrinsicCalibrationResultGetActiveResponse(MCTResponse):
    @staticmethod
    def type_identifier() -> str:
        return "detector_intrinsic_calibration_result_active_get"

    parsable_type: str = Field(default=type_identifier())

    intrinsic_calibration: Optional[IntrinsicCalibration] = Field()


class IntrinsicCalibrationResultMetadataUpdateRequest(MCTRequest):
    @staticmethod
    def type_identifier() -> str:
        return "detector_intrinsic_calibration_result_metadata_update"

    parsable_type: str = Field(default=type_identifier())

    result_identifier: str = Field()
    result_state: IntrinsicCalibrator.ResultState = Field()
    result_label: str | None = Field(default=None)


DETECTOR_RESPONSE_TYPES: list[type[MCTResponse]] = [
    CameraImageGetResponse,
    DequeueStatusMessagesResponse,
    DetectorFrameGetResponse,
    DetectorParametersGetResponse,
    DetectorParametersSetResponse,
    DetectorQueryResponse,
    EmptyResponse,
    ErrorResponse,
    IntrinsicCalibrationCalculateResponse,
    IntrinsicCalibrationImageAddResponse,
    IntrinsicCalibrationImageGetResponse,
    IntrinsicCalibrationMetadataListResponse,
    IntrinsicCalibrationResolutionListResponse,
    IntrinsicCalibrationResultGetResponse,
    IntrinsicCalibrationResultGetActiveResponse,
    TimestampGetResponse]
