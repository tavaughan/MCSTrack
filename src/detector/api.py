from src.common import \
    MCTRequest, \
    MCTResponse
from src.common.structures import \
    CaptureFormat, \
    DetectionParameters, \
    DetectorResolution, \
    IntrinsicCalibration, \
    IntrinsicParameters, \
    ImageResolution, \
    KeyValueMetaAny, \
    KeyValueSimpleAny, \
    MarkerSnapshot
from .structures import \
    CalibrationImageMetadata, \
    CalibrationImageState, \
    CalibrationResultMetadata, \
    CalibrationResultState
from pydantic import Field


class CalibrationCalculateRequest(MCTRequest):
    @staticmethod
    def parsable_type_identifier() -> str:
        return "detector_calibration_calculate"

    parsable_type: str = Field(default=parsable_type_identifier(), const=True)
    detector_serial_identifier: str = Field()
    image_resolution: ImageResolution = Field()


class CalibrationCalculateResponse(MCTResponse):
    @staticmethod
    def parsable_type_identifier() -> str:
        return "detector_calibration_calculate"

    parsable_type: str = Field(default=parsable_type_identifier(), const=True)
    result_identifier: str = Field()
    intrinsic_calibration: IntrinsicCalibration = Field()


class CalibrationDeleteStagedRequest(MCTRequest):
    @staticmethod
    def parsable_type_identifier() -> str:
        return "detector_calibration_delete_staged"

    parsable_type: str = Field(default=parsable_type_identifier(), const=True)


class CalibrationImageAddRequest(MCTRequest):
    """
    Calibrator will infer resolution from the image itself, and associate its own label to the data.
    """
    @staticmethod
    def parsable_type_identifier() -> str:
        return "detector_calibration_image_add"

    parsable_type: str = Field(default=parsable_type_identifier(), const=True)

    detector_serial_identifier: str = Field()
    format: CaptureFormat = Field()
    image_base64: str = Field()


class CalibrationImageAddResponse(MCTResponse):
    @staticmethod
    def parsable_type_identifier() -> str:
        return "detector_calibration_image_add"

    parsable_type: str = Field(default=parsable_type_identifier(), const=True)
    image_identifier: str = Field()


class CalibrationImageGetRequest(MCTRequest):
    @staticmethod
    def parsable_type_identifier() -> str:
        return "detector_calibration_image_get"

    parsable_type: str = Field(default=parsable_type_identifier(), const=True)
    image_identifier: str = Field()


class CalibrationImageGetResponse(MCTResponse):
    @staticmethod
    def parsable_type_identifier() -> str:
        return "detector_calibration_image_get"

    parsable_type: str = Field(default=parsable_type_identifier(), const=True)
    image_base64: str = Field()


class CalibrationImageMetadataListRequest(MCTRequest):
    @staticmethod
    def parsable_type_identifier() -> str:
        return "detector_calibration_image_metadata_list"

    parsable_type: str = Field(default=parsable_type_identifier(), const=True)
    detector_serial_identifier: str = Field()
    image_resolution: ImageResolution = Field()


class CalibrationImageMetadataListResponse(MCTResponse):
    @staticmethod
    def parsable_type_identifier() -> str:
        return "detector_calibration_image_metadata_list"

    parsable_type: str = Field(default=parsable_type_identifier(), const=True)
    metadata_list: list[CalibrationImageMetadata] = Field(default_factory=list)


class CalibrationImageMetadataUpdateRequest(MCTRequest):
    @staticmethod
    def parsable_type_identifier() -> str:
        return "detector_calibration_image_metadata_update"

    parsable_type: str = Field(default=parsable_type_identifier(), const=True)
    image_identifier: str = Field()
    image_state: CalibrationImageState = Field()
    image_label: str = Field()


class CalibrationResolutionListRequest(MCTRequest):
    @staticmethod
    def parsable_type_identifier() -> str:
        return "detector_calibration_detector_resolutions_list"

    parsable_type: str = Field(default=parsable_type_identifier(), const=True)


class CalibrationResolutionListResponse(MCTResponse):
    @staticmethod
    def parsable_type_identifier() -> str:
        return "detector_calibration_detector_resolutions_list"

    parsable_type: str = Field(default=parsable_type_identifier(), const=True)
    detector_resolutions: list[DetectorResolution] = Field()


class CalibrationResultGetRequest(MCTRequest):
    @staticmethod
    def parsable_type_identifier() -> str:
        return "detector_calibration_result_get"

    parsable_type: str = Field(default=parsable_type_identifier(), const=True)
    result_identifier: str = Field()


class CalibrationResultGetResponse(MCTResponse):
    @staticmethod
    def parsable_type_identifier() -> str:
        return "detector_calibration_result_get"

    parsable_type: str = Field(default=parsable_type_identifier(), const=True)
    intrinsic_calibration: IntrinsicCalibration = Field()


class CalibrationResultMetadataListRequest(MCTRequest):
    @staticmethod
    def parsable_type_identifier() -> str:
        return "detector_calibration_result_metadata_list"

    parsable_type: str = Field(default=parsable_type_identifier(), const=True)
    detector_serial_identifier: str = Field()
    image_resolution: ImageResolution = Field()


class CalibrationResultMetadataListResponse(MCTResponse):
    @staticmethod
    def parsable_type_identifier() -> str:
        return "detector_calibration_result_metadata_list"

    parsable_type: str = Field(default=parsable_type_identifier(), const=True)
    metadata_list: list[CalibrationResultMetadata] = Field(default_factory=list)


class CalibrationResultMetadataUpdateRequest(MCTRequest):
    @staticmethod
    def parsable_type_identifier() -> str:
        return "detector_calibration_result_metadata_update"

    parsable_type: str = Field(default=parsable_type_identifier(), const=True)
    result_identifier: str = Field()
    result_state: CalibrationResultState = Field()


class CameraImageGetRequest(MCTRequest):
    @staticmethod
    def parsable_type_identifier() -> str:
        return "detector_camera_image_get"

    parsable_type: str = Field(default=parsable_type_identifier(), const=True)
    format: CaptureFormat = Field()


class CameraImageGetResponse(MCTResponse):

    @staticmethod
    def parsable_type_identifier() -> str:
        return "detector_camera_image_get"

    parsable_type: str = Field(default=parsable_type_identifier(), const=True)
    format: CaptureFormat = Field()
    image_base64: str = Field()


class CameraParametersGetRequest(MCTRequest):
    @staticmethod
    def parsable_type_identifier() -> str:
        return "detector_camera_parameters_get"

    parsable_type: str = Field(default=parsable_type_identifier(), const=True)


class CameraParametersGetResponse(MCTResponse):
    @staticmethod
    def parsable_type_identifier() -> str:
        return "detector_camera_parameters_get"

    parsable_type: str = Field(default=parsable_type_identifier(), const=True)
    parameters: list[KeyValueMetaAny] = Field()


class CameraParametersSetRequest(MCTRequest):

    @staticmethod
    def parsable_type_identifier() -> str:
        return "detector_camera_parameters_set"

    parsable_type: str = Field(default=parsable_type_identifier(), const=True)
    parameters: list[KeyValueSimpleAny] = Field()


class DetectorFrameGetRequest(MCTRequest):
    @staticmethod
    def parsable_type_identifier() -> str:
        return "detector_frame_get"

    parsable_type: str = Field(default=parsable_type_identifier(), const=True)
    include_detected: bool = Field(default=True)
    include_rejected: bool = Field(default=True)


class DetectorFrameGetResponse(MCTResponse):
    @staticmethod
    def parsable_type_identifier() -> str:
        return "detector_frame_get"

    parsable_type: str = Field(default=parsable_type_identifier(), const=True)
    detected_marker_snapshots: list[MarkerSnapshot] | None = Field()
    rejected_marker_snapshots: list[MarkerSnapshot] | None = Field()


class DetectorStartRequest(MCTRequest):
    @staticmethod
    def parsable_type_identifier() -> str:
        return "detector_start"

    parsable_type: str = Field(default=parsable_type_identifier(), const=True)


class DetectorStopRequest(MCTRequest):
    @staticmethod
    def parsable_type_identifier() -> str:
        return "detector_stop"

    parsable_type: str = Field(default=parsable_type_identifier(), const=True)


class MarkerParametersGetRequest(MCTRequest):
    @staticmethod
    def parsable_type_identifier() -> str:
        return "detector_marker_parameters_get"

    parsable_type: str = Field(default=parsable_type_identifier(), const=True)


class MarkerParametersGetResponse(MCTResponse):
    @staticmethod
    def parsable_type_identifier() -> str:
        return "detector_marker_parameters_get"

    parsable_type: str = Field(default=parsable_type_identifier(), const=True)
    parameters: DetectionParameters = Field(default_factory=DetectionParameters)


class MarkerParametersSetRequest(MCTRequest):
    @staticmethod
    def parsable_type_identifier() -> str:
        return "detector_marker_parameters_set"

    parsable_type: str = Field(default=parsable_type_identifier(), const=True)
    parameters: DetectionParameters = Field(default_factory=DetectionParameters)
