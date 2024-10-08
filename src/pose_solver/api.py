from src.common import \
    MCTRequest, \
    MCTResponse
from src.common.structures import \
    DetectorFrame, \
    IntrinsicParameters, \
    Matrix4x4, \
    Pose, \
    TargetBoard, \
    TargetMarker
from pydantic import Field
from typing import Union


class PoseSolverAddDetectorFrameRequest(MCTRequest):
    @staticmethod
    def parsable_type_identifier() -> str:
        return "add_marker_corners"

    parsable_type: str = Field(default=parsable_type_identifier(), const=True)
    detector_label: str = Field()
    detector_frame: DetectorFrame = Field()


class PoseSolverAddTargetMarkerRequest(MCTRequest):
    @staticmethod
    def parsable_type_identifier() -> str:
        return "add_target_marker"

    parsable_type: str = Field(default=parsable_type_identifier(), const=True)
    target: TargetMarker = Field()


class PoseSolverAddTargetBoardRequest(MCTRequest):
    @staticmethod
    def parsable_type_identifier() -> str:
        return "add_target_board"

    parsable_type: str = Field(default=parsable_type_identifier(), const=True)
    target: TargetBoard = Field()


class PoseSolverAddTargetResponse(MCTResponse):
    @staticmethod
    def parsable_type_identifier() -> str:
        return "add_marker_corners"

    parsable_type: str = Field(default=parsable_type_identifier(), const=True)
    target_id: str = Field()


class PoseSolverGetPosesRequest(MCTRequest):
    @staticmethod
    def parsable_type_identifier() -> str:
        return "get_poses"

    parsable_type: str = Field(default=parsable_type_identifier(), const=True)


class PoseSolverGetPosesResponse(MCTResponse):
    @staticmethod
    def parsable_type_identifier() -> str:
        return "get_poses"

    parsable_type: str = Field(default=parsable_type_identifier(), const=True)
    detector_poses: list[Pose]
    target_poses: list[Pose]


class PoseSolverSetExtrinsicRequest(MCTRequest):
    @staticmethod
    def parsable_type_identifier() -> str:
        return "set_extrinsic_parameters"

    parsable_type: str = Field(default=parsable_type_identifier(), const=True)
    detector_label: str = Field()
    transform_to_reference: Matrix4x4 = Field()


class PoseSolverSetIntrinsicRequest(MCTRequest):
    @staticmethod
    def parsable_type_identifier() -> str:
        return "set_intrinsic_parameters"

    parsable_type: str = Field(default=parsable_type_identifier(), const=True)
    detector_label: str = Field()
    intrinsic_parameters: IntrinsicParameters = Field()


class PoseSolverSetReferenceRequest(MCTRequest):
    @staticmethod
    def parsable_type_identifier() -> str:
        return "set_reference_marker"

    parsable_type: str = Field(default=parsable_type_identifier(), const=True)
    marker_id: int = Field()
    marker_diameter: float = Field()


class PoseSolverSetTargetsRequest(MCTRequest):
    @staticmethod
    def parsable_type_identifier() -> str:
        return "set_targets"

    parsable_type: str = Field(default=parsable_type_identifier(), const=True)
    targets: list[Union[TargetMarker, TargetBoard]] = Field()


class PoseSolverStartRequest(MCTRequest):
    @staticmethod
    def parsable_type_identifier() -> str:
        return "start_pose_solver"

    parsable_type: str = Field(default=parsable_type_identifier(), const=True)


class PoseSolverStopRequest(MCTRequest):
    @staticmethod
    def parsable_type_identifier() -> str:
        return "stop_pose_solver"

    parsable_type: str = Field(default=parsable_type_identifier(), const=True)
