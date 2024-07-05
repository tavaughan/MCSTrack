from src.common import MCTRequest
from pydantic import Field


class GetCameraParametersRequest(MCTRequest):
    @staticmethod
    def parsable_type_identifier() -> str:
        return "get_camera_parameters"

    parsable_type: str = Field(default=parsable_type_identifier(), const=True)
