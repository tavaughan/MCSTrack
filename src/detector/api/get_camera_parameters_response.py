from src.common import MCTResponse
from src.common.structures import KeyValueMetaAny
from pydantic import Field


class GetCameraParametersResponse(MCTResponse):
    @staticmethod
    def parsable_type_identifier() -> str:
        return "get_camera_parameters"

    parsable_type: str = Field(default=parsable_type_identifier(), const=True)
    parameters: list[KeyValueMetaAny] = Field()
