from src.common import MCTRequest
from src.common.structures import KeyValueSimpleAny
from pydantic import Field


class SetCameraParametersRequest(MCTRequest):

    @staticmethod
    def parsable_type_identifier() -> str:
        return "set_capture_properties"

    parsable_type: str = Field(default=parsable_type_identifier(), const=True)
    parameters: list[KeyValueSimpleAny] = Field()
