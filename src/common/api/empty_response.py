from .mct_response import MCTResponse
from pydantic import Field


class EmptyResponse(MCTResponse):
    @staticmethod
    def parsable_type_identifier() -> str:
        return "empty"

    parsable_type = Field(default=parsable_type_identifier(), const=True)
