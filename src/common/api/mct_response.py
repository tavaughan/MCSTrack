from ..structures.mct_parsable import MCTParsable
import abc
from pydantic import BaseModel


class MCTResponse(BaseModel, MCTParsable, abc.ABC):
    parsable_type: str
