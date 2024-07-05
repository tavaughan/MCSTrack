from ..structures.mct_parsable import MCTParsable
import abc
from pydantic import BaseModel


class MCTRequest(BaseModel, MCTParsable, abc.ABC):
    parsable_type: str
