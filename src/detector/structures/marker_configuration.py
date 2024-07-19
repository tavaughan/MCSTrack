from pydantic import BaseModel, Field


class MarkerConfiguration(BaseModel):
    method: str = Field()
