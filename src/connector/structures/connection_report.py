from pydantic import BaseModel, Field


class ConnectionReport(BaseModel):
    """
    Human-readable information that shall be shown to a user about a connection.
    """
    label: str = Field()
    role: str = Field()
    ip_address: str = Field()
    port: int = Field()
    status: str = Field()
