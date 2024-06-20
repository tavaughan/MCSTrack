from src.common.structures.component_role_label import ComponentRoleLabel
from ipaddress import IPv4Address
from pydantic import BaseModel, Field


class ComponentAddress(BaseModel):
    """
    Information used to establish a connection,
    there is nothing that should change here without a user's explicit input.
    """
    label: str = Field()
    role: ComponentRoleLabel = Field()
    ip_address: IPv4Address = Field()
    port: int = Field()
