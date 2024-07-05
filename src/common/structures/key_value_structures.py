import abc
from pydantic import BaseModel, Field
from typing import Union


class KeyValueSimpleAbstract(BaseModel, abc.ABC):
    """
    Abstract class to represent a key-value pair.
    Intended use: Setting parameter over network, serialization through JSON.
    """
    parsable_type: str
    key: str = Field()


class KeyValueSimpleBool(KeyValueSimpleAbstract):
    parsable_type: str = Field(default="bool", const=True)
    value: bool = Field()


class KeyValueSimpleFloat(KeyValueSimpleAbstract):
    parsable_type: str = Field(default="float", const=True)
    value: float = Field()


class KeyValueSimpleInt(KeyValueSimpleAbstract):
    parsable_type: str = Field(default="int", const=True)
    value: int = Field()


class KeyValueSimpleString(KeyValueSimpleAbstract):
    parsable_type: str = Field(default="str", const=True)
    value: str = Field()


class KeyValueMetaAbstract(BaseModel, abc.ABC):
    """
    Abstract class to represent a key-value pair, and additional data about the datum (range, description, etc)
    Intended use: Getting parameter over network, serialization through JSON.
    """
    parsable_type: str
    key: str = Field()


class KeyValueMetaBool(KeyValueMetaAbstract):
    parsable_type: str = Field(default="bool", const=True)
    value: bool = Field()


class KeyValueMetaEnum(KeyValueMetaAbstract):
    parsable_type: str = Field(default="enum", const=True)
    value: str = Field()
    allowable_values: list[str] = Field(default_factory=list)


class KeyValueMetaFloat(KeyValueMetaAbstract):
    parsable_type: str = Field(default="float", const=True)
    value: float = Field()
    range_minimum: float = Field()
    range_maximum: float = Field()
    range_step: float = Field(default=1.0)


class KeyValueMetaInt(KeyValueMetaAbstract):
    parsable_type: str = Field(default="int", const=True)
    value: int = Field()
    range_minimum: int = Field()
    range_maximum: int = Field()
    range_step: int = Field(default=1)


# pydantic doesn't appear to handle very well typing's (TypeA, TypeB, ...) notation of a union
KeyValueSimpleAny = Union[
    KeyValueSimpleBool,
    KeyValueSimpleString,
    KeyValueSimpleFloat,
    KeyValueSimpleInt]
KeyValueMetaAny = Union[
    KeyValueMetaBool,
    KeyValueMetaEnum,
    KeyValueMetaFloat,
    KeyValueMetaInt]
