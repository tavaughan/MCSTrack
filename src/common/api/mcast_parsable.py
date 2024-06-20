from src.common.exceptions import ParsingError
import abc
from pydantic import ValidationError
from typing import TypeVar


ParsableDynamic = TypeVar('ParsableDynamic', bound='MCastParsable')


class MCastParsable(abc.ABC):

    @staticmethod
    @abc.abstractmethod
    def parsable_type_identifier() -> str:
        pass

    @staticmethod
    def parse_dynamic_series_list(
        parsable_series_dict: dict,
        supported_types: list[type[ParsableDynamic]]
    ) -> list[ParsableDynamic]:
        if "series" not in parsable_series_dict or not isinstance(parsable_series_dict["series"], list):
            message: str = "parsable_series_dict did not contain field series. Input is improperly formatted."
            raise ParsingError(message)

        output_series: list[ParsableDynamic] = list()
        for parsable_dict in parsable_series_dict["series"]:
            if not isinstance(parsable_dict, dict):
                message: str = "series contained a non-dict element. Input is improperly formatted."
                raise ParsingError(message)
            output_series.append(MCastParsable.parse_dynamic_single(
                parsable_dict=parsable_dict,
                supported_types=supported_types))

        return output_series

    @staticmethod
    def parse_dynamic_single(
        parsable_dict: dict,
        supported_types: list[type[ParsableDynamic]]
    ) -> ParsableDynamic:
        if "parsable_type" not in parsable_dict or not isinstance(parsable_dict["parsable_type"], str):
            message: str = "parsable_dict did not contain parsable_type. Input is improperly formatted."
            raise ParsingError(message) from None

        for supported_type in supported_types:
            if parsable_dict["parsable_type"] == supported_type.parsable_type_identifier():
                request: ParsableDynamic
                try:
                    request = supported_type(**parsable_dict)
                except ValidationError as e:
                    raise ParsingError(f"A request of type {supported_type} was ill-formed: {str(e)}") from None
                return request

        message: str = "parsable_type did not match any expected value. Input is improperly formatted."
        raise ParsingError(message)
