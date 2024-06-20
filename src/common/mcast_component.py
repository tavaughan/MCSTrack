from .get_kwarg import get_kwarg
from .status_message_source import StatusMessageSource
from src.common.api import \
    DequeueStatusMessagesRequest, \
    DequeueStatusMessagesResponse, \
    ErrorResponse, \
    MCastParsable, \
    MCastRequest, \
    MCastRequestSeries, \
    MCastResponse, \
    MCastResponseSeries
from src.common.exceptions import ParsingError
from src.common.structures import \
    SeverityLabel, \
    StatusMessage
import abc
import datetime
from fastapi import WebSocket, WebSocketDisconnect
import logging
from typing import Callable, Optional, TypeVar

logger = logging.getLogger(__name__)


ParsableDynamicSingle = TypeVar('ParsableDynamicSingle', bound=MCastParsable)


class MCastComponent(abc.ABC):

    _status_message_source: StatusMessageSource

    def __init__(
        self,
        status_source_label: str,
        send_status_messages_to_logger: bool = False
    ):
        self._status_message_source = StatusMessageSource(
            source_label=status_source_label,
            send_to_logger=send_status_messages_to_logger)

    def add_status_message(
        self,
        severity: SeverityLabel,
        message: str,
        source_label: Optional[str] = None,
        timestamp_utc_iso8601: Optional[datetime.datetime | str] = None
    ):
        self._status_message_source.enqueue_status_message(
            source_label=source_label,
            severity=severity,
            message=message,
            timestamp_utc_iso8601=timestamp_utc_iso8601)

    def add_status_subscriber(
        self,
        client_identifier: str
    ):
        self._status_message_source.add_status_subscriber(subscriber_label=client_identifier)

    def parse_dynamic_series_list(
        self,
        parsable_series_dict: dict,
        supported_types: list[type[ParsableDynamicSingle]]
    ) -> list[ParsableDynamicSingle]:
        try:
            return MCastParsable.parse_dynamic_series_list(
                parsable_series_dict=parsable_series_dict,
                supported_types=supported_types)
        except ParsingError as e:
            self.add_status_message(
                severity="error",
                message=e.message)
            raise e

    def dequeue_status_messages(self, **kwargs) -> DequeueStatusMessagesResponse:
        """
        :key client_identifier: str
        """
        client_identifier: str = get_kwarg(
            kwargs=kwargs,
            key="client_identifier",
            arg_type=str)
        status_messages: list[StatusMessage] = self._status_message_source.pop_new_status_messages(
            subscriber_label=client_identifier)
        return DequeueStatusMessagesResponse(
            status_messages=status_messages)

    @abc.abstractmethod
    def supported_request_types(self) -> dict[type[MCastRequest], Callable[[dict], MCastResponse]]:
        """
        All subclasses are expected to implement this method, even if it is simply a call to super().
        :return:
            A mapping between request type and the function that is meant to handle it.
            The function shall generally accept a kwargs dict, and the kwargs dict may contain:
            - client_identifier: str unique to the component making the request
            - request: Of type derived from MCastRequest that contains information specific to the request
        """
        return {DequeueStatusMessagesRequest: self.dequeue_status_messages}

    async def websocket_handler(self, websocket: WebSocket) -> None:
        await websocket.accept()
        try:
            client_identifier: str = f"{websocket.client.host}:{websocket.client.port}"
            self.add_status_subscriber(client_identifier=client_identifier)
            request_series: MCastRequestSeries
            response_series: MCastResponseSeries
            while True:
                request_series_dict = await websocket.receive_json()
                try:
                    request_series_list: list[MCastRequest] = self.parse_dynamic_series_list(
                        parsable_series_dict=request_series_dict,
                        supported_types=list(self.supported_request_types().keys()))
                except ParsingError as e:
                    logger.exception(str(e))
                    await websocket.send_json(MCastResponseSeries(requests_parsed=False).dict())
                    continue
                response_series: MCastResponseSeries = self.websocket_handle_requests(
                    client_identifier=client_identifier,
                    request_series=MCastRequestSeries(series=request_series_list))
                await websocket.send_json(response_series.dict())
        except WebSocketDisconnect as e:
            print(f"DISCONNECTED: {str(e)}")
            logger.info(str(e))

    def websocket_handle_requests(
        self,
        client_identifier: str,
        request_series: MCastRequestSeries
    ) -> MCastResponseSeries:
        request_map: dict[type[MCastRequest], Callable] = self.supported_request_types()
        response_series: list[MCastResponse] = list()
        for request in request_series.series:
            # noinspection PyBroadException
            try:
                if type(request) in request_map:
                    response_function: Callable[[dict], MCastResponse] = request_map[type(request)]
                    response: MCastResponse = response_function(
                        client_identifier=client_identifier,
                        request=request)
                    response_series.append(response)
                else:
                    message: str = f"Received unimplemented parsable_type: {request.parsable_type}."
                    logger.error(message)
                    self.add_status_message(severity="error", message=message)
                    response_series.append(ErrorResponse(message=message))
            except Exception as e:
                message: str = f"Internal error. Failed to process request series."
                logger.error(message + " " + str(e))
                self.add_status_message(severity="error", message=message)
                response_series.append(ErrorResponse(message=message))
        return MCastResponseSeries(
            requests_parsed=True,
            series=response_series)
