from ..api import \
    CameraImageGetResponse, \
    CameraImageGetRequest, \
    CameraParametersGetResponse
from src.common import \
    EmptyResponse, \
    ErrorResponse, \
    get_kwarg, \
    MCTResponse
from src.common.structures.capture_status import CaptureStatus
import abc
import base64
import cv2
import datetime
import numpy


class AbstractCameraInterface(abc.ABC):

    _captured_image: numpy.ndarray | None
    _captured_timestamp_utc: datetime.datetime
    _capture_status: CaptureStatus  # internal bookkeeping

    def __del__(self):
        pass

    @abc.abstractmethod
    def internal_update_capture(self) -> None:
        pass

    @abc.abstractmethod
    def set_capture_properties(self, **kwargs) -> EmptyResponse | ErrorResponse:
        pass

    @abc.abstractmethod
    def get_capture_properties(self, **_kwargs) -> CameraParametersGetResponse | ErrorResponse:
        pass

    @abc.abstractmethod
    def start_capture(self, **kwargs) -> MCTResponse:
        pass

    @abc.abstractmethod
    def stop_capture(self, **kwargs) -> MCTResponse:
        pass

    def get_capture_image(self, **kwargs) -> CameraImageGetResponse | ErrorResponse:
        """
        :key request: GetCaptureImageRequest
        """

        request: CameraImageGetRequest = get_kwarg(
            kwargs=kwargs,
            key="request",
            arg_type=CameraImageGetRequest)

        encoded_frame: bool
        encoded_image_rgb_single_row: numpy.array
        encoded, encoded_image_rgb_single_row = cv2.imencode(request.format, self._captured_image)
        encoded_image_rgb_bytes: bytes = encoded_image_rgb_single_row.tobytes()
        encoded_image_rgb_base64 = base64.b64encode(encoded_image_rgb_bytes)
        return CameraImageGetResponse(
            format=request.format,
            image_base64=encoded_image_rgb_base64)
