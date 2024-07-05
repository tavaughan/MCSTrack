from .abstract_camera_interface import AbstractCameraInterface
from ..api import \
    GetCameraParametersResponse, \
    SetCameraParametersRequest
from ..exceptions import UpdateCaptureError
from src.common import \
    EmptyResponse, \
    ErrorResponse, \
    get_kwarg, \
    MCTResponse
from src.common.structures.capture_status import CaptureStatus
import datetime
import logging
import numpy
from picamera2 import Picamera2
from picamera2.configuration import CameraConfiguration
from picamera2.controls import Controls
from typing import Final


logger = logging.getLogger(__name__)

_PICAMERA2_RANGE_MINIMUM_INDEX: Final[int] = 0
_PICAMERA2_RANGE_MAXIMUM_INDEX: Final[int] = 1
_PICAMERA2_RANGE_DEFAULT_INDEX: Final[int] = 2
_MICROSECONDS_PER_SECOND: Final[int] = 1000000


class PiCamera(AbstractCameraInterface):

    _camera: Picamera2
    _captured_image: numpy.ndarray | None
    _captured_timestamp_utc: datetime.datetime
    _capture_status: CaptureStatus  # internal bookkeeping

    def __init__(self):
        self._captured_image = None
        self._captured_timestamp_utc = datetime.datetime.min

        self._capture_status = CaptureStatus()
        self._capture_status.status = CaptureStatus.Status.STOPPED

        self._camera = Picamera2()
        self._camera.configure("video")

    def __del__(self):
        if self._captured_image is not None:
            self._captured_image = None

    def internal_update_capture(self) -> None:
        self._captured_image = self._camera.capture_array()

        if self._captured_image is None:
            message: str = "Failed to grab frame."
            self._status.capture_errors.append(message)
            self._capture_status.status = CaptureStatus.Status.FAILURE
            raise UpdateCaptureError(severity="error", message=message)

        self._captured_timestamp_utc = datetime.datetime.utcnow()

    # noinspection DuplicatedCode
    def set_capture_properties(self, **kwargs) -> EmptyResponse:
        """
        :key request: SetCapturePropertiesRequest
        """

        request: SetCameraParametersRequest = get_kwarg(
            kwargs=kwargs,
            key="request",
            arg_type=SetCameraParametersRequest)

        if self._captured_image is not None:

            if request.fps is not None:
                controls: Controls = self._camera.controls
                self._camera.stop()
                self._camera.video_configuration.controls.FrameRate = request.fps
                self._camera.configure("video")
                self._camera.start()
                self._camera.set_controls(controls)

            if request.resolution_x_px is not None and request.resolution_y_px is not None:
                controls: Controls = self._camera.controls
                self._camera.stop()
                self._camera.video_configuration.size = (request.resolution_x_px, request.resolution_y_px)
                self._camera.configure("video")
                self._camera.start()
                self._camera.set_controls(controls)

            with self._camera.controls as controls:
                if request.auto_exposure is not None:
                    controls.AeEnable = request.auto_exposure
                # TODO: how to enforce values in gui be entered in the proper range?
                if request.exposure is not None:
                    controls.ExposureValue = request.exposure
                if request.brightness is not None:
                    controls.Brightness = request.brightness
                if request.contrast is not None:
                    controls.Contrast = request.contrast
                if request.sharpness is not None:
                    controls.Sharpness = request.sharpness

        return EmptyResponse()

    def get_capture_properties(self, **_kwargs) -> GetCameraParametersResponse | ErrorResponse:
        if self._captured_image is None:
            return ErrorResponse(
                message="The capture is not active, and properties cannot be retrieved.")
        else:
            configuration: CameraConfiguration = self._camera.video_configuration
            frame_duration_us = configuration.controls.FrameDurationLimits[_PICAMERA2_RANGE_MAXIMUM_INDEX]
            controls: dict = {
                # Custom settings shall override default values
                **{control[0]: control[1][_PICAMERA2_RANGE_DEFAULT_INDEX]
                   for control in self._camera.camera_controls.items()},
                **configuration.controls.make_dict(),
                **self._camera.controls.make_dict()}
            fps: float = int(1.0 / round(frame_duration_us)) * _MICROSECONDS_PER_SECOND
            ret = GetCameraParametersResponse(
                resolution_x_px=int(configuration.size[0]),
                resolution_y_px=int(configuration.size[1]),
                fps=fps,
                brightness=controls["Brightness"],
                auto_exposure=bool(controls["AeEnable"]),
                exposure=int(controls["ExposureValue"]),
                contrast=controls["Contrast"],
                sharpness=controls["Sharpness"])
            return ret

    def start_capture(self, **kwargs) -> MCTResponse:
        self._camera.start()
        self._captured_image = self._camera.capture_array()
        self._capture_status.status = CaptureStatus.Status.RUNNING
        return EmptyResponse()

    def stop_capture(self, **kwargs) -> MCTResponse:
        if self._captured_image is not None:
            self._captured_image = None
        self._capture_status.status = CaptureStatus.Status.STOPPED
        self._camera.stop()
        return EmptyResponse()
