from ..exceptions import MCTDetectorRuntimeError
from ..interfaces import AbstractCameraInterface
from src.common import \
    StandardResolutions
from src.common.structures import \
    CaptureStatus, \
    ImageResolution, \
    KeyValueSimpleAbstract, \
    KeyValueSimpleAny, \
    KeyValueSimpleBool, \
    KeyValueSimpleString, \
    KeyValueSimpleFloat, \
    KeyValueSimpleInt, \
    KeyValueMetaAbstract, \
    KeyValueMetaBool, \
    KeyValueMetaEnum, \
    KeyValueMetaFloat, \
    KeyValueMetaInt
import cv2
import datetime
import logging
import os
from typing import Final


logger = logging.getLogger(__name__)


_CAMERA_RESOLUTION_KEY: Final[str] = "Resolution"
_CAMERA_RESOLUTION_DEFAULT: Final[str] = "640x480"
# This list is by no means exhaustive, but it should probably
# capture a reasonable cross-section of commonly-used camera image resolutions.
# Ideally we can query the camera/driver for supported resolutions and use that instead of this list.
_CAMERA_RESOLUTION_ALLOWABLE: Final[list[str]] = [str(resolution) for resolution in StandardResolutions.as_list()]
_CAMERA_FPS_KEY: Final[str] = "FramesPerSecond"
_CAMERA_FPS_DEFAULT: Final[float] = 30.0
_CAMERA_FPS_RANGE_MINIMUM: Final[float] = 1.0
_CAMERA_FPS_RANGE_MAXIMUM: Final[float] = 500.0
_CAMERA_FPS_RANGE_STEP: Final[float] = 1.0
_CAMERA_AUTO_EXPOSURE_KEY: Final[str] = "AutoExposure"
_CAMERA_AUTO_EXPOSURE_DEFAULT: Final[bool] = False
_CAMERA_EXPOSURE_KEY: Final[str] = "Exposure"
# Note the different value ranges depending on operating system/backend
_CAMERA_EXPOSURE_WINDOWS_DEFAULT: Final[int] = -6
_CAMERA_EXPOSURE_WINDOWS_RANGE_MINIMUM: Final[int] = -13
_CAMERA_EXPOSURE_WINDOWS_RANGE_MAXIMUM: Final[int] = 0
_CAMERA_EXPOSURE_UNIX_DEFAULT: Final[int] = 33
_CAMERA_EXPOSURE_UNIX_RANGE_MAXIMUM: Final[int] = 1000000
_CAMERA_EXPOSURE_UNIX_RANGE_MINIMUM: Final[int] = 1
_CAMERA_BRIGHTNESS_KEY: Final[str] = "Brightness"
_CAMERA_BRIGHTNESS_DEFAULT: Final[int] = 0
_CAMERA_BRIGHTNESS_RANGE_MINIMUM: Final[int] = -64
_CAMERA_BRIGHTNESS_RANGE_MAXIMUM: Final[int] = 64
_CAMERA_CONTRAST_KEY: Final[str] = "Contrast"
_CAMERA_CONTRAST_DEFAULT: Final[int] = 5
_CAMERA_CONTRAST_RANGE_MINIMUM: Final[int] = 0
_CAMERA_CONTRAST_RANGE_MAXIMUM: Final[int] = 95
_CAMERA_SHARPNESS_KEY: Final[str] = "Sharpness"
_CAMERA_SHARPNESS_DEFAULT: Final[int] = 2
_CAMERA_SHARPNESS_RANGE_MINIMUM: Final[int] = 0
_CAMERA_SHARPNESS_RANGE_MAXIMUM: Final[int] = 100
_CAMERA_GAMMA_KEY: Final[str] = "Gamma"
_CAMERA_GAMMA_DEFAULT: Final[int] = 120
_CAMERA_GAMMA_RANGE_MINIMUM: Final[int] = 80
_CAMERA_GAMMA_RANGE_MAXIMUM: Final[int] = 300


class USBWebcamWithOpenCV(AbstractCameraInterface):

    _capture: cv2.VideoCapture | None
    _captured_timestamp_utc: datetime.datetime
    _capture_device_id: str | int
    _capture_status: CaptureStatus  # internal bookkeeping

    def __init__(self, _capture_device_id):
        self._capture = None
        self._captured_image = None
        self._captured_timestamp_utc = datetime.datetime.min
        self._capture_device_id = _capture_device_id

        self._capture_status = CaptureStatus()
        self._capture_status.status = CaptureStatus.Status.STOPPED

    def __del__(self):
        if self._capture is not None:
            self._capture.release()

    @staticmethod
    def _detect_os_and_open_video(capture_device_id):
        if os.name == "nt":
            return cv2.VideoCapture(capture_device_id, cv2.CAP_DSHOW)
        elif os.name == "posix":
            return cv2.VideoCapture(capture_device_id, cv2.CAP_V4L2)
        else:
            raise RuntimeError(f"The current platform ({os.name}) is not supported.")

    def internal_update_capture(self) -> None:
        grabbed_frame: bool
        grabbed_frame = self._capture.grab()
        if not grabbed_frame:
            message: str = "Failed to grab frame."
            self._capture_status.errors.append(message)
            self._capture_status.status = CaptureStatus.Status.FAILURE
            raise MCTDetectorRuntimeError(message=message)

        retrieved_frame: bool
        retrieved_frame, self._captured_image = self._capture.retrieve()
        if not retrieved_frame:
            message: str = "Failed to retrieve frame."
            logger.error(message)
            self._capture_status.errors.append(message)
            self._capture_status.status = CaptureStatus.Status.FAILURE
            raise MCTDetectorRuntimeError(message=message)

        self._captured_timestamp_utc = datetime.datetime.utcnow()

    # noinspection DuplicatedCode
    def set_capture_properties(self, parameters: list[KeyValueSimpleAny]) -> None:

        if self._capture is None:
            raise MCTDetectorRuntimeError(message="Capture is None.")

        mismatched_keys: list[str] = list()

        key_value: KeyValueSimpleAbstract
        for key_value in parameters:
            if key_value.key == _CAMERA_RESOLUTION_KEY:
                if not isinstance(key_value, KeyValueSimpleString):
                    mismatched_keys.append(key_value.key)
                    continue
                try:
                    resolution: ImageResolution = ImageResolution.from_str(key_value.value)
                    self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, float(resolution.x_px))
                    self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, float(resolution.y_px))
                except ValueError:
                    mismatched_keys.append(key_value.key)
                    continue
            elif key_value.key == _CAMERA_FPS_KEY:
                if not isinstance(key_value, KeyValueSimpleFloat):
                    mismatched_keys.append(key_value.key)
                    continue
                self._capture.set(cv2.CAP_PROP_FPS, key_value.value)
            elif key_value.key == _CAMERA_AUTO_EXPOSURE_KEY:
                if not isinstance(key_value, KeyValueSimpleBool):
                    mismatched_keys.append(key_value.key)
                    continue
                self._capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, float(key_value.value))
            elif key_value.key == _CAMERA_EXPOSURE_KEY:
                if not isinstance(key_value, KeyValueSimpleInt):
                    mismatched_keys.append(key_value.key)
                    continue
                self._capture.set(cv2.CAP_PROP_EXPOSURE, float(key_value.value))
            elif key_value.key == _CAMERA_BRIGHTNESS_KEY:
                if not isinstance(key_value, KeyValueSimpleInt):
                    mismatched_keys.append(key_value.key)
                    continue
                self._capture.set(cv2.CAP_PROP_BRIGHTNESS, float(key_value.value))
            elif key_value.key == _CAMERA_CONTRAST_KEY:
                if not isinstance(key_value, KeyValueSimpleInt):
                    mismatched_keys.append(key_value.key)
                    continue
                self._capture.set(cv2.CAP_PROP_CONTRAST, float(key_value.value))
            elif key_value.key == _CAMERA_SHARPNESS_KEY:
                if not isinstance(key_value, KeyValueSimpleInt):
                    mismatched_keys.append(key_value.key)
                    continue
                self._capture.set(cv2.CAP_PROP_SHARPNESS, float(key_value.value))
            elif key_value.key == _CAMERA_GAMMA_KEY:
                if not isinstance(key_value, KeyValueSimpleInt):
                    mismatched_keys.append(key_value.key)
                    continue
                self._capture.set(cv2.CAP_PROP_GAMMA, float(key_value.value))
            else:
                mismatched_keys.append(key_value.key)

        if len(mismatched_keys) > 0:
            raise MCTDetectorRuntimeError(
                message=f"The following parameters could not be applied due to key mismatch: {str(mismatched_keys)}")

    def get_capture_properties(self) -> list[KeyValueMetaAbstract]:
        if self._capture is None:
            raise MCTDetectorRuntimeError(message="The capture is not active, and properties cannot be retrieved.")

        return_value: list[KeyValueMetaAbstract] = list()

        resolution_str: str = \
            f"{int(self._capture.get(cv2.CAP_PROP_FRAME_WIDTH))}x" + \
            f"{int(self._capture.get(cv2.CAP_PROP_FRAME_HEIGHT))}"
        return_value.append(KeyValueMetaEnum(
            key=_CAMERA_RESOLUTION_KEY,
            value=resolution_str,
            allowable_values=_CAMERA_RESOLUTION_ALLOWABLE))

        return_value.append(KeyValueMetaFloat(
            key=_CAMERA_FPS_KEY,
            value=self._capture.get(cv2.CAP_PROP_FPS),
            range_minimum=_CAMERA_FPS_RANGE_MINIMUM,
            range_maximum=_CAMERA_FPS_RANGE_MAXIMUM,
            range_step=_CAMERA_FPS_RANGE_STEP))

        return_value.append(KeyValueMetaBool(
            key=_CAMERA_AUTO_EXPOSURE_KEY,
            value=bool(round(self._capture.get(cv2.CAP_PROP_AUTO_EXPOSURE)))))

        # exposure implementations vary by operating system
        if os.name == "nt":  # Windows
            return_value.append(KeyValueMetaInt(
                key=_CAMERA_EXPOSURE_KEY,
                value=int(round(self._capture.get(cv2.CAP_PROP_EXPOSURE))),
                range_minimum=_CAMERA_EXPOSURE_WINDOWS_RANGE_MINIMUM,
                range_maximum=_CAMERA_EXPOSURE_WINDOWS_RANGE_MAXIMUM))
        elif os.name == "posix":  # Unix
            return_value.append(KeyValueMetaInt(
                key=_CAMERA_EXPOSURE_KEY,
                value=int(round(self._capture.get(cv2.CAP_PROP_EXPOSURE))),
                range_minimum=_CAMERA_EXPOSURE_UNIX_RANGE_MINIMUM,
                range_maximum=_CAMERA_EXPOSURE_UNIX_RANGE_MAXIMUM))

        return_value.append(KeyValueMetaInt(
            key=_CAMERA_BRIGHTNESS_KEY,
            value=int(round(self._capture.get(cv2.CAP_PROP_BRIGHTNESS))),
            range_minimum=_CAMERA_BRIGHTNESS_RANGE_MINIMUM,
            range_maximum=_CAMERA_BRIGHTNESS_RANGE_MAXIMUM))

        return_value.append(KeyValueMetaInt(
            key=_CAMERA_CONTRAST_KEY,
            value=int(round(self._capture.get(cv2.CAP_PROP_CONTRAST))),
            range_minimum=_CAMERA_CONTRAST_RANGE_MINIMUM,
            range_maximum=_CAMERA_CONTRAST_RANGE_MAXIMUM))

        return_value.append(KeyValueMetaInt(
            key=_CAMERA_SHARPNESS_KEY,
            value=int(round(self._capture.get(cv2.CAP_PROP_SHARPNESS))),
            range_minimum=_CAMERA_SHARPNESS_RANGE_MINIMUM,
            range_maximum=_CAMERA_SHARPNESS_RANGE_MAXIMUM))

        return_value.append(KeyValueMetaInt(
            key=_CAMERA_GAMMA_KEY,
            value=int(round(self._capture.get(cv2.CAP_PROP_GAMMA))),
            range_minimum=_CAMERA_GAMMA_RANGE_MINIMUM,
            range_maximum=_CAMERA_GAMMA_RANGE_MAXIMUM))

        return return_value

    def start_capture(self) -> None:
        if isinstance(self._capture_device_id, str) and self._capture_device_id.isnumeric():
            self._capture_device_id.isnumeric = int(self._capture_device_id)
        if self._capture is not None:
            return

        self._capture = self._detect_os_and_open_video(self._capture_device_id)
        # NOTE: The USB3 cameras bought for this project appear to require some basic parameters to be set,
        #       otherwise frame grab results in error
        default_resolution: ImageResolution = ImageResolution.from_str(_CAMERA_RESOLUTION_DEFAULT)
        self._capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, float(default_resolution.x_px))
        self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, float(default_resolution.y_px))
        self._capture.set(cv2.CAP_PROP_FPS, float(_CAMERA_FPS_DEFAULT))

        # These others we set to maintain consistency between start/stop cycles
        self._capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, float(_CAMERA_AUTO_EXPOSURE_DEFAULT))
        if os.name == "nt":  # Windows
            self._capture.set(cv2.CAP_PROP_EXPOSURE, float(_CAMERA_EXPOSURE_WINDOWS_DEFAULT))
        elif os.name == "posix":  # Unix
            self._capture.set(cv2.CAP_PROP_EXPOSURE, float(_CAMERA_EXPOSURE_UNIX_DEFAULT))
        self._capture.set(cv2.CAP_PROP_BRIGHTNESS, float(_CAMERA_BRIGHTNESS_DEFAULT))
        self._capture.set(cv2.CAP_PROP_CONTRAST, float(_CAMERA_CONTRAST_DEFAULT))
        self._capture.set(cv2.CAP_PROP_SHARPNESS, float(_CAMERA_SHARPNESS_DEFAULT))
        self._capture.set(cv2.CAP_PROP_GAMMA, float(_CAMERA_GAMMA_DEFAULT))

        self._capture_status.status = CaptureStatus.Status.RUNNING

    def stop_capture(self) -> None:
        if self._capture is not None:
            self._capture.release()
            self._capture = None
        self._capture_status.status = CaptureStatus.Status.STOPPED
