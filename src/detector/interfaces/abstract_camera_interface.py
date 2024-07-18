from src.common.structures import \
    CaptureFormat, \
    CaptureStatus, \
    KeyValueSimpleAny, \
    KeyValueMetaAbstract
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
        """
        May raise MCTDetectorRuntimeError
        """
        pass

    @abc.abstractmethod
    def set_capture_properties(self, parameters: list[KeyValueSimpleAny]) -> None:
        """
        May raise MCTDetectorRuntimeError
        """
        pass

    @abc.abstractmethod
    def get_capture_properties(self) -> list[KeyValueMetaAbstract]:
        """
        May raise MCTDetectorRuntimeError
        """
        pass

    @abc.abstractmethod
    def start_capture(self) -> None:
        """
        May raise MCTDetectorRuntimeError
        """
        pass

    @abc.abstractmethod
    def stop_capture(self) -> None:
        """
        May raise MCTDetectorRuntimeError
        """
        pass

    def get_encoded_image(
        self,
        image_format: CaptureFormat
    ) -> str:
        encoded_frame: bool
        encoded_image_rgb_single_row: numpy.array
        encoded, encoded_image_rgb_single_row = cv2.imencode(image_format, self._captured_image)
        encoded_image_rgb_bytes: bytes = encoded_image_rgb_single_row.tobytes()
        encoded_image_rgb_base64: str = base64.b64encode(encoded_image_rgb_bytes)
        return encoded_image_rgb_base64
