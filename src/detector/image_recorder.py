from .exceptions import MCTDetectorRuntimeError
from .structures import ImageRecorderConfiguration
from src.common import \
    ImageCoding, \
    StatusMessageSource
from src.common.util import IOUtils
import base64
import datetime
from enum import StrEnum
import io
import logging
import numpy
import os
import shutil
from typing import Final
import zipfile


logger = logging.getLogger(__name__)


class _OutputFormat(StrEnum):
    BMP: Final[str] = ".bmp"
    PNG: Final[str] = ".png"
    JPG: Final[str] = ".jpg"
    MKV: Final[str] = ".mkv"

    def is_image_format(self):
        if self == _OutputFormat.BMP or self == _OutputFormat.PNG or self == _OutputFormat.JPG:
            return True
        return False


class _CacheLocation(StrEnum):
    MEMORY: Final[str] = "memory"
    DISK: Final[str] = "disk"


_RECORDER_MAXIMUM_DURATION_SECONDS_DEFAULT: Final[float] = 30.0
_RECORDER_MAXIMUM_DURATION_SECONDS_RANGE_MINIMUM: Final[float] = 1.0
_RECORDER_MAXIMUM_DURATION_SECONDS_RANGE_MAXIMUM: Final[float] = 300.0
_RECORDER_MAXIMUM_DURATION_SECONDS_DIGIT_COUNT: Final[int] = 2
_RECORDER_CACHE_LOCATION_DEFAULT: Final[_CacheLocation] = _CacheLocation.DISK
_RECORDER_CACHE_LOCATION_OPTIONS: Final[list[_CacheLocation]] = [
    _CacheLocation.MEMORY,
    _CacheLocation.DISK]
_RECORDER_OUTPUT_FORMAT_DEFAULT: Final[_OutputFormat] = _OutputFormat.PNG
_RECORDER_OUTPUT_FORMAT_OPTIONS: Final[list[_OutputFormat]] = [
    _OutputFormat.BMP,
    _OutputFormat.PNG,
    _OutputFormat.JPG,
    _OutputFormat.MKV]


class ImageRecorder:

    class Status(StrEnum):
        IDLE: Final[str] = "IDLE"
        RUNNING: Final[str] = "RUNNING"

    _configuration: ImageRecorderConfiguration
    _status_message_source: StatusMessageSource
    _recording_status: Status
    _recording_last_timestamp: datetime.datetime | None
    _recording_image_count: int
    _remaining_time_seconds: float

    _cache_memory: list[tuple[datetime.datetime, numpy.ndarray]]
    _cache_location: _CacheLocation
    _output_format: _OutputFormat

    ZIP_IMAGE_PATH: Final[str] = "images"

    def __init__(
        self,
        configuration: ImageRecorderConfiguration,
        status_message_source: StatusMessageSource
    ):
        self._configuration = configuration
        self._status_message_source = status_message_source
        self._recording_status = ImageRecorder.Status.IDLE
        self._recording_last_timestamp = None
        self._recording_image_count = 0
        self._remaining_time_seconds = 0.0
        self._cache_memory = list()
        self._cache_location = _RECORDER_CACHE_LOCATION_DEFAULT
        self._output_format = _RECORDER_OUTPUT_FORMAT_DEFAULT

    def clear(self) -> None:
        image_path_contents: list[str] = os.listdir(self._configuration.image_path)
        for image_path_content in image_path_contents:
            if os.path.isfile(image_path_content):
                os.remove(os.path.join(self._configuration.image_path, image_path_content))
        self._recording_image_count = 0

    def get_image_count(self) -> int:
        return self._recording_image_count

    def get_remaining_time_seconds(self) -> float:
        return self._remaining_time_seconds

    def get_status(self) -> Status:
        return self._recording_status

    def retrieve_zip_base64(self) -> str:
        bytes_io: io.BytesIO = io.BytesIO()
        with zipfile.ZipFile(bytes_io, 'a', zipfile.ZIP_DEFLATED, False) as zip_memory:
            image_path_contents: list[str] = os.listdir(self._configuration.image_path)
            for image_path_content in image_path_contents:
                local_filepath: str = os.path.join(self._configuration.image_path, image_path_content)
                if not os.path.isfile(image_path_content):
                    logger.warning(f"While retrieving recording, found non-file {local_filepath}.")
                    continue
                zip_filepath: str = os.path.join(ImageRecorder.ZIP_IMAGE_PATH, image_path_content)
                with open(local_filepath, 'rb') as image_file:
                    zip_memory.writestr(zip_filepath, image_file.read())
        archive_bytes: bytes = bytes_io.getvalue()
        archive_base64: str = base64.b64encode(archive_bytes).decode("ascii")
        return archive_base64

    def start(
        self,
        recording_duration_seconds: float
    ) -> None:
        image_path_exists: bool = IOUtils.exists(
            path=self._configuration.image_path,
            pathtype="path",
            create_path=True,
            on_error_for_user=lambda msg: self._status_message_source.enqueue_status_message(
                severity="error",
                message=msg),
            on_error_for_dev=logger.error)
        if not image_path_exists:
            general_message: str = f"Could not find or create recording path."
            self._status_message_source.enqueue_status_message(
                severity="error",
                message=general_message)
            detailed_message: str = f"{self._configuration.image_path} does not exist and could not be created."
            logger.error(detailed_message)
            self._recording_status = ImageRecorder.Status.IDLE
            raise MCTDetectorRuntimeError(message=general_message)
        self.clear()
        if len(os.listdir(self._configuration.image_path)) > 0:
            general_message: str = f"Non-image contents were found in image path and could not be safely deleted."
            self._status_message_source.enqueue_status_message(
                severity="error",
                message=general_message)
            detailed_message: str = f"{self._configuration.image_path} contains non-image contents, will not clear."
            logger.error(detailed_message)
            self._recording_status = ImageRecorder.Status.IDLE
            raise MCTDetectorRuntimeError(message=general_message)
        self._recording_status = ImageRecorder.Status.RUNNING
        self._remaining_time_seconds = recording_duration_seconds
        return

    def stop(self) -> None:
        self._recording_status = ImageRecorder.Status.IDLE
        self._recording_last_timestamp = None
        self._remaining_time_seconds = 0.0

    # noinspection DuplicatedCode
    def update(
        self,
        image_data: numpy.ndarray,
        image_timestamp: datetime.datetime
    ) -> None:
        if self._recording_status != ImageRecorder.Status.RUNNING:
            return
        if self._recording_last_timestamp is not None:
            if image_timestamp <= self._recording_last_timestamp:
                return
            self._remaining_time_seconds -= (image_timestamp - self._recording_last_timestamp).total_seconds()
        self._cache_memory.append((image_timestamp, image_data))
        if self._cache_location == _CacheLocation.DISK:
            self._write_cached_frames()
        self._recording_last_timestamp = image_timestamp
        self._recording_image_count += 1
        if self._remaining_time_seconds < 0.0:
            self.stop()

    def _write_cached_frames(
        self
    ) -> None:
        for image_timestamp, image_data in self._cache_memory:
            if self._output_format.is_image_format():
                image_filename = image_timestamp.isoformat()\
                    .replace(':', '')\
                    .replace('-', '')\
                    .replace('T', '')\
                    .replace('.', '') + self._output_format
                image_filepath = os.path.join(self._configuration.image_path, image_filename)
                # noinspection PyTypeChecker
                image_bytes = ImageCoding.image_to_bytes(image_data=image_data, image_format=self._output_format)
                image_byte_count: int = len(image_bytes)

                _, _, free_disk_byte_count = shutil.disk_usage(self._configuration.image_path)
                if free_disk_byte_count < image_byte_count:
                    message: str = "Stopping image recording before writing current image due to limited disk space."
                    self._status_message_source.enqueue_status_message(
                        severity="error",
                        message=message)
                    logger.info(message)
                    self.stop()  # TODO: Review this and next block
                    return  # Don't even try to write image

                if (free_disk_byte_count - image_byte_count) < self._configuration.min_disk_byte_count:
                    message: str = "Stopping image recording after writing current image due to limited disk space."
                    self._status_message_source.enqueue_status_message(
                        severity="error",
                        message=message)
                    logger.info(message)
                    self.stop()

                with (open(image_filepath, 'wb') as in_file):
                    in_file.write(image_bytes)
            else:
                raise NotImplementedError()
