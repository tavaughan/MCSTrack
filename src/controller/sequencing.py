from .routing import \
    CallbackRouter, \
    ConnectionRouter
from src.common import \
    Annotator, \
    Camera, \
    DequeueStatusMessagesRequest, \
    DequeueStatusMessagesResponse, \
    DetectorFrame, \
    DetectorPoseMode, \
    EmptyResponse, \
    ErrorResponse, \
    ImageFormat, \
    ImageResolution, \
    IntrinsicCalibration, \
    IntrinsicParameters, \
    KeyValueMetaAny, \
    KeyValueSimpleAny, \
    Matrix4x4, \
    MCTRequest, \
    MCTRequestSeries, \
    MCTResponse, \
    MCTResponseSeries, \
    MixerFrame, \
    SeverityLabel, \
    StatusMessageSource, \
    Target, \
    TimestampGetRequest, \
    TimestampGetResponse, \
    TimeSyncStartRequest, \
    TimeSyncStopRequest
from src.detector import \
    DetectorFrameGetRequest, \
    DetectorFrameGetResponse, \
    DetectorParametersGetRequest, \
    DetectorParametersGetResponse, \
    DetectorParametersSetRequest, \
    DetectorParametersSetResponse, \
    DetectorQueryRequest, \
    DetectorQueryResponse, \
    DetectorStartRequest, \
    DetectorStopRequest, \
    IntrinsicCalibrationCalculateRequest, \
    IntrinsicCalibrationCalculateResponse, \
    IntrinsicCalibrationDeleteStagedRequest, \
    IntrinsicCalibrationImageAddRequest, \
    IntrinsicCalibrationImageAddResponse, \
    IntrinsicCalibrationImageGetRequest, \
    IntrinsicCalibrationImageGetResponse, \
    IntrinsicCalibrationImageMetadataUpdateRequest, \
    IntrinsicCalibrationMetadataListRequest, \
    IntrinsicCalibrationMetadataListResponse, \
    IntrinsicCalibrationResolutionListRequest, \
    IntrinsicCalibrationResolutionListResponse, \
    IntrinsicCalibrationResultGetRequest, \
    IntrinsicCalibrationResultGetResponse, \
    IntrinsicCalibrationResultGetActiveRequest, \
    IntrinsicCalibrationResultGetActiveResponse, \
    IntrinsicCalibrationResultMetadataUpdateRequest
from src.mixer import \
    ExtrinsicCalibrationCalculateRequest, \
    ExtrinsicCalibrationCalculateResponse, \
    ExtrinsicCalibrationDeleteStagedRequest, \
    ExtrinsicCalibrationImageAddRequest, \
    ExtrinsicCalibrationImageAddResponse, \
    ExtrinsicCalibrationImageGetRequest, \
    ExtrinsicCalibrationImageGetResponse, \
    ExtrinsicCalibrationImageMetadataListRequest, \
    ExtrinsicCalibrationImageMetadataListResponse, \
    ExtrinsicCalibrationImageMetadataUpdateRequest, \
    ExtrinsicCalibrationResultGetActiveRequest, \
    ExtrinsicCalibrationResultGetActiveResponse, \
    ExtrinsicCalibrationResultGetRequest, \
    ExtrinsicCalibrationResultGetResponse, \
    ExtrinsicCalibrationResultMetadataListRequest, \
    ExtrinsicCalibrationResultMetadataListResponse, \
    ExtrinsicCalibrationResultMetadataUpdateRequest, \
    Mixer, \
    MixerFrameGetRequest, \
    MixerFrameGetResponse, \
    MixerIntrinsicUpdateRequest, \
    MixerQueryRequest, \
    MixerQueryResponse, \
    MixerStartRequest, \
    MixerStopRequest, \
    PoseSolverDetectorFrameAddRequest, \
    PoseSolverExtrinsicSetRequest, \
    PoseSolverExtrinsicClearRequest, \
    PoseSolverTargetsClearRequest, \
    PoseSolverTargetsSetRequest
import abc
import datetime
from dataclasses import dataclass, field
import logging
import statistics
from typing import Callable, ClassVar, Final, Union
import uuid


logger = logging.getLogger(__name__)


_TIME_SYNC_DEFAULT_SAMPLE_COUNT: Final[int] = 5
_ZERO_UUID: uuid.UUID = uuid.UUID("00000000-0000-4000-8000-000000000000")


class AbstractSequencer(abc.ABC):

    _status_message_source: StatusMessageSource
    _connection_router: ConnectionRouter
    _callback_router: CallbackRouter

    _pending_request_ids: list[uuid.UUID]

    def __init__(
        self,
        status_message_source: StatusMessageSource,
        connection_router: ConnectionRouter,
        callback_router: CallbackRouter
    ):
        self._status_message_source = status_message_source
        self._connection_router = connection_router
        self._callback_router = callback_router
        self._pending_request_ids = list()

    def is_finished(self) -> bool:
        return len(self._pending_request_ids) <= 0

    def reset(self) -> None:
        for request_id in self._pending_request_ids:
            self._callback_router.remove_callback(request_id)

    def _report_response_series_and_errors(
        self,
        response_series: MCTResponseSeries,
        expected_types: list[type[MCTResponse]]
    ) -> int:
        """
        Return the number of errors found
        """
        errors_found: int = 0
        label: str = response_series.responder
        request_id: uuid.UUID
        try:
            request_id = uuid.UUID(response_series.request_id)
        except ValueError:
            self._status_message_source.enqueue_status_message(
                severity=SeverityLabel.ERROR,
                message=f"Response contained invalid request ID {response_series.request_id} from {label}.")
            errors_found += 1
            request_id = _ZERO_UUID
        if request_id not in self._pending_request_ids:
            self._status_message_source.enqueue_status_message(
                severity=SeverityLabel.ERROR,
                message=f"Received unexpected response with ID {response_series.request_id} from {label}.")
            errors_found += 1
        self._pending_request_ids.remove(request_id)
        if len(response_series.series) != len(expected_types):
            self._status_message_source.enqueue_status_message(
                severity=SeverityLabel.ERROR,
                message=f"Unexpected (incorrect) number of responses from {label}. Expected {len(expected_types)}.")
            errors_found += 1
            for response_index, response in response_series.series:
                if isinstance(response, DequeueStatusMessagesResponse):
                    for status_message in response.status_messages:
                        self._status_message_source.enqueue_status_message(
                            severity=status_message.severity,
                            message=status_message.message,
                            source_label=status_message.source_label,
                            timestamp_utc_iso8601=status_message.timestamp_utc_iso8601)
                if isinstance(response, ErrorResponse):
                    self._status_message_source.enqueue_status_message(
                        severity=SeverityLabel.ERROR,
                        message=f"Error at index {response_index}: {response.message}",
                        source_label=label)
                errors_found += 1
            return errors_found
        for response_index, response in enumerate(response_series.series):
            expected_type: type = expected_types[response_index]
            expected_type_name: str = expected_type.__name__
            if isinstance(response, DequeueStatusMessagesResponse):
                for status_message in response.status_messages:
                    self._status_message_source.enqueue_status_message(
                        severity=status_message.severity,
                        message=status_message.message,
                        source_label=status_message.source_label,
                        timestamp_utc_iso8601=status_message.timestamp_utc_iso8601)
            if isinstance(response, ErrorResponse):
                self._status_message_source.enqueue_status_message(
                    severity=SeverityLabel.ERROR,
                    message=f"Error in place of {expected_type_name}: {response.message}",
                    source_label=label)
                errors_found += 1
            if not isinstance(response, expected_type):
                self._status_message_source.enqueue_status_message(
                    severity=SeverityLabel.ERROR,
                    message=f"Unexpected response in place of {expected_type_name}: {type(response).__name__}")
                errors_found += 1
        return errors_found

    def _send_request_series(
        self,
        component_label: str,
        requests: list[MCTRequest],
        callback: CallbackRouter.CallbackFunction
    ) -> uuid.UUID:
        request_series: MCTRequestSeries = MCTRequestSeries(series=requests)
        request_id: uuid.UUID = uuid.UUID(request_series.request_id)
        self._connection_router.enqueue_request_series(label=component_label, request_series=request_series)
        self._callback_router.add_callback(request_id=request_id, callback=callback)
        self._pending_request_ids.append(request_id)
        return request_id


class TimeSyncSequencer(AbstractSequencer):

    class ComponentData:
        class Sample:
            local_sent_datetime: datetime.datetime
            remote_received_datetime: datetime.datetime
            local_received_datetime: datetime.datetime
            def __init__(
                self,
                local_sent_timestamp_iso8601: str,
                remote_received_timestamp_iso8601: str,
                local_received_timestamp: datetime.datetime
            ):
                self.local_sent_datetime = datetime.datetime.fromisoformat(local_sent_timestamp_iso8601)
                self.remote_received_datetime = datetime.datetime.fromisoformat(remote_received_timestamp_iso8601)
                self.local_received_datetime = local_received_timestamp

        samples: list[Sample]
        _network_round_delay_milliseconds: float | None
        _clock_offset_to_remote_milliseconds: float | None

        def __init__(self):
            self.samples = list()
            self._network_round_delay_milliseconds = None
            self._clock_offset_to_remote_milliseconds = None

        def _calculate(self):
            round_delays_ms: list[float] = list()  # Caused by network
            offsets_ms: list[float] = list()       # Caused by difference in computer clock time
            for sample in self.samples:
                round_delay_delta: datetime.timedelta = sample.local_sent_datetime - sample.local_received_datetime
                round_delay_ms: float = round_delay_delta / datetime.timedelta(milliseconds=1)
                round_delays_ms.append(round_delay_ms)
                one_way_delay_ms: int = round(round_delay_ms / 2.0)
                one_way_delay_delta: datetime.timedelta = datetime.timedelta(milliseconds=one_way_delay_ms)
                local_datetime: datetime.datetime = sample.local_sent_datetime + one_way_delay_delta
                remote_datetime: datetime.datetime = sample.remote_received_datetime
                offset_delta: datetime.timedelta = remote_datetime - local_datetime
                offset_ms: float = offset_delta / datetime.timedelta(milliseconds=1)
                offsets_ms.append(offset_ms)
            self._network_round_delay_milliseconds = statistics.median(round_delays_ms)
            self._clock_offset_to_remote_milliseconds = statistics.median(offsets_ms)

        @property
        def network_latency_milliseconds(self) -> int:
            """Delay from round trip (caveat that it involves some processing)."""
            if self._network_round_delay_milliseconds is None:
                self._calculate()
            return round(self._network_round_delay_milliseconds)

        @property
        def clock_offset_milliseconds(self) -> int:
            """The amount to add to the local time to get the remote time"""
            if self._clock_offset_to_remote_milliseconds is None:
                self._calculate()
            return round(self._clock_offset_to_remote_milliseconds)

    data_by_component_label: dict[str, ComponentData]
    _sample_count: int

    def __init__(
        self,
        status_message_source: StatusMessageSource,
        connection_router: ConnectionRouter,
        callback_router: CallbackRouter
    ):
        super().__init__(
            status_message_source=status_message_source,
            connection_router=connection_router,
            callback_router=callback_router)
        self.data_by_component_label = dict()
        self.reset()

    def begin(
        self,
        component_labels: list[str],
        sample_count: int = _TIME_SYNC_DEFAULT_SAMPLE_COUNT
    ):
        if len(self._pending_request_ids) > 0:
            message: str = f"MixerStartupSequencer.begin() called when already busy. Try waiting or calling reset()."
            self._status_message_source.enqueue_status_message(severity=SeverityLabel.ERROR, message=message)
            return
        if len(component_labels) == 0:
            self._status_message_source.enqueue_status_message(
                severity=SeverityLabel.ERROR,
                message=f"No component labels were provided to TimeSyncSequencer")
            return
        self.reset()
        self._sample_count = sample_count
        for component_label in component_labels:
            self.data_by_component_label[component_label] = TimeSyncSequencer.ComponentData()
        self._request_1_start()

    def reset(self):
        super().reset()
        self.data_by_component_label.clear()
        self._sample_count = _TIME_SYNC_DEFAULT_SAMPLE_COUNT

    def _request_1_start(self):
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message="TimeSyncSequencer._request_1_start()")
        for component_label in self.data_by_component_label.keys():
            self._send_request_series(
                component_label=component_label,
                requests=[
                    TimeSyncStartRequest(),
                    DequeueStatusMessagesRequest()],
                callback=self._request_1_start_responded)

    def _request_1_start_responded(
        self,
        response_series: MCTResponseSeries,
        _passthrough_parameters: dict[str, ...]
    ):
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message="TimeSyncSequencer._request_1_start_responded()")
        if self._report_response_series_and_errors(
            response_series=response_series,
            expected_types=[
                EmptyResponse,
                DequeueStatusMessagesResponse]
        ):
            return
        if len(self._pending_request_ids) > 0:
            return
        self._request_2_timestamp()

    def _request_2_timestamp(self):
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message="TimeSyncSequencer._request_2_timestamp()")
        for component_label in self.data_by_component_label.keys():
            now_utc_iso8601: str = datetime.datetime.now(tz=datetime.timezone.utc).isoformat()
            self._send_request_series(
                component_label=component_label,
                requests=[
                    TimestampGetRequest(requester_timestamp_utc_iso8601=now_utc_iso8601),
                    DequeueStatusMessagesRequest()],
                callback=self._request_2_timestamp_responded)

    def _request_2_timestamp_responded(
        self,
        response_series: MCTResponseSeries,
        _passthrough_parameters: dict[str, ...]
    ):
        now_utc_iso8601: datetime.datetime = datetime.datetime.now(tz=datetime.timezone.utc)
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message="TimeSyncSequencer._request_2_timestamp_responded()")
        if self._report_response_series_and_errors(
            response_series=response_series,
            expected_types=[
                TimestampGetResponse,
                DequeueStatusMessagesResponse]
        ):
            return
        # noinspection PyTypeChecker
        response: TimestampGetResponse = response_series.series[0]
        component_label: str = response_series.responder
        self.data_by_component_label[component_label].samples.append(TimeSyncSequencer.ComponentData.Sample(
            local_sent_timestamp_iso8601=response.requester_timestamp_utc_iso8601,
            remote_received_timestamp_iso8601=response.responder_timestamp_utc_iso8601,
            local_received_timestamp=now_utc_iso8601))
        if len(self._pending_request_ids) > 0:
            return
        samples_collected: int = len(self.data_by_component_label[component_label].samples)
        if samples_collected < self._sample_count:
            self._request_2_timestamp()
        else:
            self._request_3_stop()

    def _request_3_stop(self):
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message="TimeSyncSequencer._request_3_stop()")
        for component_label in self.data_by_component_label.keys():
            self._send_request_series(
                component_label=component_label,
                requests=[
                    TimeSyncStopRequest(),
                    DequeueStatusMessagesRequest()],
                callback=self._request_3_stop_responded)

    def _request_3_stop_responded(
        self,
        response_series: MCTResponseSeries,
        _passthrough_parameters: dict[str, ...]
    ):
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message="TimeSyncSequencer._request_3_stop_responded()")
        if self._report_response_series_and_errors(
            response_series=response_series,
            expected_types=[
                EmptyResponse,
                DequeueStatusMessagesResponse]
        ):
            return


# noinspection DuplicatedCode
class DetectorStartupSequencer(AbstractSequencer):

    @dataclass
    class InputDetectorData:
        detector_label: str = field()
        camera_parameters: list[KeyValueSimpleAny] = field()
        annotator_parameters: list[KeyValueSimpleAny] = field()

    @dataclass
    class DetectorData:
        camera_resolution: ImageResolution | None = field(default=None)
        camera_parameters: list[KeyValueMetaAny] = field(default=None)
        annotator_parameters: list[KeyValueMetaAny] = field(default=None)
        intrinsic_calibration: IntrinsicCalibration | None = field(default=None)
        done_query: bool = field(default=False)
        done_reset: bool = field(default=False)
        done_start: bool = field(default=False)
        done_set_parameters: bool = field(default=False)
        done_get_parameters: bool = field(default=False)

    _input_detector_data: list[InputDetectorData]
    data_by_detector_label: dict[str, DetectorData]

    def __init__(
        self,
        status_message_source: StatusMessageSource,
        connection_router: ConnectionRouter,
        callback_router: CallbackRouter
    ):
        super().__init__(
            status_message_source=status_message_source,
            connection_router=connection_router,
            callback_router=callback_router)
        self._input_detector_data = list()
        self.data_by_detector_label = dict()
        self.reset()

    def begin(
        self,
        detector_data: list[InputDetectorData]
    ) -> None:
        if len(self._pending_request_ids) > 0:
            message: str = f"MixerStartupSequencer.begin() called when already busy. Try waiting or calling reset()."
            self._status_message_source.enqueue_status_message(severity=SeverityLabel.ERROR, message=message)
        detector_count: int = len(detector_data)
        if detector_count <= 0:
            self._status_message_source.enqueue_status_message(
                severity=SeverityLabel.ERROR,
                message=f"No inputs were provided to DetectorStartupSequencer")
            return
        self.reset()
        self._input_detector_data = detector_data
        for detector in detector_data:
            self.data_by_detector_label[detector.detector_label] = DetectorStartupSequencer.DetectorData()
        self._request_1_query()

    def reset(self) -> None:
        super().reset()
        self._input_detector_data.clear()
        self.data_by_detector_label.clear()

    def _request_1_query(self):
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message="DetectorStartupSequencer._request_1_query()")
        for detector in self._input_detector_data:
            detector_label: str = detector.detector_label
            self._send_request_series(
                component_label=detector_label,
                requests=[
                    DetectorQueryRequest(),
                    DequeueStatusMessagesRequest()],
                callback=self._request_1_query_responded)

    def _request_1_query_responded(
        self,
        response_series: MCTResponseSeries,
        _passthrough_parameters: dict[str, ...]
    ):
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message="DetectorStartupSequencer._request_1_query_responded()")
        if self._report_response_series_and_errors(
            response_series=response_series,
            expected_types=[
                DetectorQueryResponse,
                DequeueStatusMessagesResponse]
        ):
            return
        detector_label: str = response_series.responder
        # noinspection PyTypeChecker
        response: DetectorQueryResponse = response_series.series[0]
        self.data_by_detector_label[detector_label].done_query = True
        if response.camera_status == Camera.Status.RUNNING and response.annotator_status == Annotator.Status.RUNNING:
            self.data_by_detector_label[detector_label].done_reset = True
            self.data_by_detector_label[detector_label].done_start = True  # These steps are not needed
        elif response.camera_status != Camera.Status.STOPPED and response.annotator_status == Annotator.Status.STOPPED:
            self.data_by_detector_label[detector_label].done_reset = True  # Not needed
        if len(self._pending_request_ids) > 0:
            return
        detectors_needing_reset_count: int = sum([
            (1 - self.data_by_detector_label[detector_label].done_reset)
            for detector_label in self.data_by_detector_label.keys()])
        if detectors_needing_reset_count > 0:
            self._request_2_reset()
            return
        detectors_needing_start_count: int = sum([
            (1 - self.data_by_detector_label[detector_label].done_start)
            for detector_label in self.data_by_detector_label.keys()])
        if detectors_needing_start_count > 0:
            self._request_3_start()
            return
        self._request_4_set_parameters()

    def _request_2_reset(self):
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message="DetectorStartupSequencer._request_2_reset()")
        for detector in self._input_detector_data:
            detector_label: str = detector.detector_label
            if not self.data_by_detector_label[detector_label].done_reset:
                self._send_request_series(
                    component_label=detector_label,
                    requests=[
                        DetectorStopRequest(),
                        DequeueStatusMessagesRequest()],
                    callback=self._request_2_reset_responded)

    def _request_2_reset_responded(
        self,
        response_series: MCTResponseSeries,
        _passthrough_parameters: dict[str, ...]
    ):
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message="DetectorStartupSequencer._request_2_reset_responded()")
        if self._report_response_series_and_errors(
            response_series=response_series,
            expected_types=[
                EmptyResponse,
                DequeueStatusMessagesResponse]
        ):
            return
        detector_label: str = response_series.responder
        self.data_by_detector_label[detector_label].done_reset = True
        if len(self._pending_request_ids) > 0:
            return
        detectors_needing_start_count: int = sum([
            (1 - self.data_by_detector_label[detector_label].done_start)
            for detector_label in self.data_by_detector_label.keys()])
        if detectors_needing_start_count > 0:
            self._request_3_start()
            return
        self._request_4_set_parameters()

    def _request_3_start(self):
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message="DetectorStartupSequencer._request_3_start()")
        for detector in self._input_detector_data:
            detector_label: str = detector.detector_label
            if not self.data_by_detector_label[detector_label].done_start:
                self._send_request_series(
                    component_label=detector_label,
                    requests=[
                        DetectorStartRequest(),
                        DequeueStatusMessagesRequest()],
                    callback=self._request_3_start_responded)

    def _request_3_start_responded(
        self,
        response_series: MCTResponseSeries,
        _passthrough_parameters: dict[str, ...]
    ):
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message="DetectorStartupSequencer._request_3_start_responded()")
        if self._report_response_series_and_errors(
            response_series=response_series,
            expected_types=[
                EmptyResponse,
                DequeueStatusMessagesResponse]
        ):
            return
        detector_label: str = response_series.responder
        self.data_by_detector_label[detector_label].done_start = True
        if len(self._pending_request_ids) > 0:
            return
        self._request_4_set_parameters()

    def _request_4_set_parameters(self):
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message="DetectorStartupSequencer._request_4_set_parameters()")
        for detector in self._input_detector_data:
            detector_label: str = detector.detector_label
            self._send_request_series(
                component_label=detector_label,
                requests=[
                    DetectorParametersSetRequest(
                        camera_parameters=detector.camera_parameters,
                        annotator_parameters=detector.annotator_parameters),
                    DequeueStatusMessagesRequest()],
                callback=self._request_4_set_parameters_responded)

    def _request_4_set_parameters_responded(
        self,
        response_series: MCTResponseSeries,
        _passthrough_parameters: dict[str, ...]
    ):
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message="DetectorStartupSequencer._request_4_set_parameters_responded()")
        if self._report_response_series_and_errors(
            response_series=response_series,
            expected_types=[
                DetectorParametersSetResponse,
                DequeueStatusMessagesResponse]
        ):
            return
        detector_label: str = response_series.responder
        self.data_by_detector_label[detector_label].done_set_parameters = True
        if len(self._pending_request_ids) > 0:
            return
        self._request_5_get_parameters()

    def _request_5_get_parameters(self):
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message="DetectorStartupSequencer._request_5_get_parameters()")
        for detector in self._input_detector_data:
            detector_label: str = detector.detector_label
            self._send_request_series(
                component_label=detector_label,
                requests=[
                    DetectorParametersGetRequest(),
                    IntrinsicCalibrationResultGetActiveRequest(),
                    DequeueStatusMessagesRequest()],
                callback=self._request_5_get_parameters_responded)

    def _request_5_get_parameters_responded(
        self,
        response_series: MCTResponseSeries,
        _passthrough_parameters: dict[str, ...]
    ):
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message="DetectorStartupSequencer._request_5_get_parameters_responded()")
        if self._report_response_series_and_errors(
            response_series=response_series,
            expected_types=[
                DetectorParametersGetResponse,
                IntrinsicCalibrationResultGetActiveResponse,
                DequeueStatusMessagesResponse]
        ):
            return
        detector_label: str = response_series.responder
        # noinspection PyTypeChecker
        parameters_response: DetectorParametersGetResponse = response_series.series[0]
        self.data_by_detector_label[detector_label].camera_resolution = parameters_response.camera_resolution
        self.data_by_detector_label[detector_label].camera_parameters = parameters_response.camera_parameters
        self.data_by_detector_label[detector_label].annotator_parameters = parameters_response.annotator_parameters
        # noinspection PyTypeChecker
        calibration_response: IntrinsicCalibrationResultGetActiveResponse = response_series.series[1]
        self.data_by_detector_label[detector_label].intrinsic_calibration = calibration_response.intrinsic_calibration
        self.data_by_detector_label[detector_label].done_get_parameters = True


# noinspection DuplicatedCode
class MixerStartupSequencer(AbstractSequencer):

    @dataclass
    class InputMixerData:
        @dataclass
        class Detector:
            detector_label: str = field()
            intrinsic_parameters: IntrinsicParameters = field()
            pose_mode: DetectorPoseMode = field()
            extrinsic_matrix: Matrix4x4 = field()
        mixer_label: str = field()
        detectors: list[Detector] = field()
        targets: list[Target] = field()
        solver_parameters: list[KeyValueSimpleAny] = field()

    @dataclass
    class OutputMixerData:
        extrinsic_calibration: Matrix4x4 | None = field(default=None)
        done_query: bool = field(default=False)
        done_reset: bool = field(default=False)
        done_start: bool = field(default=False)
        done_clear: bool = field(default=False)
        done_set_intrinsics: bool = field(default=False)
        done_set_targets: bool = field(default=False)
        done_set_extrinsics: bool = field(default=False)

    _input_mixer_data_by_label: dict[str, InputMixerData]
    data_by_mixer_label: dict[str, OutputMixerData]

    def __init__(
        self,
        status_message_source: StatusMessageSource,
        connection_router: ConnectionRouter,
        callback_router: CallbackRouter
    ):
        super().__init__(
            status_message_source=status_message_source,
            connection_router=connection_router,
            callback_router=callback_router)
        self._input_mixer_data_by_label = dict()
        self.data_by_mixer_label = dict()
        self.reset()

    def begin(
        self,
        mixer_data: list[InputMixerData]
    ) -> None:
        if len(self._pending_request_ids) > 0:
            message: str = f"MixerStartupSequencer.begin() called when already busy. Try waiting or calling reset()."
            self._status_message_source.enqueue_status_message(severity=SeverityLabel.ERROR, message=message)
            return
        mixer_count: int = len(mixer_data)
        if mixer_count <= 0:
            self._status_message_source.enqueue_status_message(
                severity=SeverityLabel.ERROR,
                message=f"No mixer data was provided to MixerStartupSequencer")
            return
        self.reset()
        self._input_mixer_data_by_label = {mixer.mixer_label: mixer for mixer in mixer_data}
        for mixer in self._input_mixer_data_by_label.values():
            self.data_by_mixer_label[mixer.mixer_label] = MixerStartupSequencer.OutputMixerData()
        self._request_1_query()

    def reset(self) -> None:
        super().reset()
        self._input_mixer_data_by_label.clear()
        self.data_by_mixer_label.clear()

    def _request_1_query(self):
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message="MixerStartupSequencer._request_1_query()")
        for mixer in self._input_mixer_data_by_label.values():
            mixer_label: str = mixer.mixer_label
            self._send_request_series(
                component_label=mixer_label,
                requests=[
                    MixerQueryRequest(),
                    DequeueStatusMessagesRequest()],
                callback=self._request_1_query_responded)

    def _request_1_query_responded(
        self,
        response_series: MCTResponseSeries,
        _passthrough_parameters: dict[str, ...]
    ):
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message="MixerStartupSequencer._request_1_query_responded()")
        if self._report_response_series_and_errors(
            response_series=response_series,
            expected_types=[
                MixerQueryResponse,
                DequeueStatusMessagesResponse]
        ):
            return
        mixer_label: str = response_series.responder
        # noinspection PyTypeChecker
        response: MixerQueryResponse = response_series.series[0]
        self.data_by_mixer_label[mixer_label].done_query = True
        if response.mixer_status == Mixer.Status.RUNNING:
            self.data_by_mixer_label[mixer_label].done_reset = True
            self.data_by_mixer_label[mixer_label].done_start = True  # These steps are not needed
        elif response.mixer_status != Mixer.Status.STOPPED:
            self.data_by_mixer_label[mixer_label].done_reset = True  # Not needed
        if len(self._pending_request_ids) > 0:
            return
        mixers_needing_reset_count: int = sum([
            (1 - self.data_by_mixer_label[mixer_label].done_reset)
            for mixer_label in self.data_by_mixer_label.keys()])
        if mixers_needing_reset_count > 0:
            self._request_2_reset()
            return
        mixers_needing_start_count: int = sum([
            (1 - self.data_by_mixer_label[mixer_label].done_start)
            for mixer_label in self.data_by_mixer_label.keys()])
        if mixers_needing_start_count > 0:
            self._request_3_start()
            return
        self._request_4_clear()

    def _request_2_reset(self):
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message="MixerStartupSequencer._request_2_reset()")
        for mixer in self._input_mixer_data_by_label.values():
            mixer_label: str = mixer.mixer_label
            if not self.data_by_mixer_label[mixer_label].done_reset:
                self._send_request_series(
                    component_label=mixer_label,
                    requests=[
                        MixerStopRequest(),
                        DequeueStatusMessagesRequest()],
                    callback=self._request_2_reset_responded)

    def _request_2_reset_responded(
        self,
        response_series: MCTResponseSeries,
        _passthrough_parameters: dict[str, ...]
    ):
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message="MixerStartupSequencer._request_2_reset_responded()")
        if self._report_response_series_and_errors(
            response_series=response_series,
            expected_types=[
                EmptyResponse,
                DequeueStatusMessagesResponse]
        ):
            return
        mixer_label: str = response_series.responder
        self.data_by_mixer_label[mixer_label].done_reset = True
        if len(self._pending_request_ids) > 0:
            return
        self._request_3_start()

    def _request_3_start(self):
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message="MixerStartupSequencer._request_3_start()")
        for mixer in self._input_mixer_data_by_label.values():
            mixer_label: str = mixer.mixer_label
            if not self.data_by_mixer_label[mixer_label].done_start:
                self._send_request_series(
                    component_label=mixer_label,
                    requests=[
                        MixerStartRequest(),
                        DequeueStatusMessagesRequest()],
                    callback=self._request_3_start_responded)

    def _request_3_start_responded(
        self,
        response_series: MCTResponseSeries,
        _passthrough_parameters: dict[str, ...]
    ):
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message="MixerStartupSequencer._request_3_start_responded()")
        if self._report_response_series_and_errors(
            response_series=response_series,
            expected_types=[
                EmptyResponse,
                DequeueStatusMessagesResponse]
        ):
            return
        mixer_label: str = response_series.responder
        self.data_by_mixer_label[mixer_label].done_start = True
        if len(self._pending_request_ids) > 0:
            return
        self._request_4_clear()

    def _request_4_clear(self):
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message="DetectorStartupSequencer._request_4_clear_extrinsics()")
        for mixer in self._input_mixer_data_by_label.values():
            mixer_label: str = mixer.mixer_label
            self._send_request_series(
                component_label=mixer_label,
                requests=[
                    PoseSolverTargetsClearRequest(),
                    PoseSolverExtrinsicClearRequest(),
                    DequeueStatusMessagesRequest()],
                callback=self._request_4_clear_responded)

    def _request_4_clear_responded(
        self,
        response_series: MCTResponseSeries,
        _passthrough_parameters: dict[str, ...]
    ):
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message="DetectorStartupSequencer._request_4_clear_extrinsics_responded()")
        if self._report_response_series_and_errors(
            response_series=response_series,
            expected_types=[
                EmptyResponse,
                EmptyResponse,
                DequeueStatusMessagesResponse]
        ):
            return
        mixer_label: str = response_series.responder
        self.data_by_mixer_label[mixer_label].done_clear = True
        if len(self._pending_request_ids) > 0:
            return
        self._request_5_set_intrinsics()

    def _request_5_set_intrinsics(self):
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message="DetectorStartupSequencer._request_5_set_intrinsics()")
        for mixer in self._input_mixer_data_by_label.values():
            mixer_label: str = mixer.mixer_label
            requests: list[MCTRequest] = list()
            for detector in mixer.detectors:
                requests.append(MixerIntrinsicUpdateRequest(
                    detector_label=detector.detector_label,
                    intrinsic_parameters=detector.intrinsic_parameters))
            requests.append(DequeueStatusMessagesRequest())
            self._send_request_series(
                component_label=mixer_label,
                requests=requests,
                callback=self._request_5_set_intrinsics_responded)

    def _request_5_set_intrinsics_responded(
        self,
        response_series: MCTResponseSeries,
        _passthrough_parameters: dict[str, ...]
    ):
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message="DetectorStartupSequencer._request_5_set_intrinsics_responded()")
        mixer_label: str = response_series.responder
        input_mixer_data: MixerStartupSequencer.InputMixerData = self._input_mixer_data_by_label[mixer_label]
        expected_types: list[type[MCTResponse]] = [EmptyResponse] * len(input_mixer_data.detectors)
        expected_types.append(DequeueStatusMessagesResponse)
        if self._report_response_series_and_errors(
            response_series=response_series,
            expected_types=expected_types
        ):
            return
        self.data_by_mixer_label[mixer_label].done_set_intrinsics = True
        if len(self._pending_request_ids) > 0:
            return
        self._request_6_set_targets()

    def _request_6_set_targets(self):
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message="MixerStartupSequencer._request_6_set_targets()")
        for mixer in self._input_mixer_data_by_label.values():
            mixer_label: str = mixer.mixer_label
            requests: list[MCTRequest] = [
                PoseSolverTargetsSetRequest(targets=mixer.targets),
                DequeueStatusMessagesRequest()]
            self._send_request_series(
                component_label=mixer_label,
                requests=requests,
                callback=self._request_6_set_targets_responded)

    def _request_6_set_targets_responded(
        self,
        response_series: MCTResponseSeries,
        _passthrough_parameters: dict[str, ...]
    ):
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message="DetectorStartupSequencer._request_6_set_targets_responded()")
        if self._report_response_series_and_errors(
            response_series=response_series,
            expected_types=[
                EmptyResponse,
                DequeueStatusMessagesResponse]
        ):
            return
        mixer_label: str = response_series.responder
        self.data_by_mixer_label[mixer_label].done_set_targets = True
        self._request_7_set_extrinsics()

    def _request_7_set_extrinsics(self):
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message="MixerStartupSequencer._request_7_set_extrinsics()")
        for mixer in self._input_mixer_data_by_label.values():
            mixer_label: str = mixer.mixer_label
            requests: list[MCTRequest] = list()
            for detector in mixer.detectors:
                # Even if no extrinsics are sent, it will request status (again).
                # It may not be the most efficient way to do things in that case,
                # but an extra request is a low price to pay for simpler implementation/maintenance.
                # TODO: Error checking and finish the implementation for the different modes
                if detector.pose_mode == DetectorPoseMode.STATIC_EXTERNAL:
                    requests.append(PoseSolverExtrinsicSetRequest(
                        detector_label=detector.detector_label,
                        transform_to_reference=detector.extrinsic_matrix))  # TODO: What if extrinsic_matrix is None?
            requests.append(DequeueStatusMessagesRequest())
            self._send_request_series(
                component_label=mixer_label,
                requests=requests,
                callback=self._request_7_set_extrinsics_responded)

    def _request_7_set_extrinsics_responded(
        self,
        response_series: MCTResponseSeries,
        _passthrough_parameters: dict[str, ...]
    ):
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message="MixerStartupSequencer._request_7_set_extrinsics_responded()")
        mixer_label: str = response_series.responder
        input_mixer_data: MixerStartupSequencer.InputMixerData = self._input_mixer_data_by_label[mixer_label]
        expected_extrinsic_sent_response_count: int = 0
        for detector in input_mixer_data.detectors:
            if detector.pose_mode == DetectorPoseMode.STATIC_EXTERNAL:
                expected_extrinsic_sent_response_count += 1
        expected_types: list[type[MCTResponse]] = [EmptyResponse] * expected_extrinsic_sent_response_count
        expected_types.append(DequeueStatusMessagesResponse)
        if self._report_response_series_and_errors(
            response_series=response_series,
            expected_types=expected_types
        ):
            return
        self.data_by_mixer_label[mixer_label].done_set_extrinsics = True
        if len(self._pending_request_ids) > 0:
            return


# noinspection DuplicatedCode
class DetectorFrameGetSequencer(AbstractSequencer):

    class OutputDetectorData:
        frame: DetectorFrame | None

        def __init__(self):
            self.frame = None

    data_by_detector_label: dict[str, OutputDetectorData]

    _include_annotations_detected: bool
    _include_annotations_rejected: bool
    _include_image: bool
    _requested_image_resolution: ImageResolution | None
    _requested_image_format: ImageFormat
    _on_frame_callback: Callable[[str, OutputDetectorData], None] | None

    def __init__(
        self,
        status_message_source: StatusMessageSource,
        connection_router: ConnectionRouter,
        callback_router: CallbackRouter
    ):
        super().__init__(
            status_message_source=status_message_source,
            connection_router=connection_router,
            callback_router=callback_router)
        self.data_by_detector_label = dict()
        self.reset()

    def begin(
        self,
        detector_labels: list[str],
        on_frame_callback: Callable[[str, OutputDetectorData], None],
        include_detected: bool = True,
        include_rejected: bool = False,
        include_image: bool = False,
        requested_image_format: ImageFormat = ImageFormat.FORMAT_JPG,
        requested_image_resolution: ImageResolution | None = None
    ) -> None:
        """
        :param detector_labels: Connections over which to communicate.
        :param on_frame_callback: What to do with the retrieved data.
        :param include_detected: Default True.
        :param include_rejected: Default False.
        :param include_image: Default False.
        :param requested_image_format: Either ".png" or ".jpg".
        :param requested_image_resolution: If not None, Detectors will scale images before sending.
        """
        if len(self._pending_request_ids) > 0:
            message: str = f"MixerStartupSequencer.begin() called when already busy. Try waiting or calling reset()."
            self._status_message_source.enqueue_status_message(severity=SeverityLabel.ERROR, message=message)
            return
        detector_count: int = len(detector_labels)
        if detector_count == 0:
            self._status_message_source.enqueue_status_message(
                severity=SeverityLabel.ERROR,
                message=f"No inputs were provided to DetectorFrameGetSequencer")
            return
        if not include_detected and not include_rejected and not include_image:
            self._status_message_source.enqueue_status_message(
                severity=SeverityLabel.ERROR,
                message=f"No outputs are being requested from Detector frames.")
            return
        self.reset()
        self._on_frame_callback = on_frame_callback
        self._include_annotations_detected = include_detected
        self._include_annotations_rejected = include_rejected
        self._include_image = include_image
        self._requested_image_format = requested_image_format
        self._requested_image_resolution = requested_image_resolution
        for detector_label in detector_labels:
            self.data_by_detector_label[detector_label] = DetectorFrameGetSequencer.OutputDetectorData()
            self._status_message_source.enqueue_status_message(
                severity=SeverityLabel.DEBUG,
                message=f"DetectorFrameGetSequencer - starting loop for detector {detector_label}")
            self._request_frame_get(detector_label=detector_label)

    def get_include_annotations_detected(self) -> bool:
        return self._include_annotations_detected

    def get_include_annotations_rejected(self) -> bool:
        return self._include_annotations_rejected

    def get_include_images(self) -> bool:
        return self._include_image

    def set_include_annotations_detected(self, enabled: bool) -> None:
        self._include_annotations_detected = enabled

    def set_include_annotations_rejected(self, enabled: bool) -> None:
        self._include_annotations_rejected = enabled

    def set_include_images(
        self,
        enabled: bool,
        image_format: ImageFormat = ImageFormat.FORMAT_JPG,
        image_resolution: ImageResolution | None = None
    ) -> None:
        self._include_image = enabled
        self._requested_image_format = image_format
        self._requested_image_resolution = image_resolution

    def reset(self) -> None:
        super().reset()
        self._include_annotations_detected = False
        self._include_annotations_rejected = False
        self._include_image = False
        self._requested_image_resolution = None
        self._requested_image_format = ImageFormat.FORMAT_PNG
        self._on_frame_callback = None
        self.data_by_detector_label.clear()

    def _request_frame_get(self, detector_label: str):
        self._send_request_series(
            component_label=detector_label,
            requests=[
                DetectorFrameGetRequest(
                    include_detected=self._include_annotations_detected,
                    include_rejected=self._include_annotations_rejected,
                    include_image=self._include_image,
                    image_format=self._requested_image_format,
                    image_resolution=self._requested_image_resolution),
                DequeueStatusMessagesRequest()],
            callback=self._request_frame_get_responded)

    def _request_frame_get_responded(
        self,
        response_series: MCTResponseSeries,
        _passthrough_parameters: dict[str, ...]
    ):
        if self._report_response_series_and_errors(
            response_series=response_series,
            expected_types=[
                DetectorFrameGetResponse,
                DequeueStatusMessagesResponse]
        ):
            return
        detector_label: str = response_series.responder
        detector_data: DetectorFrameGetSequencer.OutputDetectorData = self.data_by_detector_label[detector_label]
        # noinspection PyTypeChecker
        response: DetectorFrameGetResponse = response_series.series[0]
        detector_data.frame = response.frame
        if self._on_frame_callback is not None:
            self._on_frame_callback(detector_label, detector_data)
        self._request_frame_get(detector_label=detector_label)


# noinspection DuplicatedCode
class MixerFrameGetSequencer(AbstractSequencer):

    class OutputMixerData:
        frame: MixerFrame | None

        # These are intended primarily for internal state keeping, but they might be useful for deeper analyses too
        detector_labels_needing_frame_send: list[str]
        detector_labels_send_count_by_request: dict[uuid.UUID, int]

        def __init__(self):
            self.frame = None
            self.detector_labels_needing_frame_send = list()
            self.detector_labels_send_count_by_request = dict()

    _latest_frame_by_detector_label: dict[str, DetectorFrame]
    data_by_mixer_label: dict[str, OutputMixerData]

    _on_frame_callback: Callable[[str, OutputMixerData], None] | None

    def __init__(
        self,
        status_message_source: StatusMessageSource,
        connection_router: ConnectionRouter,
        callback_router: CallbackRouter
    ):
        super().__init__(
            status_message_source=status_message_source,
            connection_router=connection_router,
            callback_router=callback_router)
        self._latest_frame_by_detector_label = dict()
        self.data_by_mixer_label = dict()
        self.reset()

    def begin(
        self,
        mixer_labels: list[str],
        on_frame_callback: Callable[[str, OutputMixerData], None] | None
    ) -> None:
        if len(self._pending_request_ids) > 0:
            message: str = f"MixerStartupSequencer.begin() called when already busy. Try waiting or calling reset()."
            self._status_message_source.enqueue_status_message(severity=SeverityLabel.ERROR, message=message)
            return
        mixer_count: int = len(mixer_labels)
        if mixer_count == 0:
            self._status_message_source.enqueue_status_message(
                severity=SeverityLabel.ERROR,
                message=f"No inputs were provided to MixerFrameGetSequencer")
            return
        self.reset()
        self._on_frame_callback = on_frame_callback
        for mixer_label in mixer_labels:
            self.data_by_mixer_label[mixer_label] = MixerFrameGetSequencer.OutputMixerData()
            self._status_message_source.enqueue_status_message(
                severity=SeverityLabel.DEBUG,
                message=f"MixerFrameGetSequencer - starting loop for Mixer {mixer_label}")
            self._request_frame_get(mixer_label=mixer_label)

    def reset(self) -> None:
        super().reset()
        self._latest_frame_by_detector_label.clear()
        self.data_by_mixer_label.clear()
        self._on_frame_callback = None

    def set_latest_detector_frame(self, detector_label: str, frame: DetectorFrame) -> None:
        self._latest_frame_by_detector_label[detector_label] = frame
        for mixer_data in self.data_by_mixer_label.values():
            mixer_data.detector_labels_needing_frame_send.append(detector_label)

    def _request_frame_get(self, mixer_label: str) -> None:
        requests: list[MCTRequest] = list()
        mixer_data: MixerFrameGetSequencer.OutputMixerData = self.data_by_mixer_label[mixer_label]
        for detector_label in mixer_data.detector_labels_needing_frame_send:
            requests.append(PoseSolverDetectorFrameAddRequest(
                detector_label=detector_label,
                detector_frame=self._latest_frame_by_detector_label[detector_label]))
        detector_labels_send_count: int = len(requests)
        requests.append(MixerFrameGetRequest())
        requests.append(DequeueStatusMessagesRequest())
        request_id: uuid.UUID = self._send_request_series(
            component_label=mixer_label,
            requests=requests,
            callback=self._request_frame_get_responded)
        mixer_data.detector_labels_send_count_by_request[request_id] = detector_labels_send_count
        mixer_data.detector_labels_needing_frame_send.clear()

    def _request_frame_get_responded(
        self,
        response_series: MCTResponseSeries,
        _passthrough_parameters: dict[str, ...]
    ):
        mixer_label: str = response_series.responder
        mixer_data: MixerFrameGetSequencer.OutputMixerData = self.data_by_mixer_label[mixer_label]
        request_id: uuid.UUID = uuid.UUID(response_series.request_id)
        detector_labels_send_count: int = mixer_data.detector_labels_send_count_by_request[request_id]
        expected_types: list[type[MCTResponse]] = [EmptyResponse] * detector_labels_send_count
        expected_types.append(MixerFrameGetResponse)
        expected_types.append(DequeueStatusMessagesResponse)
        if self._report_response_series_and_errors(
            response_series=response_series,
            expected_types=expected_types
        ):
            return
        mixer_data.detector_labels_send_count_by_request.pop(request_id)
        # noinspection PyTypeChecker
        response: MixerFrameGetResponse = response_series.series[detector_labels_send_count]
        mixer_data.frame = response.frame
        if self._on_frame_callback is not None:
            self._on_frame_callback(mixer_label, mixer_data)
        self._request_frame_get(mixer_label=mixer_label)


class MixerCalibrationExtrinsicImageAddSequencer(AbstractSequencer):
    # Inputs
    _image_base64_by_detector_label: dict[str, str]
    _timestamp_utc_iso8601: str

    image_identifier_by_mixer_label: dict[str, str]  # Outputs

    _on_frame_callback: Callable[[str, list[str]], None] | None

    def __init__(
        self,
        status_message_source: StatusMessageSource,
        connection_router: ConnectionRouter,
        callback_router: CallbackRouter
    ):
        super().__init__(
            status_message_source=status_message_source,
            connection_router=connection_router,
            callback_router=callback_router)
        self._image_base64_by_detector_label = dict()
        self.image_identifier_by_mixer_label = dict()
        self.reset()

    def begin(
        self,
        mixer_labels: list[str],
        image_base64_by_detector_label: dict[str, str],
        timestamp_utc_iso8601: str,
        on_frame_callback: Callable[[str, list[str]], None] | None
    ) -> None:
        if len(self._pending_request_ids) > 0:
            message: str = f"MixerStartupSequencer.begin() called when already busy. Try waiting or calling reset()."
            self._status_message_source.enqueue_status_message(severity=SeverityLabel.ERROR, message=message)
            return
        mixer_count: int = len(mixer_labels)
        if mixer_count == 0:
            self._status_message_source.enqueue_status_message(
                severity=SeverityLabel.ERROR,
                message=f"No inputs were provided to MixerFrameGetSequencer")
            return
        self.reset()
        self._image_base64_by_detector_label = image_base64_by_detector_label
        self._timestamp_utc_iso8601 = timestamp_utc_iso8601
        self._on_frame_callback = on_frame_callback
        for mixer_label in mixer_labels:
            requests: list[MCTRequest] = list()
            for detector_label in self._image_base64_by_detector_label.keys():
                requests.append(ExtrinsicCalibrationImageAddRequest(
                    image_base64=self._image_base64_by_detector_label[detector_label],
                    detector_label=detector_label,
                    timestamp_utc_iso8601=self._timestamp_utc_iso8601))
            requests.append(DequeueStatusMessagesRequest())
            self._send_request_series(
                component_label=mixer_label,
                requests=requests,
                callback=self._responded)

    def reset(self) -> None:
        super().reset()
        self._image_base64_by_detector_label.clear()
        self.image_identifier_by_mixer_label.clear()
        self._timestamp_utc_iso8601 = str()
        self._on_frame_callback = None

    def _responded(
        self,
        response_series: MCTResponseSeries,
        _passthrough_parameters: dict[str, ...]
    ):
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message="MixerFrameGetSequencer._request_frame_get_responded()")
        mixer_label: str = response_series.responder
        detector_count: int = len(self._image_base64_by_detector_label)
        expected_types: list[type[MCTResponse]] = [ExtrinsicCalibrationImageAddResponse] * detector_count
        expected_types.append(DequeueStatusMessagesResponse)
        if self._report_response_series_and_errors(
            response_series=response_series,
            expected_types=expected_types
        ):
            return
        image_identifiers: list[str] = list()
        for detector_index in range(detector_count):
            # noinspection PyTypeChecker
            response: ExtrinsicCalibrationImageAddResponse = response_series.series[detector_index]
            image_identifiers.append(response.image_identifier)
        if self._on_frame_callback is not None:
            self._on_frame_callback(mixer_label, image_identifiers)


class AbstractSingleRoundTripSequencer(AbstractSequencer):
    """
    A common use case is to send a single message or an identical message over several connections.
    This is a convenience class that can be used in conjunction with a class factory,
    to avoid duplicated code and probably copy-paste and maintenance issues.

    The factory needs to assign _extractor_callback, _request, and _response_type:
    - _request_type is the type that is sent, constructed using the arguments in begin()
    - _response_type is the type that is normally expected in response (non-error case)
    - _extractor_callback extracts from the MCTResponse the parameters passed to the user's callback
    """

    _request_type: ClassVar[type[MCTRequest]]
    _response_type: ClassVar[type[MCTResponse]]
    _extractor_callback: ClassVar[Callable[[MCTResponse], dict[str, ...]] | None]

    _user_callback: Callable | None

    def begin(
        self,
        component_labels: list[str],
        callback: Callable | None = None,
        request_args: dict[str, ...] | None = None
    ) -> None:
        if len(self._pending_request_ids) > 0:
            message: str = \
                f"{__class__.__name__}.begin() called when requests are already in progress."
            self._status_message_source.enqueue_status_message(severity=SeverityLabel.ERROR, message=message)
            return
        component_count: int = len(component_labels)
        if component_count == 0:
            self._status_message_source.enqueue_status_message(
                severity=SeverityLabel.ERROR,
                message=f"No inputs were provided to {__class__.__name__}")
            return
        if request_args is None:
            request_args = dict()
        self.reset()
        self._user_callback = callback
        for component_label in component_labels:
            self._send_request_series(
                component_label=component_label,
                requests=[
                    self._request_type(**request_args),
                    DequeueStatusMessagesRequest()],
                callback=self._responded)

    def _responded(
        self,
        response_series: MCTResponseSeries,
        _passthrough_parameters: dict[str, ...]
    ):
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message=f"{__class__.__name__}._responded()")
        if self._report_response_series_and_errors(
            response_series=response_series,
            expected_types=[
                self.get_response_type(),
                DequeueStatusMessagesResponse]
        ):
            return
        if self._user_callback is not None:
            # Create a "shallow dump" of the model - basically we want to preserve the nested data structures
            keys: set[str] = set(response_series.series[0].__class__.model_fields.keys())
            kwargs: dict[str, ...] = {
                key: getattr(response_series.series[0], key) for key in keys if key != "parsable_type"}
            self._user_callback(component_label=response_series.responder, **kwargs)
        if len(self._pending_request_ids) > 0:
            return

    def reset(self) -> None:
        super().reset()
        self._user_callback = None

    @classmethod
    def get_request_type(cls) -> type[MCTRequest]:
        return cls._request_type

    @classmethod
    def get_response_type(cls) -> type[MCTResponse]:
        return cls._response_type

    @classmethod
    def _extractor_callback(cls) -> Callable[[MCTResponse], dict[str, ...]]:
        return cls._extractor_callback

    @staticmethod
    def create_subclass(
        class_name: str,
        request_type: type[MCTRequest],
        response_type: type[MCTResponse],
        extractor_callback: Callable[[MCTResponse], dict[str, ...]] | None = None
    ) -> type['AbstractSingleRoundTripSequencer']:
        # noinspection PyTypeChecker
        sequencer_class: type[AbstractSingleRoundTripSequencer] = type(
            class_name,
            (AbstractSingleRoundTripSequencer,),
            {
                "_request_type": request_type,
                "_response_type": response_type,
                "_extractor_callback": extractor_callback
            })
        return sequencer_class

DetectorShutdownSequencer: type[AbstractSingleRoundTripSequencer] = \
    AbstractSingleRoundTripSequencer.create_subclass(
        class_name="DetectorShutdownSequencer",
        request_type=DetectorStopRequest,
        response_type=EmptyResponse)
MixerShutdownSequencer: type[AbstractSingleRoundTripSequencer] = \
    AbstractSingleRoundTripSequencer.create_subclass(
        class_name="MixerShutdownSequencer",
        request_type=MixerStopRequest,
        response_type=EmptyResponse)

DetectorParametersGetSequencer: type[AbstractSingleRoundTripSequencer] = \
    AbstractSingleRoundTripSequencer.create_subclass(
        class_name="DetectorParametersGetSequencer",
        request_type=DetectorParametersGetRequest,
        response_type=DetectorParametersGetResponse)
DetectorParametersSetSequencer: type[AbstractSingleRoundTripSequencer] = \
    AbstractSingleRoundTripSequencer.create_subclass(
        class_name="DetectorParametersSetSequencer",
        request_type=DetectorParametersSetRequest,
        response_type=DetectorParametersSetResponse)

DetectorCalibrationIntrinsicCalculateSequencer: type[AbstractSingleRoundTripSequencer] = \
    AbstractSingleRoundTripSequencer.create_subclass(
        class_name="DetectorCalibrationIntrinsicCalculateSequencer",
        request_type=IntrinsicCalibrationCalculateRequest,
        response_type=IntrinsicCalibrationCalculateResponse)
DetectorCalibrationIntrinsicDeleteStagedSequencer: type[AbstractSingleRoundTripSequencer] = \
    AbstractSingleRoundTripSequencer.create_subclass(
        class_name="DetectorCalibrationIntrinsicDeleteStagedSequencer",
        request_type=IntrinsicCalibrationDeleteStagedRequest,
        response_type=EmptyResponse)
DetectorCalibrationIntrinsicImageAddSequencer: type[AbstractSingleRoundTripSequencer] = \
    AbstractSingleRoundTripSequencer.create_subclass(
        class_name="DetectorCalibrationIntrinsicImageAddSequencer",
        request_type=IntrinsicCalibrationImageAddRequest,
        response_type=IntrinsicCalibrationImageAddResponse)
DetectorCalibrationIntrinsicImageGetSequencer: type[AbstractSingleRoundTripSequencer] = \
    AbstractSingleRoundTripSequencer.create_subclass(
        class_name="DetectorCalibrationIntrinsicImageGetSequencer",
        request_type=IntrinsicCalibrationImageGetRequest,
        response_type=IntrinsicCalibrationImageGetResponse)
DetectorCalibrationIntrinsicImageMetadataUpdateSequencer: type[AbstractSingleRoundTripSequencer] = \
    AbstractSingleRoundTripSequencer.create_subclass(
        class_name="DetectorCalibrationIntrinsicImageMetadataUpdateSequencer",
        request_type=IntrinsicCalibrationImageMetadataUpdateRequest,
        response_type=EmptyResponse)
DetectorCalibrationIntrinsicMetadataListSequencer: type[AbstractSingleRoundTripSequencer] = \
    AbstractSingleRoundTripSequencer.create_subclass(
        class_name="DetectorCalibrationIntrinsicMetadataListSequencer",
        request_type=IntrinsicCalibrationMetadataListRequest,
        response_type=IntrinsicCalibrationMetadataListResponse)
DetectorCalibrationIntrinsicResolutionListSequencer: type[AbstractSingleRoundTripSequencer] = \
    AbstractSingleRoundTripSequencer.create_subclass(
        class_name="DetectorCalibrationIntrinsicResolutionListSequencer",
        request_type=IntrinsicCalibrationResolutionListRequest,
        response_type=IntrinsicCalibrationResolutionListResponse)
DetectorCalibrationIntrinsicResultGetSequencer: type[AbstractSingleRoundTripSequencer] = \
    AbstractSingleRoundTripSequencer.create_subclass(
        class_name="DetectorCalibrationIntrinsicResultGetSequencer",
        request_type=IntrinsicCalibrationResultGetRequest,
        response_type=IntrinsicCalibrationResultGetResponse)
DetectorCalibrationIntrinsicResultGetActiveSequencer: type[AbstractSingleRoundTripSequencer] = \
    AbstractSingleRoundTripSequencer.create_subclass(
        class_name="DetectorCalibrationIntrinsicResultGetActiveSequencer",
        request_type=IntrinsicCalibrationResultGetActiveRequest,
        response_type=IntrinsicCalibrationResultGetActiveResponse)
DetectorCalibrationIntrinsicResultMetadataUpdateSequencer: type[AbstractSingleRoundTripSequencer] = \
    AbstractSingleRoundTripSequencer.create_subclass(
        class_name="DetectorCalibrationIntrinsicResultMetadataUpdateSequencer",
        request_type=IntrinsicCalibrationResultMetadataUpdateRequest,
        response_type=EmptyResponse)

MixerCalibrationExtrinsicCalculateSequencer: type[AbstractSingleRoundTripSequencer] = \
    AbstractSingleRoundTripSequencer.create_subclass(
        class_name="MixerCalibrationExtrinsicCalculateSequencer",
        request_type=ExtrinsicCalibrationCalculateRequest,
        response_type=ExtrinsicCalibrationCalculateResponse)
MixerCalibrationExtrinsicDeleteStagedSequencer: type[AbstractSingleRoundTripSequencer] = \
    AbstractSingleRoundTripSequencer.create_subclass(
        class_name="MixerCalibrationExtrinsicDeleteStagedSequencer",
        request_type=ExtrinsicCalibrationDeleteStagedRequest,
        response_type=EmptyResponse)
MixerCalibrationExtrinsicImageGetSequencer: type[AbstractSingleRoundTripSequencer] = \
    AbstractSingleRoundTripSequencer.create_subclass(
        class_name="MixerCalibrationExtrinsicImageGetSequencer",
        request_type=ExtrinsicCalibrationImageGetRequest,
        response_type=ExtrinsicCalibrationImageGetResponse)
MixerCalibrationExtrinsicImageMetadataListSequencer: type[AbstractSingleRoundTripSequencer] = \
    AbstractSingleRoundTripSequencer.create_subclass(
        class_name="MixerCalibrationExtrinsicImageMetadataListSequencer",
        request_type=ExtrinsicCalibrationImageMetadataListRequest,
        response_type=ExtrinsicCalibrationImageMetadataListResponse)
MixerCalibrationExtrinsicImageMetadataUpdateSequencer: type[AbstractSingleRoundTripSequencer] = \
    AbstractSingleRoundTripSequencer.create_subclass(
        class_name="MixerCalibrationExtrinsicImageMetadataUpdateSequencer",
        request_type=ExtrinsicCalibrationImageMetadataUpdateRequest,
        response_type=EmptyResponse)
MixerCalibrationExtrinsicResultGetActiveSequencer: type[AbstractSingleRoundTripSequencer] = \
    AbstractSingleRoundTripSequencer.create_subclass(
        class_name="MixerCalibrationExtrinsicResultGetActiveSequencer",
        request_type=ExtrinsicCalibrationResultGetActiveRequest,
        response_type=ExtrinsicCalibrationResultGetActiveResponse)
MixerCalibrationExtrinsicResultGetSequencer: type[AbstractSingleRoundTripSequencer] = \
    AbstractSingleRoundTripSequencer.create_subclass(
        class_name="MixerCalibrationExtrinsicResultGetSequencer",
        request_type=ExtrinsicCalibrationResultGetRequest,
        response_type=ExtrinsicCalibrationResultGetResponse)
MixerCalibrationExtrinsicResultMetadataListSequencer: type[AbstractSingleRoundTripSequencer] = \
    AbstractSingleRoundTripSequencer.create_subclass(
        class_name="MixerCalibrationExtrinsicResultMetadataListSequencer",
        request_type=ExtrinsicCalibrationResultMetadataListRequest,
        response_type=ExtrinsicCalibrationResultMetadataListResponse)
MixerCalibrationExtrinsicResultMetadataUpdateSequencer: type[AbstractSingleRoundTripSequencer] = \
    AbstractSingleRoundTripSequencer.create_subclass(
        class_name="MixerCalibrationExtrinsicResultMetadataUpdateSequencer",
        request_type=ExtrinsicCalibrationResultMetadataUpdateRequest,
        response_type=EmptyResponse)
AnyUserInitiatedSequencer: type[AbstractSingleRoundTripSequencer] = Union[
    DetectorCalibrationIntrinsicCalculateSequencer,
    DetectorCalibrationIntrinsicDeleteStagedSequencer,
    DetectorCalibrationIntrinsicImageAddSequencer,
    DetectorCalibrationIntrinsicImageGetSequencer,
    DetectorCalibrationIntrinsicMetadataListSequencer,
    DetectorCalibrationIntrinsicImageMetadataUpdateSequencer,
    DetectorCalibrationIntrinsicResolutionListSequencer,
    DetectorCalibrationIntrinsicResultGetSequencer,
    DetectorCalibrationIntrinsicResultGetActiveSequencer,
    DetectorCalibrationIntrinsicResultMetadataUpdateSequencer,
    MixerCalibrationExtrinsicCalculateSequencer,
    MixerCalibrationExtrinsicDeleteStagedSequencer,
    MixerCalibrationExtrinsicImageAddSequencer, \
    MixerCalibrationExtrinsicImageGetSequencer,
    MixerCalibrationExtrinsicImageMetadataListSequencer,
    MixerCalibrationExtrinsicImageMetadataUpdateSequencer,
    MixerCalibrationExtrinsicResultGetSequencer,
    MixerCalibrationExtrinsicResultGetActiveSequencer,
    MixerCalibrationExtrinsicResultMetadataListSequencer,
    MixerCalibrationExtrinsicResultMetadataUpdateSequencer]
