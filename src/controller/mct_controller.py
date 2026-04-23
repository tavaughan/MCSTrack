from .configuration import \
    MCTComponentConfig, \
    MCTConfiguration
from .connection import \
    Connection
from .routing import \
    CallbackRouter, \
    ConnectionRouter
from .sequencing import \
    AnyUserInitiatedSequencer, \
    DetectorCalibrationIntrinsicCalculateSequencer, \
    DetectorCalibrationIntrinsicDeleteStagedSequencer, \
    DetectorCalibrationIntrinsicImageAddSequencer, \
    DetectorCalibrationIntrinsicImageGetSequencer, \
    DetectorCalibrationIntrinsicImageMetadataUpdateSequencer, \
    DetectorCalibrationIntrinsicMetadataListSequencer, \
    DetectorCalibrationIntrinsicResolutionListSequencer, \
    DetectorCalibrationIntrinsicResultGetSequencer, \
    DetectorCalibrationIntrinsicResultGetActiveSequencer, \
    DetectorCalibrationIntrinsicResultMetadataUpdateSequencer, \
    DetectorFrameGetSequencer, \
    DetectorParametersGetSequencer, \
    DetectorParametersSetSequencer, \
    DetectorShutdownSequencer, \
    DetectorStartupSequencer, \
    MixerCalibrationExtrinsicCalculateSequencer, \
    MixerCalibrationExtrinsicDeleteStagedSequencer, \
    MixerCalibrationExtrinsicImageAddSequencer, \
    MixerCalibrationExtrinsicImageGetSequencer, \
    MixerCalibrationExtrinsicImageMetadataUpdateSequencer, \
    MixerCalibrationExtrinsicMetadataListSequencer, \
    MixerCalibrationExtrinsicResultGetActiveSequencer, \
    MixerCalibrationExtrinsicResultGetSequencer, \
    MixerCalibrationExtrinsicResultMetadataUpdateSequencer, \
    MixerFrameGetSequencer, \
    MixerShutdownSequencer, \
    MixerStartupSequencer, \
    TimeSyncSequencer
from src.common import \
    BaseSink, \
    CSVPoseSink, \
    DetectorFrame, \
    DetectorPoseMode, \
    ExtrinsicCalibration, \
    ExtrinsicCalibrator, \
    ImageFormat, \
    ImageResolution, \
    IntrinsicCalibration, \
    IntrinsicCalibrator, \
    IntrinsicParameters, \
    KeyValueMetaAny, \
    KeyValueSimpleAny, \
    MCTRequest, \
    MCTRequestSeries, \
    MCTResponseSeries, \
    MixerFrame, \
    SeverityLabel, \
    StatusMessageSource
from src.detector import DETECTOR_RESPONSE_TYPES
from src.mixer import MIXER_RESPONSE_TYPES
import datetime
from enum import StrEnum
import hjson
from ipaddress import IPv4Address
import logging
import os
from pydantic import ValidationError
from typing import Callable
import uuid


logger = logging.getLogger(__name__)


class _TemporalOffset:
    """
    A remote component might not be in complete sync with this controller.
    Causes include network latency and different clock times.
    This class is for storing these quantities.
    """
    network_latency_milliseconds: int
    clock_offset_milliseconds: int  # how much time to be ADDED to go from controller time to component
    def __init__(
        self,
        network_latency_milliseconds: int,
        clock_offset_milliseconds: int
    ):
        self.network_latency_milliseconds = network_latency_milliseconds
        self.clock_offset_milliseconds = clock_offset_milliseconds


class _DetectorLiveData:
    """
    Live data received from Detectors every frame when the controller is running.
    If an individual field is None, then that information either has not been received yet,
    or the current configuration or settings in the controller mean that it is not being requested.
    """
    temporal_offset: _TemporalOffset | None
    camera_resolution: ImageResolution | None
    camera_parameters: list[KeyValueMetaAny] | None
    annotator_parameters: list[KeyValueMetaAny] | None
    intrinsic_calibration: IntrinsicCalibration | None
    frame: DetectorFrame | None
    def __init__(self):
        self.temporal_offset = None
        self.camera_resolution = None
        self.camera_parameters = None
        self.annotator_parameters = None
        self.intrinsic_calibration = None
        self.frame = None


class _MixerLiveData:
    """
    Live data received from Mixers every frame when the controller is running.
    If an individual field is None, then that information either has not been received yet,
    or the current configuration or settings in the controller mean that it is not being requested.
    """
    temporal_offset: _TemporalOffset | None
    pose_solver_parameters: list[KeyValueMetaAny] | None
    frame: MixerFrame | None
    def __init__(self):
        self.temporal_offset = None
        self.pose_solver_parameters = None
        self.frame = None


class MCTController:
    """
    This is the main class to interface with MCSTrack.
    Typical usage is to first prepare a configuration file/data structure,
    then provide it to the configure() function.
    This will load and organize data internally without initiating any connections.
    When the user is ready, start_up() can be called to begin any configured Detectors and Mixers.
    Various data and setting can be manipulated using the other functions in this class.
    To stop MCSTrack cleanly, call shut_down().

    When requesting information from, or sending instructions to, remote components,
    the effect is not going to be immediate.
    Information needs to be transmitted to and from the remote component.
    In these cases, there is almost always a built-in means to have a
    callback function called once the response is received.
    See individual function documentation for details on the expected signatures of these callbacks.
    """

    DetectorLiveData: type[_DetectorLiveData] = _DetectorLiveData
    MixerLiveData: type[_MixerLiveData] = _MixerLiveData
    TemporalOffset: type[_TemporalOffset] = _TemporalOffset

    class State(StrEnum):
        """
        State of the controller. Descriptions below:
        - INITIAL: Not running, not transitioning to or from the running state. Not configured.
        - CONFIGURED: Not running, nor transitioning to or from the running state. Ready to start.
        - CONNECTING: First step during startup. Establish connections with remote components.
        - STARTING: Remote components are connected, but information is still being transferred to/from for startup.
        - RUNNING: Receiving frames from Detectors and Mixers, accepting instructions from the user.
        - STARTING: Remote components are connected, but information is still being transferred to/from for shutdown.
        - CONNECTING: Final step during shutdown. Terminate connections with remote components.
        """
        INITIAL = "Initial"
        CONFIGURED = "Configured"
        CONNECTING = "Connecting"
        STARTING = "Starting"
        RUNNING = "Running"
        STOPPING = "Stopping"
        DISCONNECTING = "Disconnecting"

    class _Sequencers:
        """
        Sequencers are objects responsible for preparing information/instructions for remote components,
        sending said information/instructions, and handling the information that is received in response.
        """
        time_sync_sequencer: TimeSyncSequencer | None
        detector_startup_sequencer: DetectorStartupSequencer | None
        detector_frame_get_sequencer: DetectorFrameGetSequencer | None
        detector_shutdown_sequencer: DetectorShutdownSequencer | None
        mixer_startup_sequencer: MixerStartupSequencer | None
        mixer_frame_get_sequencer: MixerFrameGetSequencer | None
        mixer_shutdown_sequencer: MixerShutdownSequencer | None
        user_sequencer: AnyUserInitiatedSequencer | None  # For user-specified operations
        def __init__(self):
            self.reset()
        def reset(self):
            self.time_sync_sequencer = None
            self.detector_startup_sequencer = None
            self.detector_frame_get_sequencer = None
            self.detector_shutdown_sequencer = None
            self.mixer_startup_sequencer = None
            self.mixer_frame_get_sequencer = None
            self.mixer_shutdown_sequencer = None
            self.user_sequencer = None

    _status_message_source: StatusMessageSource
    _sink_type_registry: dict[str, type[BaseSink]]

    _state: State
    _configuration: MCTConfiguration | None
    _sequencers: _Sequencers
    _detector_live_data: dict[str, DetectorLiveData | None]
    _mixer_live_data: dict[str, MixerLiveData | None]
    _sinks: list[BaseSink]
    _connection_router: ConnectionRouter
    _callback_router: CallbackRouter

    def __init__(
        self,
        controller_name: str = "mct_controller",
        send_status_messages_to_logger: bool = False
    ):
        """
        Create the controller.
        :param controller_name: When this instance logs information, associate with this name.
        :param send_status_messages_to_logger: Log messages to python's logging module.
        """
        self._status_message_source = StatusMessageSource(
            source_label=controller_name,
            send_to_logger=send_status_messages_to_logger)
        self._sink_type_registry = {"csv": CSVPoseSink}
        self._state = MCTController.State.INITIAL
        self._configuration = None
        self._sequencers = MCTController._Sequencers()
        self._detector_live_data = dict()
        self._mixer_live_data = dict()
        self._sinks = list()
        self._connection_router = ConnectionRouter()
        self._callback_router = CallbackRouter(status_message_source=self._status_message_source)

    # =================================================================================================================
    #                                              HIGH-LEVEL CONTROL
    # =================================================================================================================

    def register_sink_type(self, implementation_str: str, sink_type: type[BaseSink]) -> None:
        self._sink_type_registry[implementation_str] = sink_type

    def configure(
        self,
        configuration: MCTConfiguration
    ) -> bool:
        """
        Specify the Detectors and Mixers, their connection information (IP address), their settings,
        what algorithms are used with what parameters,
        what are the targets being tracked and their configurations, etc.
        This function MUST be called before start_up().
        If the controller is already running,
        then shut_down() should be called first to prevent an inconsistent state.
        :param configuration: The data structure specifying the configuration
        :returns: True if there are no immediate errors.
        """
        if not (self._state == MCTController.State.INITIAL or self._state == MCTController.State.CONFIGURED):
            self._status_message_source.enqueue_status_message(
                severity=SeverityLabel.ERROR,
                message="Cannot load configuration if not in an idle state.")
            return False

        configured: bool = True
        self._configuration = configuration

        # Create connections
        def is_valid_ip_address(connection: MCTComponentConfig) -> bool:
            try:
                IPv4Address(connection.ip_address)
            except ValueError:
                self._status_message_source.enqueue_status_message(
                    severity=SeverityLabel.ERROR,
                    message=f"Connection {connection.label} has invalid IP {connection.ip_address}. Skipping.")
                return False
            if connection.port < 0 or connection.port > 65535:
                self._status_message_source.enqueue_status_message(
                    severity=SeverityLabel.ERROR,
                    message=f"Connection {connection.label} has invalid port {connection.port}. "
                            "It will be skipped.")
                return False
            return True

        for detector in self._configuration.detectors:
            if not is_valid_ip_address(detector):
                configured = False
                continue
            if detector.label in self._detector_live_data or detector.label in self._mixer_live_data:
                self._status_message_source.enqueue_status_message(
                    severity=SeverityLabel.ERROR,
                    message=f"Detector Configuration contains an existing label {detector.label}. Skipping.")
                configured = False
                continue
            component_address: Connection.ComponentAddress = Connection.ComponentAddress(
                label=detector.label,
                role="detector",
                ip_address=IPv4Address(detector.ip_address),
                port=detector.port)
            self._connection_router.add_connection(
                component_address=component_address,
                supported_response_types=DETECTOR_RESPONSE_TYPES,
                status_message_source=self._status_message_source)
            self._detector_live_data[component_address.label] = _DetectorLiveData()

        for mixer in self._configuration.mixers:
            if not is_valid_ip_address(mixer):
                configured = False
                continue
            if mixer.label in self._detector_live_data or mixer.label in self._mixer_live_data:
                self._status_message_source.enqueue_status_message(
                    severity=SeverityLabel.ERROR,
                    message=f"Mixer Configuration contains an existing label {mixer.label}. Skipping.")
                configured = False
                continue
            component_address: Connection.ComponentAddress = Connection.ComponentAddress(
                label=mixer.label,
                role="mixer",
                ip_address=IPv4Address(mixer.ip_address),
                port=mixer.port)
            self._connection_router.add_connection(
                component_address=component_address,
                supported_response_types=MIXER_RESPONSE_TYPES,
                status_message_source=self._status_message_source)
            self._mixer_live_data[component_address.label] = _MixerLiveData()

        for sink_configuration in self._configuration.sinks:
            if sink_configuration.implementation not in self._sink_type_registry:
                self._status_message_source.enqueue_status_message(
                    severity=SeverityLabel.ERROR,
                    message=f"Unrecognized sink implementation {sink_configuration.implementation}. Skipping.")
                configured = False
                continue
            sink_type: type[BaseSink] = self._sink_type_registry[sink_configuration.implementation]
            sink: BaseSink = sink_type(**sink_configuration.configuration)
            self._sinks.append(sink)

        if not configured:
            self.reset()
        else:
            self._state = MCTController.State.CONFIGURED

        return configured

    def configure_from_filepath(
        self,
        input_configuration_filepath: str
    ) -> bool:
        """
        Convenience function to load a configuration from a file then call configure() with its contents.
        :param input_configuration_filepath:
        :returns: True if there are no immediate errors.
        """
        if not os.path.exists(input_configuration_filepath):
            self._status_message_source.enqueue_status_message(
                severity=SeverityLabel.ERROR,
                message=f"File {input_configuration_filepath} does not exist. Configuration will not be loaded.")
            return False
        if not os.path.isfile(input_configuration_filepath):
            self._status_message_source.enqueue_status_message(
                severity=SeverityLabel.ERROR,
                message=f"File {input_configuration_filepath} is not a file. Configuration will not be loaded.")
            return False
        self.reset()
        configuration_dict: dict
        with open(input_configuration_filepath, 'r') as infile:
            configuration_dict = hjson.loads(infile.read())
        configuration: MCTConfiguration
        try:
            configuration = MCTConfiguration(**configuration_dict)
        except ValidationError as e:
            self._status_message_source.enqueue_status_message(
                severity=SeverityLabel.ERROR,
                message=f"Failed to load configuration file {input_configuration_filepath}. Error: {e}")
            return False
        return self.configure(configuration)

    def start_up(self) -> bool:
        """
        Launch any configured Detectors and Mixers.
        This will begin a series of interactions between the controller and the other components.
        You can check the current status of the controller by calling get_controller_state().
        Start up will be finished when the controller is RUNNING.
        :returns: True if there are no immediate errors.
        """
        if self._configuration is None:
            self._status_message_source.enqueue_status_message(
                severity=SeverityLabel.ERROR,
                message="Cannot start up if no configuration has been loaded.")
            return False
        if self._state != MCTController.State.CONFIGURED:
            self._status_message_source.enqueue_status_message(
                severity=SeverityLabel.ERROR,
                message="Cannot start up if not configured and idle. Has the controller already been started?")
            return False
        if len(self._configuration.detectors) <= 0:
            self._status_message_source.enqueue_status_message(
                severity=SeverityLabel.ERROR,
                message="No detectors were specified in the configuration.")
            return False
        self._connection_router.start_up()
        self._state = MCTController.State.CONNECTING
        return True

    def shut_down(self) -> bool:
        """
        Stop Detectors and Mixers.
        This will begin a series of interactions between the controller and the other components.
        You can check the current status of the controller by calling get_controller_state().
        Shut down will be finished when the controller is IDLE.
        :returns: True if there are no immediate errors.
        """
        if self._state != MCTController.State.RUNNING:
            self._status_message_source.enqueue_status_message(
                severity=SeverityLabel.ERROR,
                message="Cannot shut down if not in the running state. Try checking the status first?")
            return False
        self._state = MCTController.State.STOPPING
        return True

    def reset(self):
        """
        Force the controller back to its initial state.
        Normally this should not be necessary when using configure(), start_up(), and shut_down() methods,
        but this function is provided as a fallback in case something goes wrong.
        This will begin a series of interactions between the controller and the other components.
        You can check the current status of the controller by calling get_controller_state().
        Shut down will be finished when the controller is IDLE.
        :returns: True if there are no immediate errors.
        """
        if self._state == MCTController.State.RUNNING:
            self._connection_router.shut_down()
        self._state = MCTController.State.INITIAL
        self._configuration = None
        self._sequencers.reset()
        self._detector_live_data.clear()
        self._mixer_live_data.clear()
        self._sinks.clear()
        self._callback_router.reset()
        self._connection_router.reset()

    def update(self) -> None:
        """
        This function is responsible for updating the internal state and data
        of the controller based on communications with the other components.
        It is for things that should be done frequently and regularly -
        basically everything that should occur "per frame" in an update loop.
        Right now the controller doesn't update on its own -
        this function must be called externally and frequently.
        If it does not get called, then the controller will not do anything meaningful.
        """

        self._connection_router.update()

        if self._state == MCTController.State.INITIAL or self._state == MCTController.State.CONFIGURED:
            return

        if self._state == MCTController.State.DISCONNECTING:
            if self._connection_router.is_shut_down_finished():
                self._state = MCTController.State.CONFIGURED
                self._sequencers.reset()
                return
            else:
                return

        if self._state == MCTController.State.CONNECTING:
            if not self._connection_router.is_start_up_finished():
                return
            self._state = MCTController.State.STARTING
            self._sequencers.time_sync_sequencer = TimeSyncSequencer(**self._sequencer_init_args())
            self._sequencers.time_sync_sequencer.begin(component_labels=self.get_remote_labels())

        # !! NO RETURN STATEMENTS AFTER THIS POINT !! - we want to make sure callback code at the end runs

        if self._state == MCTController.State.STARTING:
            if (
                self._sequencers.time_sync_sequencer is not None and
                self._sequencers.time_sync_sequencer.is_finished() and
                self._sequencers.detector_startup_sequencer is None
            ):
                for detector_label, detector_live_data in self._detector_live_data.items():
                    component_data: TimeSyncSequencer.ComponentData = \
                        self._sequencers.time_sync_sequencer.data_by_component_label[detector_label]
                    detector_live_data.temporal_offset = _TemporalOffset(
                        network_latency_milliseconds=component_data.network_latency_milliseconds,
                        clock_offset_milliseconds=component_data.clock_offset_milliseconds)
                for mixer_label, mixer_live_data in self._mixer_live_data.items():
                    component_data: TimeSyncSequencer.ComponentData = \
                        self._sequencers.time_sync_sequencer.data_by_component_label[mixer_label]
                    mixer_live_data.temporal_offset = _TemporalOffset(
                        network_latency_milliseconds=component_data.network_latency_milliseconds,
                        clock_offset_milliseconds=component_data.clock_offset_milliseconds)
                self._sequencers.detector_startup_sequencer = DetectorStartupSequencer(**self._sequencer_init_args())
                self._sequencers.detector_startup_sequencer.begin(
                    detector_data=[DetectorStartupSequencer.InputDetectorData(
                        detector_label=detector.label,
                        camera_parameters=detector.camera_parameters,
                        annotator_parameters=detector.annotator_parameters)
                    for detector in self._configuration.detectors])
            if (
                self._sequencers.detector_startup_sequencer is not None and
                self._sequencers.detector_startup_sequencer.is_finished() and
                self._sequencers.detector_frame_get_sequencer is None and
                self._sequencers.mixer_startup_sequencer is None
            ):
                for detector_label, detector_live_data in self._detector_live_data.items():
                    detector_data: DetectorStartupSequencer.DetectorData = \
                        self._sequencers.detector_startup_sequencer.data_by_detector_label[detector_label]
                    detector_live_data.camera_parameters = detector_data.camera_parameters
                    detector_live_data.annotator_parameters = detector_data.annotator_parameters
                    detector_live_data.camera_resolution = detector_data.camera_resolution
                    detector_live_data.intrinsic_calibration = detector_data.intrinsic_calibration
                self._sequencers.detector_frame_get_sequencer = DetectorFrameGetSequencer(**self._sequencer_init_args())
                self._sequencers.detector_frame_get_sequencer.begin(
                    detector_labels=list(self._detector_live_data.keys()),
                    on_frame_callback=self._on_detector_frame_received)
                if len(self._configuration.mixers) <= 0:
                    self._state = MCTController.State.RUNNING
                else:
                    detector_labels_unmatched: set[str] = set()  # Non-empty indicates clear configuration error
                    detector_labels_without_intrinsic_calibration: set[str] = set()
                    detector_labels_without_extrinsic_calibration: set[str] = set()
                    intrinsics_by_detector_label: dict[str, IntrinsicParameters] = dict()
                    for mixer in self._configuration.mixers:
                        for detector in mixer.detectors:
                            detector_label: str = detector.detector_label
                            if detector_label in self._detector_live_data:
                                if detector_label not in intrinsics_by_detector_label:
                                    detector_live_data: _DetectorLiveData = self._detector_live_data[detector_label]
                                    if detector_live_data.intrinsic_calibration is not None:
                                        # Easy case, expected most of the time
                                        intrinsics_by_detector_label[detector_label] = \
                                            detector_live_data.intrinsic_calibration.calibrated_values
                                    else:
                                        detector_labels_without_intrinsic_calibration.add(detector_label)
                                        intrinsics_by_detector_label[detector_label] = \
                                            IntrinsicParameters.generate_zero_parameters(
                                                resolution_x_px=detector_live_data.camera_resolution.x_px,
                                                resolution_y_px=detector_live_data.camera_resolution.y_px)
                                if detector.pose_mode == DetectorPoseMode.STATIC_EXTERNAL:
                                    if detector.detector_to_reference is None:
                                        detector_labels_without_extrinsic_calibration.add(detector_label)
                            else:
                                detector_labels_unmatched.add(detector_label)
                    if len(detector_labels_unmatched) > 0:
                        message: str = \
                            "Mixers refer to non-existing detectors. " + \
                            "Please check the configuration for these detector labels: " + \
                            ", ".join(detector_labels_unmatched)
                        self._status_message_source.enqueue_status_message(
                            severity=SeverityLabel.ERROR,  # FOR SURE an error
                            message=message)
                    if len(detector_labels_without_intrinsic_calibration) > 0:
                        message: str = \
                            "Intrinsic parameters were missing for the following detectors, " + \
                            "and default \"zero\" parameters were used. " + \
                            "This may be normal if an intrinsic calibration has not yet been done. " + \
                            "Detector labels. " + ", ".join(detector_labels_without_intrinsic_calibration)
                        self._status_message_source.enqueue_status_message(
                            severity=SeverityLabel.WARNING,
                            message=message)
                    if len(detector_labels_without_extrinsic_calibration) > 0:
                        message: str = \
                            "Extrinsic parameters were missing in at least one place for some detectors, " + \
                            "and default \"identity\" matrices were used. " + \
                            "This may be normal if an extrinsic calibration has not yet been done. " + \
                            "Detector labels: " + ", ".join(detector_labels_without_extrinsic_calibration)
                        self._status_message_source.enqueue_status_message(
                            severity=SeverityLabel.WARNING,
                            message=message)
                    self._sequencers.mixer_startup_sequencer = MixerStartupSequencer(**self._sequencer_init_args())
                    self._sequencers.mixer_startup_sequencer.begin(
                        mixer_data=[MixerStartupSequencer.InputMixerData(
                            mixer_label=mixer.label,
                            detectors=[MixerStartupSequencer.InputMixerData.Detector(
                                detector_label=detector.detector_label,
                                intrinsic_parameters=intrinsics_by_detector_label[detector.detector_label],
                                pose_mode=detector.pose_mode,
                                extrinsic_matrix=detector.detector_to_reference)
                                for detector in mixer.detectors],
                            targets=mixer.targets,
                            solver_parameters=mixer.solver_parameters)
                            for mixer in self._configuration.mixers])
            if (
                self._sequencers.mixer_startup_sequencer is not None and
                self._sequencers.mixer_startup_sequencer.is_finished()
            ):
                self._sequencers.mixer_frame_get_sequencer = MixerFrameGetSequencer(**self._sequencer_init_args())
                self._sequencers.mixer_frame_get_sequencer.begin(
                    mixer_labels=list(self._mixer_live_data.keys()),
                    on_frame_callback=self._on_mixer_frame_received)
                self._state = MCTController.State.RUNNING

        if self._state == MCTController.State.STOPPING:
            if not self.is_user_task_running():
                if (
                    self._sequencers.detector_shutdown_sequencer is None
                ):
                    self._sequencers.detector_shutdown_sequencer = \
                        DetectorShutdownSequencer(**self._sequencer_init_args())
                    self._sequencers.detector_shutdown_sequencer.begin(
                        component_labels=list(self._detector_live_data.keys()))
                if (
                    len(self._configuration.mixers) > 0 and
                    self._sequencers.mixer_shutdown_sequencer is None
                ):
                    self._sequencers.mixer_shutdown_sequencer = \
                        MixerShutdownSequencer(**self._sequencer_init_args())
                    self._sequencers.mixer_shutdown_sequencer.begin(
                        component_labels=list(self._mixer_live_data.keys()))
                if (
                    self._sequencers.detector_shutdown_sequencer is not None and
                    self._sequencers.detector_shutdown_sequencer.is_finished() and
                    len(self._configuration.mixers) <= 0 or (
                        self._sequencers.mixer_shutdown_sequencer is not None and
                        self._sequencers.mixer_shutdown_sequencer.is_finished())
                ):
                    self._connection_router.shut_down()
                    self._state = MCTController.State.DISCONNECTING

        response_series_lists: list[list[MCTResponseSeries]] = self._connection_router.dequeue_response_series_lists()
        for response_series_list in response_series_lists:
            for response_series in response_series_list:
                try:
                    self._callback_router.handle_callback(response_series)
                except Exception as e:
                    message: str = \
                        f"Exception occurred on handling response from {response_series.responder}: " + \
                        f"{e.__class__.__name__} {e}"
                    self._status_message_source.enqueue_status_message(
                        severity=SeverityLabel.ERROR,
                        message=message)

    def get_detector_includes_annotations_detected(self) -> bool:
        """
        :returns: True if Detectors are currently getting detected annotations, else False.
        """
        if not self._state == MCTController.State.RUNNING:
            self._status_message_source.enqueue_status_message(
                severity=SeverityLabel.WARNING,
                message="Requesting frame properties when Detectors are not running. Call start_up() first.")
            return False
        return self._sequencers.detector_frame_get_sequencer.get_include_annotations_detected()

    def get_detector_includes_annotations_rejected(self) -> bool:
        """
        :returns: True if Detectors are currently getting rejected annotations, else False.
        """
        if not self._state == MCTController.State.RUNNING:
            self._status_message_source.enqueue_status_message(
                severity=SeverityLabel.WARNING,
                message="Requesting frame properties when Detectors are not running. Call start_up() first.")
            return False
        return self._sequencers.detector_frame_get_sequencer.get_include_annotations_rejected()

    def get_detector_includes_images(self) -> bool:
        """
        :returns: True if Detectors are currently getting images, else False.
        """
        if not self._state == MCTController.State.RUNNING:
            self._status_message_source.enqueue_status_message(
                severity=SeverityLabel.WARNING,
                message="Requesting frame properties when Detectors are not running. Call start_up() first.")
            return False
        return self._sequencers.detector_frame_get_sequencer.get_include_images()

    def set_detector_includes_annotations_detected(self, enabled: bool) -> bool:
        """
        Indicate to Detectors to NOT include in their frames identified annotations.
        :returns: True if there were no immediate errors. False if unable to apply the setting.
        """
        if not self._state == MCTController.State.RUNNING:
            self._status_message_source.enqueue_status_message(
                severity=SeverityLabel.ERROR,
                message="Cannot change frame properties until the controller is running. Call start_up() first.")
            return True
        self._sequencers.detector_frame_get_sequencer.set_include_annotations_detected(enabled=enabled)
        return True

    def set_detector_includes_annotations_rejected(self, enabled: bool) -> bool:
        """
        Indicate to Detectors to NOT include in their frames unidentified annotations.
        :returns: True if there were no immediate errors. False if unable to apply the setting.
        """
        if not self._state == MCTController.State.RUNNING:
            self._status_message_source.enqueue_status_message(
                severity=SeverityLabel.ERROR,
                message="Cannot change frame properties until the controller is running. Call start_up() first.")
            return True
        self._sequencers.detector_frame_get_sequencer.set_include_annotations_rejected(enabled=enabled)
        return True

    def set_detector_includes_images(
        self,
        enabled: bool,
        image_format: ImageFormat = ImageFormat.FORMAT_JPG,
        image_resolution: ImageResolution | None = None
    ) -> bool:
        """
        Indicate to Detectors that they shall NOT include in their frames its camera image.
        :returns: True if there were no immediate errors. False if unable to apply the setting.
        """
        if not self._state == MCTController.State.RUNNING:
            self._status_message_source.enqueue_status_message(
                severity=SeverityLabel.ERROR,
                message="Cannot change frame properties until the controller is running. Call start_up() first.")
            return True
        self._sequencers.detector_frame_get_sequencer.set_include_images(
            enabled=enabled,
            image_format=image_format,
            image_resolution=image_resolution)
        return True

    def _sequencer_init_args(self):
        """
        Convenience function for common arguments in sequencers
        (Sequencers assemble, send, and handle communications with other components)
        """
        return {
            "status_message_source": self._status_message_source,
            "connection_router": self._connection_router,
            "callback_router": self._callback_router}

    def _on_detector_frame_received(
        self,
        detector_label: str,
        detector_data: DetectorFrameGetSequencer.OutputDetectorData
    ):
        """
        Internal handling for receiving Detector frames (annotations and sometimes the associated image).
        """
        self._detector_live_data[detector_label].frame = detector_data.frame
        if self._sequencers.mixer_frame_get_sequencer is not None:
            self._sequencers.mixer_frame_get_sequencer.set_latest_detector_frame(
                detector_label=detector_label,
                frame=DetectorFrame(
                    annotations=detector_data.frame.annotations,
                    timestamp_utc_iso8601=detector_data.frame.timestamp_utc_iso8601,
                    image_resolution=detector_data.frame.image_resolution,
                    image_base64=None))  # Explicitly exclude image; even if it is present, the mixer doesn't need it

    def _on_mixer_frame_received(
        self,
        mixer_label: str,
        mixer_data: MixerFrameGetSequencer.OutputMixerData
    ) -> None:
        """
        Internal handling for receiving Mixer frames (tracked poses).
        """
        self._mixer_live_data[mixer_label].frame = mixer_data.frame

    # =================================================================================================================
    #                                          DATA ACCESS/MANIPULATION
    # =================================================================================================================

    def get_configuration(self) -> MCTConfiguration:
        """
        :returns: A (deep) copy of the configuration.
        """
        return self._configuration.model_copy(deep=True)

    def get_controller_state(self) -> State:
        """
        The state of the controller can be useful information to help troubleshoot issues.
        Descriptions of the different states are documented with the data structure.
        :returns: state of the controller.
        """
        return self._state

    def get_connection_reports(self) -> list[Connection.Report]:
        """
        Get information on individual connections with remote connections.
        May be useful for troubleshooting.
        :returns: A list of reports (data structure) containing information about each connection.
        """
        return self._connection_router.get_connection_reports()

    def get_live_detector_data(
        self,
        detector_label: str
    ) -> DetectorLiveData | None:
        """
        Retrieve a direct reference to the latest information from the indicated Detector.
        Such information may include annotations (feature coordinates from images) and images,
        depending on current settings.
        The returned data structure may contain only None values if no data has been received.
        :param detector_label: From configuration.
        :returns: Data structure containing most recent data.
        :raises IndexError: If the Detector does not exist.
        """
        if detector_label not in self._detector_live_data:
            raise IndexError()
        return self._detector_live_data[detector_label]

    def get_live_mixer_data(
        self,
        mixer_label: str
    ) -> MixerLiveData | None:
        """
        Retrieve a direct reference to the latest information from the indicated Mixer.
        Such information may include tracked poses.
        The returned data structure may contain only None values if no data has been received.
        :param mixer_label: From configuration.
        :returns: Data structure containing most recent data.
        :raises IndexError: If the Mixer does not exist.
        """
        if mixer_label not in self._mixer_live_data:
            raise IndexError()
        return self._mixer_live_data[mixer_label]

    def get_remote_labels(self) -> list[str]:
        """
        Get the labels for *all* configured Detectors and Mixers.
        :returns: A list of str
        """
        return self.get_remote_labels_detectors() + self.get_remote_labels_mixer()

    def get_remote_labels_detectors(self) -> list[str]:
        """
        Get the labels for configured Detectors.
        :returns: A list of str
        """
        return list(self._detector_live_data.keys())

    def get_remote_labels_mixer(self) -> list[str]:
        """
        Get the labels for configured Mixers.
        :returns: A list of str
        """
        return list(self._mixer_live_data.keys())

    def get_status_message_source(self) -> StatusMessageSource:
        """
        Applications are expected to use this instance to retrieve messages from remote components.
        :returns: The StatusMessageSource that reports both local and remote messages.
        """
        return self._status_message_source

    def is_user_task_running(self) -> bool:
        """
        At the time of writing, only one user-specified remote communication operation is allowed at a time.
        This helps to prevent inconsistent states.
        :returns: True if a user-specified operation is in progress; False otherwise.
        """
        if self._sequencers.user_sequencer is None:
            return False
        return not self._sequencers.user_sequencer.is_finished()

    def send_custom_request(
        self,
        component_label: str,
        requests: list[MCTRequest],
        callback: CallbackRouter.CallbackFunction = None,
        passthrough_arguments: dict[str, ...] | None = None
    ) -> uuid.UUID:
        """
        ADVANCED USE ONLY. May be removed in the future.

        This is a way for users to build their own requests and handle responses in the exact way that they want.
        This kind of manipulation requires deep knowledge of what types of MCTRequest objects are supported by the
        remote component, and also the types and content of MCTResponse that may be returned.
        Sending certain MCTRequest types (especially Start and Stop requests)
        may break state-related assumptions that are made in update(),
        or cause the live data stored in this class to become invalid or out-of-date.
        If another function (or series of functions) already exists to achieve something,
        then it is strongly recommended to use it (or them) instead.

        :param component_label: The label associated with the remote component.
        :param requests: Requests to send to the remote component.
        :param callback: Callback function to be called once the remote component responds.
        :param passthrough_arguments:
        :returns: A unique ID associated with the request.
        """
        # TODO: We probably want to deprecate this one
        request_series = MCTRequestSeries(series=requests)
        request_id: uuid.UUID = uuid.UUID(request_series.request_id)
        self._connection_router.enqueue_request_series(
            label=component_label,
            request_series=request_series)
        if callback is not None:
            if passthrough_arguments is None:
                passthrough_arguments = dict()
            self._callback_router.add_callback(
                request_id=request_id,
                callback=callback,
                passthrough_arguments=passthrough_arguments)
        return request_id

    # =================================================================================================================
    #                                                 CALIBRATION
    # =================================================================================================================
    # This section of code is intended to provide a "nice" interface
    # to the more-involved parameter-tweaking or calibration operations,
    # which can involve a lot of back-and-forth communication.
    # Yes, it's long. Yes, it's repetitive. Yes, it's ugly.
    # But this will probably help IDEs with static analysis
    # and greatly simplify the interface for the end-user
    # compared to a more "clever" or generic solution.

    def _user_task_can_proceed_including_error_report(self) -> bool:
        """
        Indicate if it is safe to start a user-specified task.
        If there is any reason why it is not safe to start a user-specified task,
        then it will get logged as an error in the StatusMessageSource.
        :returns: True if it is safe, False otherwise.
        """
        if self._state != MCTController.State.RUNNING:
            self._status_message_source.enqueue_status_message(
                severity=SeverityLabel.ERROR,
                message=f"Cannot send calibration request if the controller is not running.")
            return False
        if self.is_user_task_running():
            self._status_message_source.enqueue_status_message(
                severity=SeverityLabel.ERROR,
                message=f"Cannot send calibration request while another task is already active.")
            return False
        return True

    def detector_parameters_get(
        self,
        detector_label: str,
        callback: Callable[[str, ImageResolution, list[KeyValueMetaAny], list[KeyValueMetaAny]], None] | None = None
    ) -> bool:
        """
        Start a specific user-initiated task. Check is_busy_with_user_task() before calling.

        This function may be called to get a list of Detector parameters and valid values.
        To modify these, see detector_parameter_set().

        :param detector_label: label to which this shall apply
        :param callback: Callback args:
            0 - component_label: str (detector)
            1 - camera_resolution: ImageResolution
            2 - camera_parameters: list[KeyValueMetaAny]
            2 - annotator_parameters: list[KeyValueMetaAny]
        :returns: True if no errors immediately occurred and the request was sent.
        """
        if not self._user_task_can_proceed_including_error_report():
            return False
        self._sequencers.user_sequencer = \
            DetectorParametersGetSequencer(**self._sequencer_init_args())
        self._sequencers.user_sequencer.begin(
            component_labels=[detector_label],
            callback=callback)
        return True

    def detector_parameters_set(
        self,
        detector_label: str,
        camera_resolution: ImageResolution | None = None,
        camera_parameters: list[KeyValueSimpleAny] | None = None,
        annotator_parameters: list[KeyValueSimpleAny] | None = None,
        callback: Callable[[str, ImageResolution, list[KeyValueMetaAny], list[KeyValueMetaAny]], None] | None = None
    ) -> bool:
        """
        Start a specific user-initiated task. Check is_busy_with_user_task() before calling.

        This function may be called to attempt setting a list of camera parameters.
        The Detector will report updated settings (and valid values) after the attempt.
        The user is encouraged to review these to ensure parameters were set as desired.
        In order to first discover valid values, see detector_parameters_get().

        :param detector_label: label to which this shall apply
        :param camera_resolution:
        :param camera_parameters:
        :param annotator_parameters:
        :param callback: Callback args:
            0 - component_label: str (detector)
            1 - camera_resolution: ImageResolution
            2 - camera_parameters: list[KeyValueMetaAny]
            3 - annotator_parameters: list[KeyValueMetaAny]
        :returns: True if no errors immediately occurred and the request was sent.
        """
        if not self._user_task_can_proceed_including_error_report():
            return False
        self._sequencers.user_sequencer = \
            DetectorParametersSetSequencer(**self._sequencer_init_args())
        self._sequencers.user_sequencer.begin(
            component_labels=[detector_label],
            callback=callback,
            request_args={
                "camera_resolution": camera_resolution,
                "camera_parameters": camera_parameters,
                "annotator_parameters": annotator_parameters})
        return True

    def calibrate_intrinsic_calculate(
        self,
        detector_label: str,
        image_resolution: ImageResolution,
        callback: Callable[[str, str, IntrinsicCalibration], None] | None = None
    ) -> bool:
        """
        Start a specific user-initiated task. Check is_busy_with_user_task() before calling.

        This function should be called after several images have been taken and stored by the indicated Detector,
        consistent with its calibration method.
        To take these images, see calibrate_intrinsic_image_add().
        The Detector will create a new "result", which stores the resulting calibration.
        The Detector will return a unique identifier that is associated with the result,
        and can be used to manipulate data remotely.

        :param detector_label: label to which this shall apply
        :param image_resolution: Resolution for which to calibrate (different resolutions do not mix)
        :param callback: Callback args:
            0 - component_label: str (detector)
            1 - result_identifier: str
            2 - intrinsic_calibration: IntrinsicCalibration
        :returns: True if no errors immediately occurred and the request was sent.
        """
        if not self._user_task_can_proceed_including_error_report():
            return False
        self._sequencers.user_sequencer = \
            DetectorCalibrationIntrinsicCalculateSequencer(**self._sequencer_init_args())
        self._sequencers.user_sequencer.begin(
            component_labels=[detector_label],
            callback=callback,
            request_args={"image_resolution": image_resolution})
        # TODO: If a new "Active" Calibration is set, then Mixer(s) need to be notified
        return True

    def calibrate_intrinsic_delete_staged(
        self,
        detector_label: str,
        callback: Callable[[str], None] | None = None
    ) -> bool:
        """
        Start a specific user-initiated task. Check is_busy_with_user_task() before calling.

        Deletes images or results stored remotely in the Detector if they have been staged for deletion.
        Images can be staged for deletion using calibrate_intrinsic_image_metadata_update().
        Results can be staged for deletion using calibrate_intrinsic_result_metadata_update().

        :param detector_label: label to which this shall apply
        :param callback: Callback args:
            0 - component_label: str (detector)
        :returns: True if no errors immediately occurred and the request was sent.
        """
        if not self._user_task_can_proceed_including_error_report():
            return False
        self._sequencers.user_sequencer = \
            DetectorCalibrationIntrinsicDeleteStagedSequencer(**self._sequencer_init_args())
        self._sequencers.user_sequencer.begin(
            component_labels=[detector_label],
            callback=callback)
        return True

    def calibrate_intrinsic_image_add(
        self,
        detector_label: str,
        callback: Callable[[str, str], None] | None = None
    ) -> bool:
        """
        Start a specific user-initiated task. Check is_busy_with_user_task() before calling.

        Indicates to the Detector to immediately capture an image and store it for calibration.
        The Detector will return a unique identifier that is associated with the image,
        and can be used to manipulate data remotely.
        Note that no image is sent from the controller; the Detector will use its own most recent image.

        :param detector_label: label to which this shall apply
        :param callback: Callback args:
            0 - component_label: str (detector)
            1 - image_identifier: str
        :returns: True if no errors immediately occurred and the request was sent.
        """
        if not self._user_task_can_proceed_including_error_report():
            return False
        self._sequencers.user_sequencer = \
            DetectorCalibrationIntrinsicImageAddSequencer(**self._sequencer_init_args())
        self._sequencers.user_sequencer.begin(
            component_labels=[detector_label],
            callback=callback)
        return True

    def calibrate_intrinsic_image_get(
        self,
        detector_label: str,
        image_identifier: str,
        callback: Callable[[str, str], None] | None = None
    ) -> bool:
        """
        Start a specific user-initiated task. Check is_busy_with_user_task() before calling.

        The Detector will return the image file (as a base64 str) associated with the provided identifier.

        :param detector_label: label to which this shall apply
        :param image_identifier: Retrieved either by a list operation or after an add operation
        :param callback: Callback args:
            0 - component_label: str (detector)
            1 - image_base64: str
        :returns: True if no errors immediately occurred and the request was sent.
        """
        if not self._user_task_can_proceed_including_error_report():
            return False
        self._sequencers.user_sequencer = \
            DetectorCalibrationIntrinsicImageGetSequencer(**self._sequencer_init_args())
        self._sequencers.user_sequencer.begin(
            component_labels=[detector_label],
            callback=callback,
            request_args={"image_identifier": image_identifier})
        return True

    def calibrate_intrinsic_image_metadata_update(
        self,
        detector_label: str,
        image_identifier: str,
        image_state: IntrinsicCalibrator.ImageState,
        image_label: str | None,
        callback: Callable[[str], None] | None = None
    ) -> bool:
        """
        Start a specific user-initiated task. Check is_busy_with_user_task() before calling.

        Update the Detector's metadata related to the image associated with the indicated label.
        The update may indicate to ignore the image for future calibrations, or to stage it for deletion,
        Image identifiers may be retrieved using calibrate_intrinsic_image_metadata_list().
        Images can be obtained (for viewing) using calibrate_intrinsic_image_get().

        :param detector_label: label to which this shall apply
        :param image_identifier:
        :param image_state:
        :param image_label:
        :param callback: Callback args:
            0 - component_label: str (detector)
        :returns: True if no errors immediately occurred and the request was sent.
        """
        if not self._user_task_can_proceed_including_error_report():
            return False
        self._sequencers.user_sequencer = \
            DetectorCalibrationIntrinsicImageMetadataUpdateSequencer(**self._sequencer_init_args())
        self._sequencers.user_sequencer.begin(
            component_labels=[detector_label],
            callback=callback,
            request_args={
                "image_identifier": image_identifier,
                "image_state": str(image_state),
                "image_label": image_label})
        return True

    def calibrate_intrinsic_metadata_list(
        self,
        detector_label: str,
        image_resolution: ImageResolution,
        callback: Callable[[
                str,
                list[IntrinsicCalibrator.ImageMetadata],
                list[IntrinsicCalibrator.ResultMetadata]
            ],None] | None = None
    ) -> bool:
        """
        Start a specific user-initiated task. Check is_busy_with_user_task() before calling.

        The Detector will return a list of data about the images that have been captured for calibration,
        as well as the calibrations themselves.
        Among these data there will be a unique identifier for each image and for each result (calibration),
        as well as their resolutions.
        The images themselves are NOT returned since there may be many, and they may be large.

        :param detector_label: label to which this shall apply
        :param image_resolution: Resolution for which to calibrate (different resolutions do not mix)
        :param callback: Callback args:
            0 - component_label: str (detector)
            1 - image_metadata_list: list[IntrinsicCalibrator.ImageMetadata]
            1 - result_metadata_list: list[IntrinsicCalibrator.ResultMetadata]
        :returns: True if no errors immediately occurred and the request was sent.
        """
        if not self._user_task_can_proceed_including_error_report():
            return False
        self._sequencers.user_sequencer = \
            DetectorCalibrationIntrinsicMetadataListSequencer(**self._sequencer_init_args())
        self._sequencers.user_sequencer.begin(
            component_labels=[detector_label],
            callback=callback,
            request_args={"image_resolution": image_resolution})
        return True

    def calibrate_intrinsic_resolution_list(
        self,
        detector_label: str,
        callback: Callable[[str, list[ImageResolution]], None] | None = None
    ) -> bool:
        """
        Start a specific user-initiated task. Check is_busy_with_user_task() before calling.

        The Detector will return a list of ImageResolutions for which it has calibration data.
        An intrinsic calibration is specific to the resolution of the images used to calculate it.
        The reason for this is that, even with the same physical,
        different resolutions may use different parts of the imaging sensor.

        :param detector_label: label to which this shall apply
        :param callback: Callback args:
            0 - component_label: str (detector)
            1 - resolutions: list[ImageResolution]
        :returns: True if no errors immediately occurred and the request was sent.
        """
        if not self._user_task_can_proceed_including_error_report():
            return False
        self._sequencers.user_sequencer = \
            DetectorCalibrationIntrinsicResolutionListSequencer(**self._sequencer_init_args())
        self._sequencers.user_sequencer.begin(
            component_labels=[detector_label],
            callback=callback)
        return True

    def calibrate_intrinsic_result_get(
        self,
        detector_label: str,
        result_identifier: str,
        callback: Callable[[str, IntrinsicCalibration], None] | None = None
    ) -> bool:
        """
        Start a specific user-initiated task. Check is_busy_with_user_task() before calling.

        The Detector will return the calibration associated with the provided identifier.
        To get an identifier, see calibrate_intrinsic_result_metadata_list().

        :param detector_label: label to which this shall apply
        :param result_identifier: Retrieved either by a list operation or after calibration
        :param callback: Callback args:
            0 - component_label: str (detector)
            1 - intrinsic_calibration: IntrinsicCalibration
        :returns: True if no errors immediately occurred and the request was sent.
        """
        if not self._user_task_can_proceed_including_error_report():
            return False
        self._sequencers.user_sequencer = \
            DetectorCalibrationIntrinsicResultGetSequencer(**self._sequencer_init_args())
        self._sequencers.user_sequencer.begin(
            component_labels=[detector_label],
            callback=callback,
            request_args={"result_identifier": result_identifier})
        return True

    def calibrate_intrinsic_result_get_active(
        self,
        detector_label: str,
        callback: Callable[[str, IntrinsicCalibration | None], None] | None = None
    ) -> bool:
        """
        Start a specific user-initiated task. Check is_busy_with_user_task() before calling.

        The Detector will return the calibration associated with the current resolution.
        This differs from the non "_active" version because it does not require a result_identifier.
        Instead, this method relies on the Detector keeping track
        of the most recently-"active" calibration for its current resolution.
        If there is no "active" calibration (possibly because it has never been calibrated),
        then the Detector will return None.

        :param detector_label: label to which this shall apply
        :param callback: Callback args:
            0 - component_label: str (detector)
            1 - intrinsic_calibration: IntrinsicCalibration | None
        :returns: True if no errors immediately occurred and the request was sent.
        """
        if not self._user_task_can_proceed_including_error_report():
            return False
        self._sequencers.user_sequencer = \
            DetectorCalibrationIntrinsicResultGetActiveSequencer(**self._sequencer_init_args())
        self._sequencers.user_sequencer.begin(
            component_labels=[detector_label],
            callback=callback)
        return True

    def calibrate_intrinsic_result_metadata_update(
        self,
        detector_label: str,
        result_identifier: str,
        result_state: IntrinsicCalibrator.ResultState,
        result_label: str | None,
        callback: Callable[[str], None] | None = None
    ) -> bool:
        """
        Start a specific user-initiated task. Check is_busy_with_user_task() before calling.

        Update the Detector's metadata related to the result associated with the indicated label.
        The update may indicate to use a particular result as the active calibration.
        Result identifiers may be retrieved using calibrate_intrinsic_result_metadata_list().
        Calibrations can be obtained using calibrate_intrinsic_result_get().

        :param detector_label: label to which this shall apply
        :param result_identifier:
        :param result_state:
        :param result_label:
        :param callback: Callback args:
            0 - component_label: str (detector)
        :returns: True if no errors immediately occurred and the request was sent.
        """
        if not self._user_task_can_proceed_including_error_report():
            return False
        self._sequencers.user_sequencer = \
            DetectorCalibrationIntrinsicResultMetadataUpdateSequencer(**self._sequencer_init_args())
        self._sequencers.user_sequencer.begin(
            component_labels=[detector_label],
            callback=callback,
            request_args={
                "result_identifier": result_identifier,
                "result_state": str(result_state),
                "result_label": result_label})
        # TODO: If a new "Active" Calibration is set, then Mixer(s) need to be notified
        return True

    def calibrate_extrinsic_calculate(
        self,
        mixer_label: str,
        callback: Callable[[str, str, ExtrinsicCalibration], None] | None = None
    ) -> bool:
        """
        Start a specific user-initiated task. Check is_busy_with_user_task() before calling.

        This function should be called after several images have been taken and stored by the indicated Mixer,
        consistent with its calibration method.
        To take these images, see calibrate_extrinsic_image_add().
        The Mixer will create a new "result", which stores the resulting calibration.
        The Mixer will return a unique identifier that is associated with the result,
        and can be used to manipulate data remotely.

        :param mixer_label: label to which this shall apply
        :param callback: Callback args:
            0 - component_label: str (mixer)
            1 - result_identifier: str
            2 - extrinsic_calibration: ExtrinsicCalibration
        :returns: True if no errors immediately occurred and the request was sent.
        """
        if not self._user_task_can_proceed_including_error_report():
            return False
        self._sequencers.user_sequencer = \
            MixerCalibrationExtrinsicCalculateSequencer(**self._sequencer_init_args())
        self._sequencers.user_sequencer.begin(
            component_labels=[mixer_label],
            callback=callback)
        return True

    def calibrate_extrinsic_delete_staged(
        self,
        mixer_label: str,
        callback: Callable[[str], None] | None = None
    ) -> bool:
        """
        Start a specific user-initiated task. Check is_busy_with_user_task() before calling.

        Deletes images or results stored remotely in the Mixer if they have been staged for deletion.
        Images can be staged for deletion using calibrate_extrinsic_image_metadata_update().
        Results can be staged for deletion using calibrate_extrinsic_result_metadata_update().

        :param mixer_label: label to which this shall apply
        :param callback: Callback args:
            0 - component_label: str (mixer)
        :returns: True if no errors immediately occurred and the request was sent.
        """
        if not self._user_task_can_proceed_including_error_report():
            return False
        self._sequencers.user_sequencer = \
            MixerCalibrationExtrinsicDeleteStagedSequencer(**self._sequencer_init_args())
        self._sequencers.user_sequencer.begin(
            component_labels=[mixer_label],
            callback=callback)
        return True

    def calibrate_extrinsic_image_add(
        self,
        mixer_label: str,
        callback: Callable[[str, list[str]], None] | None = None
    ) -> bool:
        """
        Start a specific user-initiated task. Check is_busy_with_user_task() before calling.

        Send current images from EACH Detector to indicated Mixer for future calibration.
        The Mixer will return unique identifiers that are associated with the images,
        and can be used to manipulate data remotely.

        :param mixer_label: label to which this shall apply
        :param callback: Callback args:
            0 - component_label: str (mixer)
            1 - image_identifiers: list[str]
        :returns: True if no errors immediately occurred and the request was sent.
        """
        if not self._user_task_can_proceed_including_error_report():
            return False
        for detector_label, detector_data in self._detector_live_data.items():
            detector_data: _DetectorLiveData
            if detector_data.frame.image_base64 is None:
                self._status_message_source.enqueue_status_message(
                    severity=SeverityLabel.ERROR,
                    message=f"Images are not available for detector {detector_label} - are images being collected?")
                return False
        self._sequencers.user_sequencer = \
            MixerCalibrationExtrinsicImageAddSequencer(**self._sequencer_init_args())
        # noinspection PyArgumentList
        self._sequencers.user_sequencer.begin(
            mixer_labels=[mixer_label],
            image_base64_by_detector_label={
                detector_label: detector_data.frame.image_base64
                for detector_label, detector_data in self._detector_live_data.items()},
            timestamp_utc_iso8601=datetime.datetime.now(tz=datetime.timezone.utc).isoformat(),
            on_frame_callback=callback)
        return True

    def calibrate_extrinsic_image_get(
        self,
        mixer_label: str,
        image_identifier: str,
        callback: Callable[[str, str], None] | None = None
    ) -> bool:
        """
        Start a specific user-initiated task. Check is_busy_with_user_task() before calling.

        The Mixer will return the image file (as a base64 str) associated with the provided identifier.

        :param mixer_label: label to which this shall apply
        :param image_identifier: Retrieved either by a list operation or after an add operation
        :param callback: Callback args:
            0 - component_label: str (mixer)
            1 - image_base64: str
        :returns: True if no errors immediately occurred and the request was sent.
        """
        if not self._user_task_can_proceed_including_error_report():
            return False
        self._sequencers.user_sequencer = \
            MixerCalibrationExtrinsicImageGetSequencer(**self._sequencer_init_args())
        self._sequencers.user_sequencer.begin(
            component_labels=[mixer_label],
            callback=callback,
            request_args={"image_identifier": image_identifier})
        return True

    def calibrate_extrinsic_image_metadata_update(
        self,
        mixer_label: str,
        image_identifier: str,
        image_state: ExtrinsicCalibrator.ImageState,
        image_label: str | None,
        callback: Callable[[str], None] | None = None
    ) -> bool:
        """
        Start a specific user-initiated task. Check is_busy_with_user_task() before calling.

        Update the Mixer's metadata related to the image associated with the indicated label.
        The update may indicate to ignore the image for future calibrations, or to stage it for deletion,
        Image identifiers may be retrieved using calibrate_extrinsic_image_metadata_list().
        Images can be obtained (for viewing) using calibrate_extrinsic_image_get().

        :param mixer_label: label to which this shall apply
        :param image_identifier:
        :param image_state:
        :param image_label:
        :param callback: Callback args:
            0 - component_label: str (mixer)
        :returns: True if no errors immediately occurred and the request was sent.
        """
        if not self._user_task_can_proceed_including_error_report():
            return False
        self._sequencers.user_sequencer = \
            MixerCalibrationExtrinsicImageMetadataUpdateSequencer(**self._sequencer_init_args())
        self._sequencers.user_sequencer.begin(
            component_labels=[mixer_label],
            callback=callback,
            request_args={
                "image_identifier": image_identifier,
                "image_state": str(image_state),
                "image_label": image_label})
        return True

    def calibrate_extrinsic_metadata_list(
        self,
        mixer_label: str,
        callback: Callable[[
                str,
                list[ExtrinsicCalibrator.ImageMetadata],
                list[ExtrinsicCalibrator.ResultMetadata]],
            None] | None = None
    ) -> bool:
        """
        Start a specific user-initiated task. Check is_busy_with_user_task() before calling.

        The Mixer will return a list of data about the images and calibrations that have been created.
        Among these data there will be a unique identifier for each image and for each result (calibration).
        The images themselves are NOT returned since there may be many, and they may be large.

        :param mixer_label: label to which this shall apply
        :param callback: Callback args:
            0 - component_label: str (mixer)
            1 - image_metadata_list: list[ExtrinsicCalibrator.ImageMetadata]
            1 - result_metadata_list: list[ExtrinsicCalibrator.ResultMetadata]
        :returns: True if no errors immediately occurred and the request was sent.
        """
        if not self._user_task_can_proceed_including_error_report():
            return False
        self._sequencers.user_sequencer = \
            MixerCalibrationExtrinsicMetadataListSequencer(**self._sequencer_init_args())
        self._sequencers.user_sequencer.begin(
            component_labels=[mixer_label],
            callback=callback)
        return True

    def calibrate_extrinsic_result_get(
        self,
        mixer_label: str,
        result_identifier: str,
        callback: Callable[[str, ExtrinsicCalibration], None] | None = None
    ) -> bool:
        """
        Start a specific user-initiated task. Check is_busy_with_user_task() before calling.

        The Mixer will return the calibration associated with the provided identifier.
        To get an identifier, see calibrate_extrinsic_result_metadata_list().

        :param mixer_label: label to which this shall apply
        :param result_identifier: Retrieved either by a list operation or after calibration
        :param callback: Callback args:
            0 - component_label: str (mixer)
            1 - extrinsic_calibration: ExtrinsicCalibration
        :returns: True if no errors immediately occurred and the request was sent.
        """
        if not self._user_task_can_proceed_including_error_report():
            return False
        self._sequencers.user_sequencer = \
            MixerCalibrationExtrinsicResultGetSequencer(**self._sequencer_init_args())
        self._sequencers.user_sequencer.begin(
            component_labels=[mixer_label],
            callback=callback,
            request_args={"result_identifier": result_identifier})
        return True

    def calibrate_extrinsic_result_get_active(
        self,
        mixer_label: str,
        callback: Callable[[str, ExtrinsicCalibration | None], None] | None = None
    ) -> bool:
        """
        Start a specific user-initiated task. Check is_busy_with_user_task() before calling.

        The Mixer will return the calibration currently indicated as active.
        This differs from the non "_active" version because it does not require a result_identifier.
        Instead, this method relies on the Mixer keeping track of the most recently-"active" calibration.
        If there is no "active" calibration (possibly because it has never been calibrated),
        then the Mixer will return None.

        :param mixer_label: label to which this shall apply
        :param callback: Callback args:
            0 - component_label: str (mixer)
            1 - extrinsic_calibration: ExtrinsicCalibration | None
        :returns: True if no errors immediately occurred and the request was sent.
        """
        if not self._user_task_can_proceed_including_error_report():
            return False
        self._sequencers.user_sequencer = \
            MixerCalibrationExtrinsicResultGetActiveSequencer(**self._sequencer_init_args())
        self._sequencers.user_sequencer.begin(
            component_labels=[mixer_label],
            callback=callback)
        return True

    def calibrate_extrinsic_result_metadata_update(
        self,
        mixer_label: str,
        result_identifier: str,
        result_state: ExtrinsicCalibrator.ResultState,
        result_label: str | None,
        callback: Callable[[str], None] | None = None
    ) -> bool:
        """
        Start a specific user-initiated task. Check is_busy_with_user_task() before calling.

        Update the Mixer's metadata related to the result associated with the indicated label.
        The update may indicate to use a particular result as the active calibration.
        Result identifiers may be retrieved using calibrate_extrinsic_result_metadata_list().
        Calibrations can be obtained using calibrate_extrinsic_result_get().

        :param mixer_label: label to which this shall apply
        :param result_identifier:
        :param result_state:
        :param result_label:
        :param callback: Callback args:
            0 - component_label: str (mixer)
        :returns: True if no errors immediately occurred and the request was sent.
        """
        if not self._user_task_can_proceed_including_error_report():
            return False
        self._sequencers.user_sequencer = \
            MixerCalibrationExtrinsicResultMetadataUpdateSequencer(**self._sequencer_init_args())
        self._sequencers.user_sequencer.begin(
            component_labels=[mixer_label],
            callback=callback,
            request_args={
                "result_identifier": result_identifier,
                "result_state": str(result_state),
                "result_label": result_label})
        return True
