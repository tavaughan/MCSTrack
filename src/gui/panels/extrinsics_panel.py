from .base_panel import \
    BasePanel
from .feedback import \
    ImagePanel
from .parameters import \
    ParameterSelector, \
    ParameterText
from .specialized import \
    CalibrationImageTable, \
    CalibrationResultTable
from src.common import \
    ErrorResponse, \
    EmptyResponse, \
    ExtrinsicCalibrator, \
    ImageFormat, \
    ImageResolution, \
    ImageUtils, \
    IntrinsicCalibrator, \
    MCTRequestSeries, \
    MCTResponse, \
    MCTResponseSeries, \
    StatusMessageSource
from src.controller import \
    MCTController
from src.detector import \
    CameraImageGetRequest, \
    CameraImageGetResponse, \
    IntrinsicCalibrationResultGetActiveRequest, \
    IntrinsicCalibrationResultGetActiveResponse
from src.mixer import \
    ExtrinsicCalibrationCalculateRequest, \
    ExtrinsicCalibrationCalculateResponse, \
    ExtrinsicCalibrationDeleteStagedRequest, \
    ExtrinsicCalibrationImageAddRequest, \
    ExtrinsicCalibrationImageAddResponse, \
    ExtrinsicCalibrationImageGetRequest, \
    ExtrinsicCalibrationImageGetResponse, \
    ExtrinsicCalibrationImageMetadataUpdateRequest, \
    ExtrinsicCalibrationMetadataListRequest, \
    ExtrinsicCalibrationMetadataListResponse, \
    ExtrinsicCalibrationResultGetRequest, \
    ExtrinsicCalibrationResultGetResponse, \
    ExtrinsicCalibrationResultMetadataUpdateRequest, \
    MixerIntrinsicUpdateRequest
import datetime
from io import BytesIO
import logging
import numpy
from typing import Optional
import uuid
import wx
import wx.grid


logger = logging.getLogger(__name__)
_PREVIEW_CAPTURE_FORMAT: ImageFormat = ImageFormat.FORMAT_JPG


class ExtrinsicsPanel(BasePanel):

    _controller: MCTController

    _mixer_selector: ParameterSelector
    _reload_button: wx.Button
    _preview_toggle_button: wx.ToggleButton
    _capture_button: wx.Button
    _image_table: CalibrationImageTable
    _image_label_textbox: ParameterText
    _image_state_selector: ParameterSelector
    _image_update_button: wx.Button
    _calibrate_button: wx.Button
    _calibrate_status_textbox: wx.TextCtrl
    _result_table: CalibrationResultTable
    _result_display_textbox: wx.TextCtrl
    _result_label_textbox: ParameterText
    _result_state_selector: ParameterSelector
    _result_update_button: wx.Button
    _image_panel: ImagePanel

    _control_blocking_request_ids: set[uuid.UUID]
    _is_updating: bool  # Some things should only trigger during explicit user events
    _preview_request_ids_by_detector_label: dict[str, uuid.UUID]
    _preview_images_by_detector_label: dict[str, numpy.ndarray]
    _extrinsic_image: numpy.ndarray | None
    _current_capture_timestamp: datetime.datetime | None  # None indicates no capture in progress
    _calibration_in_progress: bool
    _image_metadata_list: list[IntrinsicCalibrator.ImageMetadata]
    _result_metadata_list: list[IntrinsicCalibrator.ResultMetadata]

    def __init__(
        self,
        parent: wx.Window,
        controller: MCTController,
        name: str = "IntrinsicsPanel"
    ):
        super().__init__(
            parent=parent,
            name=name)
        self._controller = controller

        self._control_blocking_request_ids = set()
        self._is_updating = False
        self._preview_request_ids_by_detector_label = dict()
        self._preview_images_by_detector_label = dict()
        self._extrinsic_image = None
        self._current_capture_timestamp = None
        self._calibration_in_progress = False
        self._image_metadata_list = list()
        self._result_metadata_list = list()

        horizontal_split_sizer: wx.BoxSizer = wx.BoxSizer(orient=wx.HORIZONTAL)

        control_border_panel: wx.Panel = wx.Panel(parent=self)
        control_border_box: wx.StaticBoxSizer = wx.StaticBoxSizer(
            orient=wx.VERTICAL,
            parent=control_border_panel)
        control_panel: wx.ScrolledWindow = wx.ScrolledWindow(
            parent=control_border_panel)
        control_panel.SetScrollRate(
            xstep=1,
            ystep=1)
        control_panel.ShowScrollbars(
            horz=wx.SHOW_SB_NEVER,
            vert=wx.SHOW_SB_ALWAYS)

        control_sizer: wx.BoxSizer = wx.BoxSizer(orient=wx.VERTICAL)

        self._mixer_selector = self.add_control_selector(
            parent=control_panel,
            sizer=control_sizer,
            label="Mixer",
            selectable_values=list())

        self._reload_button = self.add_control_button(
            parent=control_panel,
            sizer=control_sizer,
            label="Reload Metadata")

        self._preview_toggle_button = wx.ToggleButton(
            parent=control_panel,
            label="Preview")
        control_sizer.Add(
            window=self._preview_toggle_button,
            flags=wx.SizerFlags(0).Expand())
        control_sizer.AddSpacer(size=BasePanel.DEFAULT_SPACING_PX_VERTICAL)

        self._capture_button = self.add_control_button(
            parent=control_panel,
            sizer=control_sizer,
            label="Capture")

        self._calibrate_button: wx.Button = self.add_control_button(
            parent=control_panel,
            sizer=control_sizer,
            label="Calibrate")

        self._calibrate_status_textbox = wx.TextCtrl(
            parent=control_panel,
            style=wx.TE_READONLY | wx.TE_RICH)
        self._calibrate_status_textbox.SetEditable(False)
        self._calibrate_status_textbox.SetBackgroundColour(colour=wx.Colour(red=249, green=249, blue=249, alpha=255))
        control_sizer.Add(
            window=self._calibrate_status_textbox,
            flags=wx.SizerFlags(0).Expand())

        self.add_horizontal_line_to_spacer(
            parent=control_panel,
            sizer=control_sizer)

        self._image_table = CalibrationImageTable(parent=control_panel)
        self._image_table.SetMaxSize((-1, self._image_table.GetSize().GetHeight()))
        control_sizer.Add(
            window=self._image_table,
            flags=wx.SizerFlags(0).Expand())
        control_sizer.AddSpacer(size=BasePanel.DEFAULT_SPACING_PX_VERTICAL)

        self._image_label_textbox: ParameterText = self.add_control_text_input(
            parent=control_panel,
            sizer=control_sizer,
            label="Image Label")

        self._image_state_selector: ParameterSelector = self.add_control_selector(
            parent=control_panel,
            sizer=control_sizer,
            label="Image State",
            selectable_values=[state.name for state in IntrinsicCalibrator.ImageState])

        self._image_update_button: wx.Button = self.add_control_button(
            parent=control_panel,
            sizer=control_sizer,
            label="Update Image")

        self.add_horizontal_line_to_spacer(
            parent=control_panel,
            sizer=control_sizer)

        self._result_table = CalibrationResultTable(parent=control_panel)
        control_sizer.Add(
            window=self._result_table,
            flags=wx.SizerFlags(0).Expand())
        control_sizer.AddSpacer(size=BasePanel.DEFAULT_SPACING_PX_VERTICAL)

        self._result_display_textbox = wx.TextCtrl(
            parent=control_panel,
            style=wx.TE_MULTILINE | wx.TE_READONLY | wx.TE_RICH)
        self._result_display_textbox.SetEditable(False)
        self._result_display_textbox.SetBackgroundColour(colour=wx.Colour(red=249, green=249, blue=249, alpha=255))
        control_sizer.Add(
            window=self._result_display_textbox,
            flags=wx.SizerFlags(1).Align(wx.EXPAND))

        self._result_label_textbox: ParameterText = self.add_control_text_input(
            parent=control_panel,
            sizer=control_sizer,
            label="Result Label")

        self._result_state_selector: ParameterSelector = self.add_control_selector(
            parent=control_panel,
            sizer=control_sizer,
            label="Result State",
            selectable_values=[state.name for state in IntrinsicCalibrator.ResultState])

        self._result_update_button: wx.Button = self.add_control_button(
            parent=control_panel,
            sizer=control_sizer,
            label="Update Result")

        self.add_horizontal_line_to_spacer(
            parent=control_panel,
            sizer=control_sizer)

        control_spacer_sizer: wx.BoxSizer = wx.BoxSizer(orient=wx.HORIZONTAL)
        control_sizer.Add(
            sizer=control_spacer_sizer,
            flags=wx.SizerFlags(1).Expand())

        control_panel.SetSizerAndFit(sizer=control_sizer)
        control_border_box.Add(
            window=control_panel,
            flags=wx.SizerFlags(1).Expand())
        control_border_panel.SetSizer(sizer=control_border_box)
        horizontal_split_sizer.Add(
            window=control_border_panel,
            flags=wx.SizerFlags(50).Expand())

        self._image_panel = ImagePanel(parent=self)
        self._image_panel.SetBackgroundColour(colour=wx.BLACK)
        horizontal_split_sizer.Add(
            window=self._image_panel,
            flags=wx.SizerFlags(50).Expand())

        self.SetSizerAndFit(sizer=horizontal_split_sizer)

        self._mixer_selector.selector.Bind(
            event=wx.EVT_CHOICE,
            handler=self._on_mixer_reload)
        self._reload_button.Bind(
            event=wx.EVT_BUTTON,
            handler=self._on_mixer_reload)
        self._preview_toggle_button.Bind(
            event=wx.EVT_BUTTON,
            handler=self._on_preview_toggled)
        self._capture_button.Bind(
            event=wx.EVT_BUTTON,
            handler=self._on_capture_pressed)
        self._calibrate_button.Bind(
            event=wx.EVT_BUTTON,
            handler=self._on_calibrate_pressed)
        self._image_table.table.Bind(
            event=wx.grid.EVT_GRID_SELECT_CELL,
            handler=self._on_image_metadata_selected)
        self._image_update_button.Bind(
            event=wx.EVT_BUTTON,
            handler=self._on_image_update_pressed)
        self._result_table.table.Bind(
            event=wx.grid.EVT_GRID_SELECT_CELL,
            handler=self._on_result_metadata_selected)
        self._result_update_button.Bind(
            event=wx.EVT_BUTTON,
            handler=self._on_result_update_pressed)

    def handle_error_response(
        self,
        response: ErrorResponse
    ):
        super().handle_error_response(response=response)
        if self._calibration_in_progress:
            self._calibrate_status_textbox.SetForegroundColour(colour=wx.Colour(red=127, green=0, blue=0, alpha=255))
            self._calibrate_status_textbox.SetValue(f"Error: {response.message}")

    def handle_response_series(
        self,
        response_series: MCTResponseSeries,
        task_description: Optional[str] = None,
        expected_response_count: Optional[int] = None
    ) -> None:
        response: MCTResponse
        for response in response_series.series:
            if isinstance(response, CameraImageGetResponse):
                self._handle_response_camera_image_get(response=response, detector_label=response_series.responder)
            elif isinstance(response, ExtrinsicCalibrationCalculateResponse):
                self._handle_response_extrinsic_calibration_calculate(response=response)
            elif isinstance(response, ExtrinsicCalibrationImageAddResponse):
                self._handle_response_extrinsic_calibration_image_add(response=response)
            elif isinstance(response, ExtrinsicCalibrationImageGetResponse):
                self._handle_response_extrinsic_calibration_image_get(response=response)
            elif isinstance(response, ExtrinsicCalibrationResultGetResponse):
                self._handle_response_extrinsic_calibration_result_get(response=response)
            elif isinstance(response, ExtrinsicCalibrationMetadataListResponse):
                self._handle_response_extrinsic_calibration_image_metadata_list(response=response)
            elif isinstance(response, IntrinsicCalibrationResultGetActiveResponse):
                self._handle_response_intrinsic_calibration_result_get_active(
                    response=response,
                    detector_label=response_series.responder)
            elif isinstance(response, ErrorResponse):
                self.handle_error_response(response=response)
            elif not isinstance(response, EmptyResponse):
                self.handle_unknown_response(response=response)

    def on_ui_page_select(self) -> None:
        super().on_ui_page_select()
        available_mixer_labels: list[str] = self._controller.get_remote_labels_mixer()
        self._mixer_selector.set_options(option_list=available_mixer_labels)
        self._update_ui_controls()

    def update_loop(self) -> None:
        super().update_loop()
        self._is_updating = True

        response_series: MCTResponseSeries | None
        responded_request_ids: list[tuple[uuid.UUID, MCTResponseSeries]] = list()
        for request_id in self._control_blocking_request_ids:
            _, response_series = self._controller.response_series_pop(request_series_id=request_id)
            if response_series is not None:
                responded_request_ids.append((request_id, response_series))
        if len(responded_request_ids) > 0:
            # Clean up the request id list first
            for request_id, response_series in responded_request_ids:
                self._control_blocking_request_ids.remove(request_id)
            # THEN handle the responses. This order of ops is assumed in calibration handling.
            for request_id, response_series in responded_request_ids:
                self.handle_response_series(response_series)
            self._update_ui_controls()

        if self._preview_toggle_button.GetValue():
            detector_labels_with_responses: set[str] = set()
            for detector_label, request_id in self._preview_request_ids_by_detector_label.items():
                _, response_series = self._controller.response_series_pop(request_series_id=request_id)
                if response_series is not None and \
                   len(response_series.series) > 0 and \
                   isinstance(response_series.series[0], CameraImageGetResponse):
                    response: CameraImageGetResponse = response_series.series[0]
                    self._preview_images_by_detector_label[detector_label] = \
                        ImageUtils.base64_to_image(response.image_base64)
                    detector_labels_with_responses.add(detector_label)
            detector_labels: list[str] = self._controller.get_remote_labels_detectors()
            for detector_label in detector_labels:
                if detector_label in detector_labels_with_responses or \
                   detector_label not in self._preview_request_ids_by_detector_label:
                    request_series: MCTRequestSeries = MCTRequestSeries(
                        series=[CameraImageGetRequest(
                            format=_PREVIEW_CAPTURE_FORMAT,
                            requested_resolution=ImageResolution(x_px=800, y_px=480))])  # TODO: Parameterize
                    preview_request_id = self._controller.send_custom_request(
                        component_label=detector_label,
                        request_series=request_series)
                    self._preview_request_ids_by_detector_label[detector_label] = preview_request_id

        self._update_ui_image()

        self._is_updating = False

    def _handle_response_camera_image_get(
        self,
        response: CameraImageGetResponse,
        detector_label: str
    ) -> None:
        # Note: This is for the control-blocking requests ONLY!
        mixer_label: str = self._mixer_selector.selector.GetStringSelection()
        request_series: MCTRequestSeries = MCTRequestSeries(series=[
            ExtrinsicCalibrationImageAddRequest(
                image_base64=response.image_base64,
                detector_label=detector_label,
                timestamp_utc_iso8601=self._current_capture_timestamp.isoformat()),
            ExtrinsicCalibrationMetadataListRequest()])
        self._control_blocking_request_ids.add(self._controller.send_custom_request(
            component_label=mixer_label,
            request_series=request_series))

    def _handle_response_intrinsic_calibration_result_get_active(
        self,
        response: IntrinsicCalibrationResultGetActiveResponse,
        detector_label: str
    ) -> None:
        mixer_label: str = self._mixer_selector.selector.GetStringSelection()
        request_series: MCTRequestSeries = MCTRequestSeries(series=[
            MixerIntrinsicUpdateRequest(
                detector_label=detector_label,
                intrinsic_parameters=response.intrinsic_calibration.calibrated_values)])
        if len(self._control_blocking_request_ids) <= 0:  # This is the last intrinsic - we are ready to calculate
            request_series.series.append(ExtrinsicCalibrationCalculateRequest())
            # request_series.series.append(ExtrinsicCalibrationResultMetadataListRequest())
        self._control_blocking_request_ids.add(self._controller.send_custom_request(
            component_label=mixer_label,
            request_series=request_series))

    def _handle_response_extrinsic_calibration_calculate(
        self,
        response: ExtrinsicCalibrationCalculateResponse
    ) -> None:
        if not self._calibration_in_progress:
            self.status_message_source.enqueue_status_message(
                severity="warning",
                message=f"Received CalibrateResponse while no calibration is in progress.")
        self._calibrate_status_textbox.SetForegroundColour(colour=wx.Colour(red=0, green=0, blue=127, alpha=255))
        self._calibrate_status_textbox.SetValue(f"Calibration {response.result_identifier} complete.")
        self._result_display_textbox.SetValue(response.extrinsic_calibration.model_dump_json(indent=4))
        self._calibration_in_progress = False

    # noinspection PyUnusedLocal
    def _handle_response_extrinsic_calibration_image_add(
        self,
        response: ExtrinsicCalibrationImageAddResponse
    ) -> None:
        if len(self._control_blocking_request_ids) <= 0:
            self._current_capture_timestamp = None

    def _handle_response_extrinsic_calibration_image_get(
        self,
        response: ExtrinsicCalibrationImageGetResponse
    ) -> None:
        self._extrinsic_image = ImageUtils.base64_to_image(response.image_base64)

    def _handle_response_extrinsic_calibration_image_metadata_list(
        self,
        response: ExtrinsicCalibrationMetadataListResponse
    ) -> None:
        self._image_metadata_list = response.metadata_list
        self._image_table.update_contents(row_contents=self._image_metadata_list)

    def _handle_response_extrinsic_calibration_result_get(
        self,
        response: ExtrinsicCalibrationResultGetResponse
    ) -> None:
        self._result_display_textbox.SetValue(str(response.extrinsic_calibration.model_dump_json(indent=4)))

    def _handle_response_extrinsic_calibration_result_metadata_list(
        self,
        response  # : ExtrinsicCalibrationResultMetadataListResponse
    ) -> None:
        self._result_metadata_list = response.metadata_list
        self._result_table.update_contents(row_contents=self._result_metadata_list)

    def _on_mixer_reload(self, _event: wx.CommandEvent) -> None:
        self._image_metadata_list = list()
        self._result_metadata_list = list()
        self._calibrate_status_textbox.SetValue(str())
        self._result_display_textbox.SetValue(str())
        mixer_label: str = self._mixer_selector.selector.GetStringSelection()
        request_series: MCTRequestSeries = MCTRequestSeries(series=[
            ExtrinsicCalibrationMetadataListRequest()])
            # ExtrinsicCalibrationResultMetadataListRequest()
        self._control_blocking_request_ids.add(self._controller.send_custom_request(
            component_label=mixer_label,
            request_series=request_series))
        self._update_ui_controls()

    def _on_preview_toggled(self, _event: wx.CommandEvent) -> None:
        if self._is_updating:
            return
        self._image_table.set_selected_row_index(None)
        self._update_ui_controls()

    def _on_capture_pressed(self, _event: wx.CommandEvent) -> None:
        self._current_capture_timestamp = datetime.datetime.now(tz=datetime.timezone.utc)
        detector_labels: list[str] = self._controller.get_remote_labels_detectors()
        for detector_label in detector_labels:
            request_series: MCTRequestSeries = MCTRequestSeries(series=[
                CameraImageGetRequest(format=ImageFormat.FORMAT_PNG)])
            self._control_blocking_request_ids.add(self._controller.send_custom_request(
                component_label=detector_label,
                request_series=request_series))
        self._update_ui_controls()

    def _on_calibrate_pressed(self, _event: wx.CommandEvent) -> None:
        self._calibrate_status_textbox.SetForegroundColour(colour=wx.Colour(red=0, green=0, blue=0, alpha=255))
        self._calibrate_status_textbox.SetValue("Calibrating...")
        self._result_display_textbox.SetValue(str())
        detector_labels: list[str] = self._controller.get_remote_labels_detectors()
        for detector_label in detector_labels:
            request_series: MCTRequestSeries = MCTRequestSeries(series=[IntrinsicCalibrationResultGetActiveRequest()])
            self._control_blocking_request_ids.add(self._controller.send_custom_request(
                component_label=detector_label,
                request_series=request_series))
        self._calibration_in_progress = True
        self._update_ui_controls()

    def _on_image_metadata_selected(self, _event: wx.grid.GridEvent) -> None:
        if self._is_updating:
            return
        self._preview_toggle_button.SetValue(False)
        image_index: int = self._image_table.get_selected_row_index()
        image_identifier: str | None = self._image_metadata_list[image_index].identifier
        if image_identifier is not None:
            request_series: MCTRequestSeries = MCTRequestSeries(series=[
                ExtrinsicCalibrationImageGetRequest(image_identifier=image_identifier)])
            mixer_label: str = self._mixer_selector.selector.GetStringSelection()
            self._control_blocking_request_ids.add(self._controller.send_custom_request(
                component_label=mixer_label,
                request_series=request_series))
        self._update_ui_controls()

    def _on_image_update_pressed(self, _event: wx.CommandEvent) -> None:
        self._calibrate_status_textbox.SetValue(str())
        mixer_label: str = self._mixer_selector.selector.GetStringSelection()
        image_index: int = self._image_table.get_selected_row_index()
        image_identifier: str = self._image_metadata_list[image_index].identifier
        image_state: IntrinsicCalibrator.ImageState = \
            ExtrinsicCalibrator.ImageState[self._image_state_selector.selector.GetStringSelection()]
        image_label: str = self._image_label_textbox.textbox.GetValue()
        request_series: MCTRequestSeries = MCTRequestSeries(series=[
            ExtrinsicCalibrationImageMetadataUpdateRequest(
                image_identifier=image_identifier,
                image_state=image_state,
                image_label=image_label),
            ExtrinsicCalibrationDeleteStagedRequest(),
            ExtrinsicCalibrationMetadataListRequest()])
        self._control_blocking_request_ids.add(self._controller.send_custom_request(
            component_label=mixer_label,
            request_series=request_series))
        self._update_ui_controls()

    def _on_result_metadata_selected(self, _event: wx.grid.GridEvent) -> None:
        if self._is_updating:
            return
        self._preview_toggle_button.SetValue(False)
        self._result_display_textbox.SetValue(str())
        result_index: int = self._result_table.get_selected_row_index()
        result_identifier: str | None = self._result_metadata_list[result_index].identifier
        if result_identifier is not None:
            request_series: MCTRequestSeries = MCTRequestSeries(series=[
                ExtrinsicCalibrationResultGetRequest(result_identifier=result_identifier)])
            mixer_label: str = self._mixer_selector.selector.GetStringSelection()
            self._control_blocking_request_ids.add(self._controller.send_custom_request(
                component_label=mixer_label,
                request_series=request_series))
        self._update_ui_controls()

    def _on_result_update_pressed(self, _event: wx.CommandEvent) -> None:
        self._result_display_textbox.SetValue(str())
        mixer_label: str = self._mixer_selector.selector.GetStringSelection()
        result_index: int = self._result_table.get_selected_row_index()
        result_identifier: str = self._result_metadata_list[result_index].identifier
        result_state: ExtrinsicCalibrator.ResultState = \
            ExtrinsicCalibrator.ResultState[self._result_state_selector.selector.GetStringSelection()]
        result_label: str = self._result_label_textbox.textbox.GetValue()
        request_series: MCTRequestSeries = MCTRequestSeries(series=[
            ExtrinsicCalibrationResultMetadataUpdateRequest(
                result_identifier=result_identifier,
                result_state=result_state,
                result_label=result_label),
            ExtrinsicCalibrationDeleteStagedRequest()])
            # ExtrinsicCalibrationResultMetadataListRequest()
        self._control_blocking_request_ids.add(self._controller.send_custom_request(
            component_label=mixer_label,
            request_series=request_series))
        self._update_ui_controls()

    def _update_ui_controls(self) -> None:
        self._mixer_selector.Enable(False)
        self._reload_button.Enable(False)
        self._preview_toggle_button.Enable(False)
        self._capture_button.Enable(False)
        self._calibrate_button.Enable(False)
        self._image_table.Enable(False)
        self._image_label_textbox.Enable(False)
        self._image_label_textbox.textbox.SetValue(str())
        self._image_state_selector.Enable(False)
        self._image_state_selector.selector.SetStringSelection(str())
        self._image_update_button.Enable(False)
        self._calibrate_status_textbox.Enable(False)
        self._result_table.Enable(False)
        self._result_display_textbox.Enable(False)
        self._result_label_textbox.Enable(False)
        self._result_label_textbox.textbox.SetValue(str())
        self._result_state_selector.Enable(False)
        self._result_state_selector.selector.SetStringSelection(str())
        self._result_update_button.Enable(False)
        if len(self._control_blocking_request_ids) > 0:
            return  # We're waiting for something
        self._mixer_selector.Enable(True)
        mixer_label: str = self._mixer_selector.selector.GetStringSelection()
        if len(mixer_label) <= 0:
            self._preview_toggle_button.SetValue(False)
            return
        self._reload_button.Enable(True)
        self._preview_toggle_button.Enable(True)
        self._capture_button.Enable(True)
        # == NO RETURN GUARDS AFTER THIS POINT ==
        if len(self._image_metadata_list) > 0:
            self._image_table.Enable(True)
            image_index: int | None = self._image_table.get_selected_row_index()
            if image_index is not None:
                if image_index >= len(self._image_metadata_list):
                    self.status_message_source.enqueue_status_message(
                        severity="warning",
                        message=f"Selected image index {image_index} is out of bounds. Setting to None.")
                    self._image_table.set_selected_row_index(None)
                else:
                    image_metadata: IntrinsicCalibrator.ImageMetadata = self._image_metadata_list[image_index]
                    self._image_label_textbox.Enable(True)
                    self._image_label_textbox.textbox.SetValue(image_metadata.label)
                    self._image_state_selector.Enable(True)
                    self._image_state_selector.selector.SetStringSelection(image_metadata.state.name)
                    self._image_update_button.Enable(True)
            calibration_image_count: int = 0
            for image_metadata in self._image_metadata_list:
                if image_metadata.state == IntrinsicCalibrator.ImageState.SELECT:
                    calibration_image_count += 1
            if calibration_image_count > 0:
                self._calibrate_button.Enable(True)
                self._calibrate_status_textbox.Enable(True)
        if len(self._result_metadata_list) > 0:
            self._result_table.Enable(True)
            result_index: int | None = self._result_table.get_selected_row_index()
            if result_index is not None:
                if result_index >= len(self._result_metadata_list):
                    self.status_message_source.enqueue_status_message(
                        severity="warning",
                        message=f"Selected result index {result_index} is out of bounds. Setting to None.")
                    self._result_table.set_selected_row_index(None)
                else:
                    result_metadata: IntrinsicCalibrator.ResultMetadata = self._result_metadata_list[result_index]
                    self._result_display_textbox.Enable(True)
                    self._result_label_textbox.Enable(True)
                    self._result_label_textbox.textbox.SetValue(result_metadata.label)
                    self._result_state_selector.Enable(True)
                    self._result_state_selector.selector.SetStringSelection(result_metadata.state.name)
                    self._result_update_button.Enable(True)
        self.Layout()
        self.Refresh()
        self.Update()

    def _update_ui_image(self):
        display_image: numpy.ndarray = ImageUtils.black_image(resolution_px=self._image_panel.GetSize())
        available_size_px: int = (display_image.shape[1], display_image.shape[0])
        if self._preview_toggle_button.GetValue():
            detector_labels: list[str] = self._controller.get_remote_labels_detectors()
            image_dimensions: tuple[int, int]
            image_positions: list[tuple[int, int]]
            image_dimensions, image_positions = ImageUtils.partition_rect(
                available_size_px=available_size_px,
                partition_count=len(detector_labels))
            for detector_index, detector_label in enumerate(detector_labels):
                if detector_label in self._preview_images_by_detector_label:
                    detector_image: numpy.ndarray = self._preview_images_by_detector_label[detector_label]
                    detector_image = ImageUtils.image_resize_to_fit(
                        opencv_image=detector_image,
                        available_size=image_dimensions)
                    offset_y_px: int = image_positions[detector_index][1] + (image_dimensions[1] - detector_image.shape[0]) // 2
                    offset_x_px: int = image_positions[detector_index][0] + (image_dimensions[0] - detector_image.shape[1]) // 2
                    display_image[
                        offset_y_px:offset_y_px + detector_image.shape[0],
                        offset_x_px:offset_x_px + detector_image.shape[1]
                    ] = detector_image
        elif self._extrinsic_image is not None:
            extrinsic_image: numpy.ndarray = ImageUtils.image_resize_to_fit(
                opencv_image=self._extrinsic_image,
                available_size=available_size_px)
            offset_y_px: int = (display_image.shape[0] - extrinsic_image.shape[0]) // 2
            offset_x_px: int = (display_image.shape[1] - extrinsic_image.shape[1]) // 2
            display_image[
                offset_y_px:offset_y_px + extrinsic_image.shape[0],
                offset_x_px:offset_x_px + extrinsic_image.shape[1],
            ] = extrinsic_image

        image_buffer: bytes = ImageUtils.image_to_bytes(image_data=display_image, image_format=".jpg")
        image_buffer_io: BytesIO = BytesIO(image_buffer)
        wx_image: wx.Image = wx.Image(image_buffer_io)
        wx_bitmap: wx.Bitmap = wx_image.ConvertToBitmap()
        self._image_panel.set_bitmap(wx_bitmap)
        self._image_panel.paint()
