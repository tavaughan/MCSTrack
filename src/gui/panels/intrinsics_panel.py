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
    ImageResolution, \
    ImageUtils, \
    IntrinsicCalibrator, \
    MCTRequestSeries, \
    MCTResponse, \
    MCTResponseSeries
from src.controller import \
    MCTController
from src.detector import \
    IntrinsicCalibrationImageAddResponse, \
    IntrinsicCalibrationCalculateRequest, \
    IntrinsicCalibrationCalculateResponse, \
    IntrinsicCalibrationDeleteStagedRequest, \
    IntrinsicCalibrationImageGetRequest, \
    IntrinsicCalibrationImageGetResponse, \
    IntrinsicCalibrationImageMetadataListRequest, \
    IntrinsicCalibrationImageMetadataListResponse, \
    IntrinsicCalibrationImageMetadataUpdateRequest, \
    IntrinsicCalibrationResolutionListRequest, \
    IntrinsicCalibrationResolutionListResponse, \
    IntrinsicCalibrationResultGetRequest, \
    IntrinsicCalibrationResultGetResponse, \
    IntrinsicCalibrationResultMetadataListRequest, \
    IntrinsicCalibrationResultMetadataListResponse, \
    IntrinsicCalibrationResultMetadataUpdateRequest
from io import BytesIO
import logging
from typing import Optional
import wx
import wx.grid


logger = logging.getLogger(__name__)


class IntrinsicsPanel(BasePanel):

    _controller: MCTController

    _detector_selector: ParameterSelector
    _detector_resolution_selector: ParameterSelector
    _live_preview_button: wx.Button
    _capture_button: wx.Button
    _calibrate_button: wx.Button
    _calibrate_status_textbox: wx.TextCtrl
    _load_metadata_button: wx.Button
    _image_table: CalibrationImageTable
    _image_label_textbox: ParameterText
    _image_state_selector: ParameterSelector
    _image_update_button: wx.Button
    _result_table: CalibrationResultTable
    _result_display_textbox: wx.TextCtrl
    _result_label_textbox: ParameterText
    _result_state_selector: ParameterSelector
    _result_update_button: wx.Button
    _image_panel: ImagePanel

    _awaiting_user_task: bool
    _force_last_result_selected: bool
    _detector_resolutions: list[ImageResolution]
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

        self._awaiting_user_task = False
        self._force_last_result_selected = False
        self._detector_resolutions = list()
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

        self._detector_selector = self.add_control_selector(
            parent=control_panel,
            sizer=control_sizer,
            label="Detector",
            selectable_values=list())

        self._detector_resolution_selector: ParameterSelector = self.add_control_selector(
            parent=control_panel,
            sizer=control_sizer,
            label="Resolution",
            selectable_values=list())

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

        self._load_metadata_button: wx.Button = self.add_control_button(
            parent=control_panel,
            sizer=control_sizer,
            label="Load Metadata")

        self._image_table = CalibrationImageTable(parent=control_panel)
        self._image_table.SetMaxSize(size=wx.Size(-1, self._image_table.GetSize().GetHeight()))
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

        self._detector_selector.selector.Bind(
            event=wx.EVT_CHOICE,
            handler=self._on_detector_selected)
        self._detector_resolution_selector.selector.Bind(
            event=wx.EVT_CHOICE,
            handler=self._on_detector_resolution_selected)
        self._load_metadata_button.Bind(
            event=wx.EVT_BUTTON,
            handler=self._on_detector_load_pressed)
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
        self._calibrate_button.Bind(
            event=wx.EVT_BUTTON,
            handler=self._on_calibrate_pressed)

    def begin_capture_calibration(self) -> None:
        selected_detector_label: str = self._detector_selector.selector.GetStringSelection()
        self._controller.calibrate_intrinsic_image_add(
            detector_label=selected_detector_label)  # TODO: This needs a callback, and to be called somewhere?

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
            if isinstance(response, IntrinsicCalibrationImageAddResponse):
                self._handle_response_add_calibration_image_response(response=response)
            elif isinstance(response, IntrinsicCalibrationCalculateResponse):
                self._handle_response_calibrate(response=response)
            elif isinstance(response, IntrinsicCalibrationImageGetResponse):
                self._handle_response_get_calibration_image(response=response)
            elif isinstance(response, IntrinsicCalibrationResultGetResponse):
                self._handle_response_get_calibration_result(response=response)
            elif isinstance(response, IntrinsicCalibrationResolutionListResponse):
                self._handle_response_list_calibration_detector_resolutions(response=response)
            elif isinstance(response, IntrinsicCalibrationImageMetadataListResponse):
                self._handle_response_list_calibration_image_metadata(response=response)
            elif isinstance(response, IntrinsicCalibrationResultMetadataListResponse):
                self._handle_response_list_calibration_result_metadata(response=response)
            elif isinstance(response, ErrorResponse):
                self.handle_error_response(response=response)
            elif not isinstance(response, EmptyResponse):
                self.handle_unknown_response(response=response)

    def on_ui_page_select(self) -> None:
        super().on_ui_page_select()
        selected_detector_label: str = self._detector_selector.selector.GetStringSelection()
        available_detector_labels: list[str] = self._controller.get_remote_labels_detectors()
        self._detector_selector.set_options(option_list=available_detector_labels)
        if selected_detector_label in available_detector_labels:
            self._detector_selector.selector.SetStringSelection(selected_detector_label)
        else:
            self._detector_selector.selector.SetStringSelection(str())
        self._update_ui_controls()

    def update_loop(self) -> None:
        super().update_loop()
        self._is_updating = True
        response_series: MCTResponseSeries | None
        if self._control_blocking_request_id is not None:
            self._control_blocking_request_id, response_series = self._controller.response_series_pop(
                request_series_id=self._control_blocking_request_id)
            if response_series is not None:  # self._control_blocking_request_id will be None
                self.handle_response_series(response_series)
                self._update_ui_controls()
        self._is_updating = False

    def _handle_response_add_calibration_image_response(
        self,
        response: IntrinsicCalibrationImageAddResponse
    ):
        self.status_message_source.enqueue_status_message(
            severity="info",
            message=f"Added image {response.image_identifier}.")

    def _handle_response_calibrate(
        self,
        response: IntrinsicCalibrationCalculateResponse
    ) -> None:
        if not self._calibration_in_progress:
            self.status_message_source.enqueue_status_message(
                severity="warning",
                message=f"Received CalibrateResponse while no calibration is in progress.")
        self._calibrate_status_textbox.SetForegroundColour(colour=wx.Colour(red=0, green=0, blue=127, alpha=255))
        self._calibrate_status_textbox.SetValue(
            f"Calibration {response.result_identifier} complete - values: "
            f"{str(response.intrinsic_calibration.calibrated_values.as_array())}")
        self._result_display_textbox.SetValue(response.intrinsic_calibration.model_dump_json(indent=4))
        self._calibration_in_progress = False
        self._force_last_result_selected = True

    def _handle_response_get_calibration_image(
        self,
        response: IntrinsicCalibrationImageGetResponse
    ) -> None:
        opencv_image = ImageUtils.base64_to_image(input_base64=response.image_base64)
        opencv_image = ImageUtils.image_resize_to_fit(
            opencv_image=opencv_image,
            available_size=self._image_panel.GetSize())
        image_buffer: bytes = ImageUtils.image_to_bytes(image_data=opencv_image, image_format=".jpg")
        image_buffer_io: BytesIO = BytesIO(image_buffer)
        wx_image: wx.Image = wx.Image(image_buffer_io)
        wx_bitmap: wx.Bitmap = wx_image.ConvertToBitmap()
        self._image_panel.set_bitmap(wx_bitmap)
        self._image_panel.paint()

    def _handle_response_get_calibration_result(
        self,
        response: IntrinsicCalibrationResultGetResponse
    ) -> None:
        self._result_display_textbox.SetValue(str(response.intrinsic_calibration.model_dump_json(indent=4)))

    def _handle_response_list_calibration_detector_resolutions(
        self,
        response: IntrinsicCalibrationResolutionListResponse
    ) -> None:
        self._detector_resolutions = sorted(response.resolutions)
        self._detector_resolution_selector.set_options([str(res) for res in self._detector_resolutions])
        self._update_ui_controls()

    def _handle_response_list_calibration_image_metadata(
        self,
        response: IntrinsicCalibrationImageMetadataListResponse
    ) -> None:
        self._image_metadata_list = response.metadata_list
        self._image_table.update_contents(row_contents=self._image_metadata_list)

    def _handle_response_list_calibration_result_metadata(
        self,
        response: IntrinsicCalibrationResultMetadataListResponse
    ) -> None:
        self._result_metadata_list = response.metadata_list
        self._result_table.update_contents(row_contents=self._result_metadata_list)
        if self._force_last_result_selected:
            self._result_table.set_selected_row_index(len(self._result_metadata_list) - 1)
            self._force_last_result_selected = False

    def _on_calibrate_pressed(self, _event: wx.CommandEvent) -> None:
        self._calibrate_status_textbox.SetForegroundColour(colour=wx.Colour(red=0, green=0, blue=0, alpha=255))
        self._calibrate_status_textbox.SetValue("Calibrating...")
        self._result_display_textbox.SetValue(str())
        selected_detector_label: str = self._detector_selector.selector.GetStringSelection()
        selected_image_resolution: ImageResolution = \
            ImageResolution.from_str(self._detector_resolution_selector.selector.GetStringSelection())
        request_series: MCTRequestSeries = MCTRequestSeries(series=[
            IntrinsicCalibrationCalculateRequest(
                image_resolution=selected_image_resolution),
            IntrinsicCalibrationResultMetadataListRequest(
                image_resolution=selected_image_resolution)])
        self._control_blocking_request_id = self._controller.send_custom_request(
            component_label=selected_detector_label,
            request_series=request_series)
        self._calibration_in_progress = True
        self._update_ui_controls()

    def _on_detector_selected(self, _event: wx.CommandEvent) -> None:
        self._detector_resolutions = list()
        self._image_metadata_list = list()
        self._result_metadata_list = list()
        self._calibrate_status_textbox.SetValue(str())
        self._result_display_textbox.SetValue(str())
        detector_label: str = self._detector_selector.selector.GetStringSelection()
        request_series: MCTRequestSeries = MCTRequestSeries(series=[IntrinsicCalibrationResolutionListRequest()])
        self._control_blocking_request_id = self._controller.send_custom_request(
            component_label=detector_label,
            request_series=request_series)
        self._update_ui_controls()

    def _on_detector_load_pressed(self, _event: wx.CommandEvent) -> None:
        self._image_metadata_list = list()
        self._result_metadata_list = list()
        self._calibrate_status_textbox.SetValue(str())
        self._result_display_textbox.SetValue(str())
        selected_detector_label: str = self._detector_selector.selector.GetStringSelection()
        selected_image_resolution: ImageResolution = \
            ImageResolution.from_str(self._detector_resolution_selector.selector.GetStringSelection())
        request_series: MCTRequestSeries = MCTRequestSeries(series=[
            IntrinsicCalibrationImageMetadataListRequest(
                image_resolution=selected_image_resolution),
            IntrinsicCalibrationResultMetadataListRequest(
                image_resolution=selected_image_resolution)])
        self._control_blocking_request_id = self._controller.send_custom_request(
            component_label=selected_detector_label,
            request_series=request_series)
        self._update_ui_controls()

    def _on_detector_resolution_selected(self, _event: wx.CommandEvent) -> None:
        self._image_metadata_list = list()
        self._result_metadata_list = list()
        self._calibrate_status_textbox.SetValue(str())
        self._result_display_textbox.SetValue(str())
        found: bool = False
        selected_detector_resolution: str = self._detector_resolution_selector.selector.GetStringSelection()
        for image_resolution in self._detector_resolutions:
            if str(image_resolution) == selected_detector_resolution:
                found = True
                break
        if not found:
            self._detector_resolution_selector.selector.SetStringSelection(str())
        self._update_ui_controls()

    def _on_image_metadata_selected(self, _event: wx.grid.GridEvent) -> None:
        if self._is_updating:
            return
        image_index: int = self._image_table.get_selected_row_index()
        image_identifier: str | None = self._image_metadata_list[image_index].identifier
        if image_identifier is not None:
            request_series: MCTRequestSeries = MCTRequestSeries(series=[
                IntrinsicCalibrationImageGetRequest(image_identifier=image_identifier)])
            detector_label: str = self._detector_selector.selector.GetStringSelection()
            self._control_blocking_request_id = self._controller.send_custom_request(
                component_label=detector_label,
                request_series=request_series)
        self._update_ui_controls()

    def _on_image_update_pressed(self, _event: wx.CommandEvent) -> None:
        self._calibrate_status_textbox.SetValue(str())
        detector_label: str = self._detector_selector.selector.GetStringSelection()
        image_resolution: ImageResolution = \
            ImageResolution.from_str(self._detector_resolution_selector.selector.GetStringSelection())
        image_index: int = self._image_table.get_selected_row_index()
        image_identifier: str = self._image_metadata_list[image_index].identifier
        image_state: IntrinsicCalibrator.ImageState = \
            IntrinsicCalibrator.ImageState[self._image_state_selector.selector.GetStringSelection()]
        image_label: str = self._image_label_textbox.textbox.GetValue()
        request_series: MCTRequestSeries = MCTRequestSeries(series=[
            IntrinsicCalibrationImageMetadataUpdateRequest(
                image_identifier=image_identifier,
                image_state=image_state,
                image_label=image_label),
            IntrinsicCalibrationDeleteStagedRequest(),
            IntrinsicCalibrationImageMetadataListRequest(
                image_resolution=image_resolution)])
        self._control_blocking_request_id = self._controller.send_custom_request(
            component_label=detector_label,
            request_series=request_series)
        self._update_ui_controls()

    def _on_result_metadata_selected(self, _event: wx.grid.GridEvent) -> None:
        if self._is_updating:
            return
        self._result_display_textbox.SetValue(str())
        result_index: int = self._result_table.get_selected_row_index()
        result_identifier: str | None = self._result_metadata_list[result_index].identifier
        if result_identifier is not None:
            request_series: MCTRequestSeries = MCTRequestSeries(series=[
                IntrinsicCalibrationResultGetRequest(result_identifier=result_identifier)])
            detector_label: str = self._detector_selector.selector.GetStringSelection()
            self._control_blocking_request_id = self._controller.send_custom_request(
                component_label=detector_label,
                request_series=request_series)
        self._update_ui_controls()

    def _on_result_update_pressed(self, _event: wx.CommandEvent) -> None:
        self._result_display_textbox.SetValue(str())
        detector_label: str = self._detector_selector.selector.GetStringSelection()
        image_resolution: ImageResolution = \
            ImageResolution.from_str(self._detector_resolution_selector.selector.GetStringSelection())
        result_index: int = self._result_table.get_selected_row_index()
        result_identifier: str = self._result_metadata_list[result_index].identifier
        result_state: IntrinsicCalibrator.ResultState = \
            IntrinsicCalibrator.ResultState[self._result_state_selector.selector.GetStringSelection()]
        result_label: str = self._result_label_textbox.textbox.GetValue()
        request_series: MCTRequestSeries = MCTRequestSeries(series=[
            IntrinsicCalibrationResultMetadataUpdateRequest(
                result_identifier=result_identifier,
                result_state=result_state,
                result_label=result_label),
            IntrinsicCalibrationDeleteStagedRequest(),
            IntrinsicCalibrationResultMetadataListRequest(
                image_resolution=image_resolution)])
        self._control_blocking_request_id = self._controller.send_custom_request(
            component_label=detector_label,
            request_series=request_series)
        self._update_ui_controls()

    def _update_ui_controls(self) -> None:
        self._detector_selector.Enable(False)
        self._detector_resolution_selector.Enable(False)
        self._load_metadata_button.Enable(False)
        self._image_table.Enable(False)
        self._image_label_textbox.Enable(False)
        self._image_label_textbox.textbox.SetValue(str())
        self._image_state_selector.Enable(False)
        self._image_state_selector.selector.SetStringSelection(str())
        self._image_update_button.Enable(False)
        self._calibrate_button.Enable(False)
        self._calibrate_status_textbox.Enable(False)
        self._result_table.Enable(False)
        self._result_display_textbox.Enable(False)
        self._result_label_textbox.Enable(False)
        self._result_label_textbox.textbox.SetValue(str())
        self._result_state_selector.Enable(False)
        self._result_state_selector.selector.SetStringSelection(str())
        self._result_update_button.Enable(False)
        if self._control_blocking_request_id is not None:
            return  # We're waiting for something
        self._detector_selector.Enable(True)
        if len(self._detector_resolutions) <= 0:
            return
        self._detector_resolution_selector.Enable(True)
        resolution: str = self._detector_resolution_selector.selector.GetStringSelection()
        if len(resolution) <= 0:
            return
        self._load_metadata_button.Enable(True)
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
            result_index: int | None
            if self._force_last_result_selected:
                result_index = len(self._result_metadata_list) - 1
                self._force_last_result_selected = False
            else:
                result_index = self._result_table.get_selected_row_index()
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
