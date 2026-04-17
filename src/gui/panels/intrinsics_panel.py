from .base_panel import \
    BasePanel
from .parameters import \
    ParameterCheckbox, \
    ParameterSelector, \
    ParameterText
from .specialized import \
    CalibrationImageTable, \
    CalibrationResultTable, \
    DetectorSingleFramePanel
from src.common import \
    IntrinsicCalibration, \
    IntrinsicCalibrator, \
    ImageResolution, \
    SeverityLabel, \
    StatusMessageSource
from src.controller import \
    MCTController
import logging
import wx
import wx.grid


logger = logging.getLogger(__name__)


class IntrinsicsPanel(BasePanel):

    _controller: MCTController
    _status_message_source: StatusMessageSource

    _detector_selector: ParameterSelector
    _detector_resolution_selector: ParameterSelector
    _preview_image_checkbox: ParameterCheckbox
    _capture_button: wx.Button
    _calibrate_button: wx.Button
    _calibrate_status_textbox: wx.TextCtrl
    _reload_metadata_button: wx.Button
    _image_table: CalibrationImageTable
    _image_label_textbox: ParameterText
    _image_state_selector: ParameterSelector
    _image_update_button: wx.Button
    _result_table: CalibrationResultTable
    _result_display_textbox: wx.TextCtrl
    _result_label_textbox: ParameterText
    _result_state_selector: ParameterSelector
    _result_update_button: wx.Button
    _delete_staged_button: wx.Button
    _preview_panel: DetectorSingleFramePanel

    _awaiting_user_task: bool
    _metadata_needs_update: bool
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
        self._status_message_source = controller.get_status_message_source()

        self._awaiting_user_task = False
        self._metadata_needs_update = False
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

        self._preview_image_checkbox = self.add_control_checkbox(
            parent=control_panel,
            sizer=control_sizer,
            label="Preview Image")

        self._capture_button: wx.Button = self.add_control_button(
            parent=control_panel,
            sizer=control_sizer,
            label="Capture Calibration Image")

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

        self._reload_metadata_button: wx.Button = self.add_control_button(
            parent=control_panel,
            sizer=control_sizer,
            label="Reload Metadata")

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

        self._delete_staged_button: wx.Button = self.add_control_button(
            parent=control_panel,
            sizer=control_sizer,
            label="Delete Staged")

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

        self._preview_panel = DetectorSingleFramePanel(parent=self)
        self._preview_panel.SetBackgroundColour(colour=wx.BLACK)
        horizontal_split_sizer.Add(
            window=self._preview_panel,
            flags=wx.SizerFlags(50).Expand())

        self.SetSizerAndFit(sizer=horizontal_split_sizer)

        self._detector_selector.selector.Bind(
            event=wx.EVT_CHOICE,
            handler=self._on_ui_detector_selected)
        self._detector_resolution_selector.selector.Bind(
            event=wx.EVT_CHOICE,
            handler=self._on_ui_detector_resolution_selected)
        self._preview_image_checkbox.checkbox.Bind(
            event=wx.EVT_CHECKBOX,
            handler=self._on_ui_preview_toggled)
        self._capture_button.Bind(
            event=wx.EVT_BUTTON,
            handler=self._on_ui_capture_pressed)
        self._calibrate_button.Bind(
            event=wx.EVT_BUTTON,
            handler=self._on_ui_calibrate_pressed)
        self._reload_metadata_button.Bind(
            event=wx.EVT_BUTTON,
            handler=self._on_ui_metadata_reload_pressed)
        self._image_table.table.Bind(
            event=wx.grid.EVT_GRID_SELECT_CELL,
            handler=self._on_ui_image_metadata_selected)
        self._image_update_button.Bind(
            event=wx.EVT_BUTTON,
            handler=self._on_ui_image_update_pressed)
        self._result_table.table.Bind(
            event=wx.grid.EVT_GRID_SELECT_CELL,
            handler=self._on_ui_result_metadata_selected)
        self._result_update_button.Bind(
            event=wx.EVT_BUTTON,
            handler=self._on_ui_result_update_pressed)
        self._delete_staged_button.Bind(
            event=wx.EVT_BUTTON,
            handler=self._on_ui_delete_staged_pressed)

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

    def _on_ui_calibrate_pressed(self, _event: wx.CommandEvent) -> None:
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message=f"intrinsics_panel._on_ui_calibrate_pressed called.")
        self._calibrate_status_textbox.SetForegroundColour(colour=wx.Colour(red=0, green=0, blue=0, alpha=255))
        self._calibrate_status_textbox.SetValue("Calibrating...")
        self._result_display_textbox.SetValue(str())
        selected_detector_label: str = self._detector_selector.selector.GetStringSelection()
        selected_detector_resolution: ImageResolution = \
            ImageResolution.from_str(self._detector_resolution_selector.selector.GetStringSelection())
        self._controller.calibrate_intrinsic_calculate(
            detector_label=selected_detector_label,
            image_resolution=selected_detector_resolution,
            callback=self._on_response_calibrate)
        self._awaiting_user_task = True
        self._update_ui_controls()

    def _on_ui_capture_pressed(self, _event: wx.CommandEvent) -> None:
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message=f"intrinsics_panel._on_ui_capture_pressed called.")
        selected_detector_label: str = self._detector_selector.selector.GetStringSelection()
        self._controller.calibrate_intrinsic_image_add(
            detector_label=selected_detector_label,
            callback=self._on_response_image_add)
        self._awaiting_user_task = True
        self._update_ui_controls()

    def _on_ui_delete_staged_pressed(self, _event: wx.CommandEvent) -> None:
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message=f"intrinsics_panel._on_ui_delete_staged_pressed called.")
        selected_detector_label: str = self._detector_selector.selector.GetStringSelection()
        self._controller.calibrate_intrinsic_delete_staged(
            detector_label=selected_detector_label,
            callback=self._on_response_delete_staged)
        self._awaiting_user_task = True
        self._update_ui_controls()

    def _on_ui_detector_resolution_selected(self, _event: wx.CommandEvent) -> None:
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message=f"intrinsics_panel._on_ui_detector_resolution_selected called.")
        found: bool = False
        selected_detector_resolution: str = self._detector_resolution_selector.selector.GetStringSelection()
        for image_resolution in self._detector_resolutions:
            if str(image_resolution) == selected_detector_resolution:
                found = True
                break
        if not found:
            self._detector_resolution_selector.selector.SetStringSelection(str())
        self._reload_metadata()

    def _on_ui_detector_selected(self, _event: wx.CommandEvent) -> None:
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message=f"intrinsics_panel._on_ui_detector_selected called.")
        self._detector_resolutions = list()
        self._image_metadata_list = list()
        self._result_metadata_list = list()
        self._calibrate_status_textbox.SetValue(str())
        self._result_display_textbox.SetValue(str())
        selected_detector_label: str = self._detector_selector.selector.GetStringSelection()
        self._controller.calibrate_intrinsic_resolution_list(
            detector_label=selected_detector_label,
            callback=self._on_response_resolutions_list)
        self._awaiting_user_task = True
        self._update_ui_controls()

    def _on_ui_image_metadata_selected(self, _event: wx.grid.GridEvent) -> None:
        if self._awaiting_user_task:
            return  # Not initiated by user
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message=f"intrinsics_panel._on_ui_image_metadata_selected called.")
        image_index: int = self._image_table.get_selected_row_index()
        image_identifier: str | None = self._image_metadata_list[image_index].identifier
        if image_identifier is not None:
            selected_detector_label: str = self._detector_selector.selector.GetStringSelection()
            self._controller.calibrate_intrinsic_image_get(
                detector_label=selected_detector_label,
                image_identifier=image_identifier,
                callback=self._on_response_image_get)
            self._awaiting_user_task = True
        self._update_ui_controls()

    def _on_ui_image_update_pressed(self, _event: wx.CommandEvent) -> None:
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message=f"intrinsics_panel._on_ui_image_update_pressed called.")
        self._calibrate_status_textbox.SetValue(str())
        selected_detector_label: str = self._detector_selector.selector.GetStringSelection()
        image_index: int = self._image_table.get_selected_row_index()
        image_identifier: str = self._image_metadata_list[image_index].identifier
        # noinspection PyTypeChecker
        image_state: IntrinsicCalibrator.ImageState = \
            IntrinsicCalibrator.ImageState[self._image_state_selector.selector.GetStringSelection()]
        image_label: str = self._image_label_textbox.textbox.GetValue()
        self._controller.calibrate_intrinsic_image_metadata_update(
            detector_label=selected_detector_label,
            image_identifier=image_identifier,
            image_state=image_state,
            image_label=image_label,
            callback=self._on_response_image_update)
        self._awaiting_user_task = True
        self._update_ui_controls()

    def _on_ui_metadata_reload_pressed(self, _event: wx.CommandEvent) -> None:
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message=f"intrinsics_panel._on_ui_metadata_reload_pressed called.")
        self._reload_metadata()

    def _on_ui_preview_toggled(self, _event: wx.CommandEvent):
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message=f"intrinsics_panel._on_ui_preview_toggled called.")
        preview_on: bool = self._preview_image_checkbox.checkbox.GetValue()
        if preview_on:
            self._result_table.set_selected_row_index(None)
            self._controller.set_detector_includes_images(True)
            self._preview_panel.set_draw_image(True)
        else:
            self._controller.set_detector_includes_images(False)
            self._preview_panel.set_draw_image(False)

    def _on_ui_result_metadata_selected(self, _event: wx.grid.GridEvent) -> None:
        if self._awaiting_user_task:
            return  # Not initiated by user
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message=f"intrinsics_panel._on_ui_result_metadata_selected called.")
        self._result_display_textbox.SetValue(str())
        result_index: int = self._result_table.get_selected_row_index()
        result_identifier: str | None = self._result_metadata_list[result_index].identifier
        if result_identifier is not None:
            selected_detector_label: str = self._detector_selector.selector.GetStringSelection()
            self._controller.calibrate_intrinsic_result_get(
                detector_label=selected_detector_label,
                result_identifier=result_identifier,
                callback=self._on_response_result_get)
            self._awaiting_user_task = True
        self._update_ui_controls()

    def _on_ui_result_update_pressed(self, _event: wx.CommandEvent) -> None:
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message=f"intrinsics_panel._on_ui_result_update_pressed called.")
        self._result_display_textbox.SetValue(str())
        selected_detector_label: str = self._detector_selector.selector.GetStringSelection()
        result_index: int = self._result_table.get_selected_row_index()
        result_identifier: str = self._result_metadata_list[result_index].identifier
        # noinspection PyTypeChecker
        result_state: IntrinsicCalibrator.ResultState = \
            IntrinsicCalibrator.ResultState[self._result_state_selector.selector.GetStringSelection()]
        result_label: str = self._result_label_textbox.textbox.GetValue()
        self._controller.calibrate_intrinsic_result_metadata_update(
            detector_label=selected_detector_label,
            result_identifier=result_identifier,
            result_state=result_state,
            result_label=result_label,
            callback=self._on_response_result_update)
        self._awaiting_user_task = True
        self._update_ui_controls()

    def _on_response_calibrate(
        self,
        component_label: str,
        result_identifier: str,
        intrinsic_calibration: IntrinsicCalibration
    ) -> None:
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message=f"intrinsics_panel._on_response_calibrate called in response to {component_label}.")
        self._calibrate_status_textbox.SetForegroundColour(colour=wx.Colour(red=0, green=0, blue=127, alpha=255))
        self._calibrate_status_textbox.SetValue(
            f"Calibration {result_identifier} from {component_label} complete - values: "
            f"{str(intrinsic_calibration.calibrated_values.as_array())}")
        self._result_display_textbox.SetValue(intrinsic_calibration.model_dump_json(indent=4))
        self._force_last_result_selected = True
        self._reload_metadata()

    def _on_response_delete_staged(
        self,
        component_label: str
    ) -> None:
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message=f"intrinsics_panel._on_response_delete_staged called in response to {component_label}.")
        self._reload_metadata()

    # noinspection PyUnusedLocal
    def _on_response_image_add(
        self,
        component_label: str,
        image_identifier: str
    ) -> None:
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message=f"intrinsics_panel._on_response_image_add called in response to {component_label}.")
        self._reload_metadata()

    def _on_response_image_get(
        self,
        component_label: str,
        image_base64: str
    ) -> None:
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message=f"intrinsics_panel._on_response_image_get called in response to {component_label}.")
        self._preview_image_checkbox.checkbox.SetValue(False)
        self._preview_panel.set_draw_image(True)
        self._preview_panel.update_image(image_base64=image_base64)

    def _on_response_image_update(
        self,
        component_label: str
    ) -> None:
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message=f"intrinsics_panel._on_response_image_update called in response to {component_label}.")
        self._reload_metadata()

    def _on_response_metadata_list(
        self,
        component_label: str,
        image_metadata_list: list[IntrinsicCalibrator.ImageMetadata],
        result_metadata_list: list[IntrinsicCalibrator.ResultMetadata]
    ) -> None:
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message=f"intrinsics_panel._on_response_metadata_list called in response to {component_label}.")
        self._image_metadata_list = image_metadata_list
        self._image_table.update_contents(row_contents=self._image_metadata_list)
        self._result_metadata_list = result_metadata_list
        self._result_table.update_contents(row_contents=self._result_metadata_list)
        if self._force_last_result_selected:
            self._result_table.set_selected_row_index(len(self._result_metadata_list) - 1)
            self._force_last_result_selected = False

    def _on_response_resolutions_list(
        self,
        component_label: str,
        resolutions: list[ImageResolution]
    ) -> None:
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message=f"intrinsics_panel._on_response_resolutions_list called in response to {component_label}.")
        self._detector_resolutions = resolutions
        self._detector_resolution_selector.set_options([str(res) for res in self._detector_resolutions])
        self._update_ui_controls()

    def _on_response_result_get(
        self,
        component_label: str,
        intrinsic_calibration: IntrinsicCalibration
    ) -> None:
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message=f"intrinsics_panel._on_response_result_get called in response to {component_label}.")
        self._result_display_textbox.SetValue(str(intrinsic_calibration.model_dump_json(indent=4)))

    def _on_response_result_update(
        self,
        component_label: str
    ) -> None:
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message=f"intrinsics_panel._on_response_result_update called in response to {component_label}.")
        self._reload_metadata()

    def _reload_metadata(self) -> None:
        self._metadata_needs_update = True

    def update_loop(self):
        super().update_loop()
        if self._awaiting_user_task:
            if not self._controller.is_user_task_running():
                self._awaiting_user_task = False
                self._update_ui_controls()
        if not self._awaiting_user_task and self._metadata_needs_update:
            self._image_metadata_list = list()
            self._result_metadata_list = list()
            self._calibrate_status_textbox.SetValue(str())
            self._result_display_textbox.SetValue(str())
            selected_detector_label: str = self._detector_selector.selector.GetStringSelection()
            selected_detector_resolution: ImageResolution = \
                ImageResolution.from_str(self._detector_resolution_selector.selector.GetStringSelection())
            self._controller.calibrate_intrinsic_metadata_list(
                detector_label=selected_detector_label,
                image_resolution=selected_detector_resolution,
                callback=self._on_response_metadata_list)
            self._metadata_needs_update = False
            self._update_ui_controls()
            self._awaiting_user_task = True
        selected_detector_label: str = self._detector_selector.selector.GetStringSelection()
        if (
            (selected_detector_label is not None) and
            (len(selected_detector_label) > 0)
        ):
            if self._preview_image_checkbox.checkbox.GetValue():
                detector_live_data: MCTController.DetectorLiveData = \
                    self._controller.get_live_detector_data(detector_label=selected_detector_label)
                self._preview_panel.update_image(
                    image_base64=detector_live_data.frame.image_base64,
                    capture_resolution=detector_live_data.camera_resolution)
            elif self._image_table.get_selected_row_index() is None:
                self._preview_panel.update_image()
        else:
            self._preview_panel.update_image()

    def _update_ui_controls(self) -> None:
        self._detector_selector.Enable(False)
        self._detector_resolution_selector.Enable(False)
        self._preview_image_checkbox.Enable(False)
        self._capture_button.Enable(False)
        self._calibrate_button.Enable(False)
        self._calibrate_status_textbox.Enable(False)
        self._reload_metadata_button.Enable(False)
        self._image_table.Enable(False)
        self._image_label_textbox.Enable(False)
        self._image_label_textbox.textbox.SetValue(str())
        self._image_state_selector.Enable(False)
        self._image_state_selector.selector.SetStringSelection(str())
        self._image_update_button.Enable(False)
        self._result_table.Enable(False)
        self._result_display_textbox.Enable(False)
        self._result_label_textbox.Enable(False)
        self._result_label_textbox.textbox.SetValue(str())
        self._result_state_selector.Enable(False)
        self._result_state_selector.selector.SetStringSelection(str())
        self._result_update_button.Enable(False)
        self._delete_staged_button.Enable(False)
        if self._awaiting_user_task:
            return  # We're waiting for something
        self._detector_selector.Enable(True)
        if len(self._detector_resolutions) <= 0:
            return
        self._detector_resolution_selector.Enable(True)
        resolution: str = self._detector_resolution_selector.selector.GetStringSelection()
        if len(resolution) <= 0:
            return
        self._preview_image_checkbox.Enable(True)
        self._capture_button.Enable(True)
        self._reload_metadata_button.Enable(True)
        # == NO RETURN GUARDS AFTER THIS POINT ==
        if len(self._image_metadata_list) > 0:
            self._image_table.Enable(True)
            image_index: int | None = self._image_table.get_selected_row_index()
            if image_index is not None:
                if image_index >= len(self._image_metadata_list):
                    self._status_message_source.enqueue_status_message(
                        severity=SeverityLabel.WARNING,
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
            self._delete_staged_button.Enable(True)
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
                    self._status_message_source.enqueue_status_message(
                        severity=SeverityLabel.WARNING,
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
            self._delete_staged_button.Enable(True)
        self.Layout()
        self.Refresh()
        self.Update()
