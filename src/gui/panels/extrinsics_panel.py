from .base_panel import \
    BasePanel
from .parameters import \
    ParameterSelector, \
    ParameterText
from .specialized import \
    CalibrationImageTable, \
    CalibrationResultTable, \
    DetectorMultiFramePanel
from src.common import \
    ExtrinsicCalibration, \
    ExtrinsicCalibrator, \
    ImageFormat, \
    SeverityLabel, \
    StatusMessageSource
from src.controller import \
    MCTController
import logging
import numpy
import wx
import wx.grid


logger = logging.getLogger(__name__)
_PREVIEW_CAPTURE_FORMAT: ImageFormat = ImageFormat.FORMAT_JPG


class ExtrinsicsPanel(BasePanel):

    _controller: MCTController
    _status_message_source: StatusMessageSource

    _mixer_selector: ParameterSelector
    _preview_toggle_button: wx.ToggleButton
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
    _preview_panel: DetectorMultiFramePanel

    _awaiting_user_task: bool
    _metadata_needs_update: bool
    _force_last_result_selected: bool
    _image_metadata_list: list[ExtrinsicCalibrator.ImageMetadata]
    _result_metadata_list: list[ExtrinsicCalibrator.ResultMetadata]

    _extrinsic_image: numpy.ndarray | None

    def __init__(
        self,
        parent: wx.Window,
        controller: MCTController,
        name: str = "ExtrinsicsPanel"
    ):
        super().__init__(
            parent=parent,
            name=name)
        self._controller = controller
        self._status_message_source = controller.get_status_message_source()

        self._awaiting_user_task = False
        self._metadata_needs_update = False
        self._force_last_result_selected = False
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

        self._preview_toggle_button = wx.ToggleButton(
            parent=control_panel,
            label="Preview Images")
        control_sizer.Add(
            window=self._preview_toggle_button,
            flags=wx.SizerFlags(0).Expand())
        control_sizer.AddSpacer(size=BasePanel.DEFAULT_SPACING_PX_VERTICAL)

        self._capture_button = self.add_control_button(
            parent=control_panel,
            sizer=control_sizer,
            label="Capture Calibration Images")

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

        self._reload_metadata_button = self.add_control_button(
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
            selectable_values=[state.name for state in ExtrinsicCalibrator.ImageState])

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
            selectable_values=[state.name for state in ExtrinsicCalibrator.ResultState])

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

        self._preview_panel = DetectorMultiFramePanel(parent=self)
        self._preview_panel.SetBackgroundColour(colour=wx.BLACK)
        horizontal_split_sizer.Add(
            window=self._preview_panel,
            flags=wx.SizerFlags(50).Expand())

        self.SetSizerAndFit(sizer=horizontal_split_sizer)

        self._mixer_selector.selector.Bind(
            event=wx.EVT_CHOICE,
            handler=self._on_ui_mixer_selected)
        self._preview_toggle_button.Bind(
            event=wx.EVT_TOGGLEBUTTON,
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
        selected_mixer_label: str = self._mixer_selector.selector.GetStringSelection()
        available_mixer_labels: list[str] = self._controller.get_remote_labels_mixer()
        self._mixer_selector.set_options(option_list=available_mixer_labels)
        if selected_mixer_label in available_mixer_labels:
            self._mixer_selector.selector.SetStringSelection(selected_mixer_label)
        else:
            self._mixer_selector.selector.SetStringSelection(str())
        self._update_ui_controls()

    def on_ui_page_deselect(self) -> None:
        super().on_ui_page_deselect()
        # Some cleanup in case settings were changed.
        self._controller.set_detector_includes_images(False)
        self._controller.set_detector_includes_annotations_detected(True)
        self._controller.set_detector_includes_annotations_rejected(False)

    def _on_ui_calibrate_pressed(self, _event: wx.CommandEvent) -> None:
        # TODO: Need to sync intrinsics or calibration may be incorrect
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message=f"extrinsics_panel._on_ui_calibrate_pressed called.")
        self._calibrate_status_textbox.SetForegroundColour(colour=wx.Colour(red=0, green=0, blue=0, alpha=255))
        self._calibrate_status_textbox.SetValue("Calibrating...")
        self._result_display_textbox.SetValue(str())
        selected_mixer_label: str = self._mixer_selector.selector.GetStringSelection()
        self._controller.calibrate_extrinsic_calculate(
            mixer_label=selected_mixer_label,
            callback=self._on_response_calibrate)
        self._awaiting_user_task = True
        self._update_ui_controls()

    def _on_ui_capture_pressed(self, _event: wx.CommandEvent) -> None:
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message=f"extrinsics_panel._on_ui_capture_pressed called.")
        selected_mixer_label: str = self._mixer_selector.selector.GetStringSelection()
        self._controller.calibrate_extrinsic_image_add(
            mixer_label=selected_mixer_label,
            callback=self._on_response_image_add)
        self._awaiting_user_task = True
        self._update_ui_controls()

    def _on_ui_delete_staged_pressed(self, _event: wx.CommandEvent) -> None:
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message=f"extrinsics_panel._on_ui_delete_staged_pressed called.")
        selected_mixer_label: str = self._mixer_selector.selector.GetStringSelection()
        self._controller.calibrate_extrinsic_delete_staged(
            mixer_label=selected_mixer_label,
            callback=self._on_response_delete_staged)
        self._awaiting_user_task = True
        self._update_ui_controls()

    def _on_ui_image_metadata_selected(self, _event: wx.grid.GridEvent) -> None:
        if self._awaiting_user_task:
            return  # Not initiated by user
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message=f"extrinsics_panel._on_ui_image_metadata_selected called.")
        image_index: int = self._image_table.get_selected_row_index()
        image_identifier: str | None = self._image_metadata_list[image_index].identifier
        if image_identifier is not None:
            selected_mixer_label: str = self._mixer_selector.selector.GetStringSelection()
            self._controller.calibrate_extrinsic_image_get(
                mixer_label=selected_mixer_label,
                image_identifier=image_identifier,
                callback=self._on_response_image_get)
            self._awaiting_user_task = True
        self._update_ui_controls()

    def _on_ui_image_update_pressed(self, _event: wx.CommandEvent) -> None:
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message=f"extrinsics_panel._on_ui_image_update_pressed called.")
        self._calibrate_status_textbox.SetValue(str())
        selected_mixer_label: str = self._mixer_selector.selector.GetStringSelection()
        image_index: int = self._image_table.get_selected_row_index()
        image_identifier: str = self._image_metadata_list[image_index].identifier
        # noinspection PyTypeChecker
        image_state: ExtrinsicCalibrator.ImageState = \
            ExtrinsicCalibrator.ImageState[self._image_state_selector.selector.GetStringSelection()]
        image_label: str = self._image_label_textbox.textbox.GetValue()
        self._controller.calibrate_extrinsic_image_metadata_update(
            mixer_label=selected_mixer_label,
            image_identifier=image_identifier,
            image_state=image_state,
            image_label=image_label,
            callback=self._on_response_image_update)
        self._awaiting_user_task = True
        self._update_ui_controls()

    def _on_ui_metadata_reload_pressed(self, _event: wx.CommandEvent) -> None:
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message=f"extrinsics_panel._on_ui_metadata_reload_pressed called.")
        self._reload_metadata()

    def _on_ui_mixer_selected(self, _event: wx.CommandEvent) -> None:
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message=f"extrinsics_panel._on_ui_mixer_selected called.")
        self._image_metadata_list = list()
        self._result_metadata_list = list()
        self._calibrate_status_textbox.SetValue(str())
        self._result_display_textbox.SetValue(str())
        self._reload_metadata()

    def _on_ui_preview_toggled(self, _event: wx.CommandEvent):
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message=f"extrinsics_panel._on_ui_preview_toggled called.")
        preview_on: bool = self._preview_toggle_button.GetValue()
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
            message=f"extrinsics_panel._on_ui_result_metadata_selected called.")
        self._result_display_textbox.SetValue(str())
        result_index: int = self._result_table.get_selected_row_index()
        result_identifier: str | None = self._result_metadata_list[result_index].identifier
        if result_identifier is not None:
            selected_mixer_label: str = self._mixer_selector.selector.GetStringSelection()
            self._controller.calibrate_extrinsic_result_get(
                mixer_label=selected_mixer_label,
                result_identifier=result_identifier,
                callback=self._on_response_result_get)
            self._awaiting_user_task = True
        self._update_ui_controls()

    def _on_ui_result_update_pressed(self, _event: wx.CommandEvent) -> None:
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message=f"extrinsics_panel._on_ui_result_update_pressed called.")
        self._result_display_textbox.SetValue(str())
        selected_mixer_label: str = self._mixer_selector.selector.GetStringSelection()
        result_index: int = self._result_table.get_selected_row_index()
        result_identifier: str = self._result_metadata_list[result_index].identifier
        # noinspection PyTypeChecker
        result_state: ExtrinsicCalibrator.ResultState = \
            ExtrinsicCalibrator.ResultState[self._result_state_selector.selector.GetStringSelection()]
        result_label: str = self._result_label_textbox.textbox.GetValue()
        self._controller.calibrate_extrinsic_result_metadata_update(
            mixer_label=selected_mixer_label,
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
        extrinsic_calibration: ExtrinsicCalibration
    ) -> None:
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message=f"extrinsics_panel._on_response_calibrate called in response to {component_label}.")
        self._calibrate_status_textbox.SetForegroundColour(colour=wx.Colour(red=0, green=0, blue=127, alpha=255))
        self._calibrate_status_textbox.SetValue(
            f"Calibration {result_identifier} from {component_label} complete.")
        self._result_display_textbox.SetValue(extrinsic_calibration.model_dump_json(indent=4))
        self._force_last_result_selected = True
        self._reload_metadata()

    def _on_response_delete_staged(
        self,
        component_label: str
    ) -> None:
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message=f"extrinsics_panel._on_response_delete_staged called in response to {component_label}.")
        self._reload_metadata()

    # noinspection PyUnusedLocal
    def _on_response_image_add(
        self,
        component_label: str,
        image_identifiers: list[str]
    ) -> None:
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message=f"extrinsics_panel._on_response_image_add called in response to {component_label}.")
        self._reload_metadata()

    def _on_response_image_get(
        self,
        component_label: str,
        image_base64: str
    ) -> None:
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message=f"extrinsics_panel._on_response_image_get called in response to {component_label}.")
        self._preview_toggle_button.SetValue(False)
        self._preview_panel.set_draw_image(True)
        self._preview_panel.update_image(images_base64=[image_base64])

    def _on_response_image_update(
        self,
        component_label: str
    ) -> None:
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message=f"extrinsics_panel._on_response_image_update called in response to {component_label}.")
        self._reload_metadata()

    def _on_response_metadata_list(
        self,
        component_label: str,
        image_metadata_list: list[ExtrinsicCalibrator.ImageMetadata],
        result_metadata_list: list[ExtrinsicCalibrator.ResultMetadata]
    ) -> None:
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message=f"extrinsics_panel._on_response_metadata_list called in response to {component_label}.")
        self._image_metadata_list = image_metadata_list
        self._image_table.update_contents(row_contents=self._image_metadata_list)
        self._result_metadata_list = result_metadata_list
        self._result_table.update_contents(row_contents=self._result_metadata_list)
        if self._force_last_result_selected:
            self._result_table.set_selected_row_index(len(self._result_metadata_list) - 1)
            self._force_last_result_selected = False

    def _on_response_result_get(
        self,
        component_label: str,
        extrinsic_calibration: ExtrinsicCalibration
    ) -> None:
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message=f"extrinsics_panel._on_response_result_get called in response to {component_label}.")
        self._result_display_textbox.SetValue(str(extrinsic_calibration.model_dump_json(indent=4)))

    def _on_response_result_update(
        self,
        component_label: str
    ) -> None:
        self._status_message_source.enqueue_status_message(
            severity=SeverityLabel.DEBUG,
            message=f"extrinsics_panel._on_response_result_update called in response to {component_label}.")
        self._reload_metadata()

    def _reload_metadata(self) -> None:
        self._metadata_needs_update = True

    def update_loop(self) -> None:
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
            selected_mixer_label: str = self._mixer_selector.selector.GetStringSelection()
            self._controller.calibrate_extrinsic_metadata_list(
                mixer_label=selected_mixer_label,
                callback=self._on_response_metadata_list)
            self._metadata_needs_update = False
            self._update_ui_controls()
            self._awaiting_user_task = True
        selected_mixer_label: str = self._mixer_selector.selector.GetStringSelection()
        if (
            (selected_mixer_label is not None) and
            (len(selected_mixer_label) > 0)
        ):
            if self._preview_toggle_button.GetValue():
                detector_labels: list[str] = self._controller.get_remote_labels_detectors()
                images_base64: list[str] = [
                    self._controller.get_live_detector_data(detector_label=detector_label).frame.image_base64
                    for detector_label in detector_labels]
                self._preview_panel.update_image(images_base64=images_base64)
            elif self._image_table.get_selected_row_index() is None:
                self._preview_panel.update_image()
        else:
            self._preview_panel.update_image()

    def _update_ui_controls(self) -> None:
        self._mixer_selector.Enable(False)
        self._preview_toggle_button.Enable(False)
        self._capture_button.Enable(False)
        self._calibrate_button.Enable(False)
        self._reload_metadata_button.Enable(False)
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
        self._delete_staged_button.Enable(False)
        if self._awaiting_user_task:
            return  # We're waiting for something
        self._mixer_selector.Enable(True)
        mixer_label: str = self._mixer_selector.selector.GetStringSelection()
        if len(mixer_label) <= 0:
            self._preview_toggle_button.SetValue(False)
            return
        self._reload_metadata_button.Enable(True)
        self._preview_toggle_button.Enable(True)
        self._capture_button.Enable(True)
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
                    image_metadata: ExtrinsicCalibrator.ImageMetadata = self._image_metadata_list[image_index]
                    self._image_label_textbox.Enable(True)
                    self._image_label_textbox.textbox.SetValue(image_metadata.label)
                    self._image_state_selector.Enable(True)
                    self._image_state_selector.selector.SetStringSelection(image_metadata.state.name)
                    self._image_update_button.Enable(True)
            calibration_image_count: int = 0
            for image_metadata in self._image_metadata_list:
                if image_metadata.state == ExtrinsicCalibrator.ImageState.SELECT:
                    calibration_image_count += 1
            if calibration_image_count > 0:
                self._calibrate_button.Enable(True)
                self._calibrate_status_textbox.Enable(True)
            self._delete_staged_button.Enable(True)
        if len(self._result_metadata_list) > 0:
            self._result_table.Enable(True)
            result_index: int | None = self._result_table.get_selected_row_index()
            if result_index is not None:
                if result_index >= len(self._result_metadata_list):
                    self._status_message_source.enqueue_status_message(
                        severity=SeverityLabel.WARNING,
                        message=f"Selected result index {result_index} is out of bounds. Setting to None.")
                    self._result_table.set_selected_row_index(None)
                else:
                    result_metadata: ExtrinsicCalibrator.ResultMetadata = self._result_metadata_list[result_index]
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
