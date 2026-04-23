from .base_panel import \
    BasePanel
from .parameters import \
    ParameterBase, \
    ParameterCheckbox, \
    ParameterSpinboxFloat, \
    ParameterSelector
from .specialized import \
    DetectorSingleFramePanel
from src.common import \
    ImageFormat, \
    ImageResolution, \
    KeyValueMetaAny, \
    KeyValueSimpleAny
from src.controller import \
    MCTController
import logging
import wx


logger = logging.getLogger(__name__)


class DetectorPanel(BasePanel):

    _controller: MCTController

    _detector_selector: ParameterSelector

    _preview_scale_factor: ParameterSpinboxFloat
    _preview_image_checkbox: ParameterCheckbox
    _annotate_detected_checkbox: ParameterCheckbox
    _annotate_rejected_checkbox: ParameterCheckbox

    _camera_parameter_panel: wx.Panel
    _camera_parameter_sizer: wx.BoxSizer
    _camera_parameter_uis: list[ParameterBase]

    _annotator_parameter_panel: wx.Panel
    _annotator_parameter_sizer: wx.BoxSizer
    _annotator_parameter_uis: list[ParameterBase]

    _send_detector_parameters_button: wx.Button

    _preview_panel: DetectorSingleFramePanel

    _awaiting_user_task: bool

    def __init__(
        self,
        parent: wx.Window,
        controller: MCTController,
        name: str = "DetectorPanel"
    ):
        super().__init__(
            parent=parent,
            name=name)
        self._controller = controller

        self._camera_parameter_uis = list()
        self._annotator_parameter_uis = list()
        self._awaiting_user_task = False

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

        self.add_horizontal_line_to_spacer(
            parent=control_panel,
            sizer=control_sizer)

        self.add_text_label(
            parent=control_panel,
            sizer=control_sizer,
            label="Detector",
            font_size_delta=2,
            bold=True)

        self._preview_image_checkbox = self.add_control_checkbox(
            parent=control_panel,
            sizer=control_sizer,
            label="Preview Image")

        self._preview_scale_factor = self.add_control_spinbox_float(
            parent=control_panel,
            sizer=control_sizer,
            label="Preview Scale",
            minimum_value=0.03125,  # 1/32 in each dimension, for a minimum of 1/1024 original resolution
            maximum_value=1,  # No scaling
            initial_value=0.25,  # 1/4 in each dimension, for a default of 1/16 original resolution
            step_value=0.125,
            digit_count=4)

        self._annotate_detected_checkbox = self.add_control_checkbox(
            parent=control_panel,
            sizer=control_sizer,
            label="Annotate Detected")

        self._annotate_rejected_checkbox = self.add_control_checkbox(
            parent=control_panel,
            sizer=control_sizer,
            label="Annotate Rejected")

        self._send_detector_parameters_button = self.add_control_button(
            parent=control_panel,
            sizer=control_sizer,
            label="Sync Detector Parameters")

        self.add_horizontal_line_to_spacer(
            parent=control_panel,
            sizer=control_sizer)

        self.add_text_label(
            parent=control_panel,
            sizer=control_sizer,
            label="Capture",
            font_size_delta=2,
            bold=True)

        self._camera_parameter_panel: wx.Panel = wx.Panel(parent=control_panel)
        self._camera_parameter_sizer: wx.BoxSizer = wx.BoxSizer(orient=wx.VERTICAL)
        self._camera_parameter_panel.SetSizer(sizer=self._camera_parameter_sizer)
        control_sizer.Add(
            window=self._camera_parameter_panel,
            flags=wx.SizerFlags(0).Expand())

        self.add_horizontal_line_to_spacer(
            parent=control_panel,
            sizer=control_sizer)

        self.add_text_label(
            parent=control_panel,
            sizer=control_sizer,
            label="Detection",
            font_size_delta=2,
            bold=True)

        self._annotator_parameter_panel: wx.Panel = wx.Panel(parent=control_panel)
        self._annotator_parameter_sizer: wx.BoxSizer = wx.BoxSizer(orient=wx.VERTICAL)
        self._annotator_parameter_panel.SetSizer(sizer=self._annotator_parameter_sizer)
        control_sizer.Add(
            window=self._annotator_parameter_panel,
            flags=wx.SizerFlags(0).Expand())

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
            flags=wx.SizerFlags(35).Expand())

        self._preview_panel = DetectorSingleFramePanel(parent=self)
        self._preview_panel.SetBackgroundColour(colour=wx.BLACK)
        horizontal_split_sizer.Add(
            window=self._preview_panel,
            flags=wx.SizerFlags(65).Expand())

        self.SetSizerAndFit(sizer=horizontal_split_sizer)

        self._detector_selector.selector.Bind(
            event=wx.EVT_CHOICE,
            handler=self.on_ui_detector_selected)
        self._preview_image_checkbox.checkbox.Bind(
            event=wx.EVT_CHECKBOX,
            handler=self.on_ui_preview_image_settings_changed)
        self._preview_scale_factor.Bind(
            event=wx.EVT_SPIN,
            handler=self.on_ui_preview_image_settings_changed)
        self._annotate_detected_checkbox.checkbox.Bind(
            event=wx.EVT_CHECKBOX,
            handler=self.on_ui_preview_image_settings_changed)
        self._annotate_rejected_checkbox.checkbox.Bind(
            event=wx.EVT_CHECKBOX,
            handler=self.on_ui_preview_image_settings_changed)
        self._send_detector_parameters_button.Bind(
            event=wx.EVT_BUTTON,
            handler=self.on_ui_detector_sync_parameters_pressed)

        self._update_ui_controls()

    def on_ui_page_select(self):
        super().on_ui_page_select()
        available_detector_labels: list[str] = self._controller.get_remote_labels_detectors()
        self._detector_selector.set_options(option_list=available_detector_labels)
        self._update_ui_controls()

    def on_ui_page_deselect(self) -> None:
        super().on_ui_page_deselect()
        # Some cleanup in case settings were changed.
        self._controller.set_detector_includes_images(False)
        self._controller.set_detector_includes_annotations_detected(True)
        self._controller.set_detector_includes_annotations_rejected(False)

    def on_ui_detector_selected(self, _event: wx.CommandEvent):
        selected_detector_label: str = self._detector_selector.selector.GetStringSelection()
        self._controller.detector_parameters_get(
            detector_label=selected_detector_label,
            callback=self.on_response_detector_parameters_received)
        self._awaiting_user_task = True
        self._update_ui_controls()

    def on_ui_detector_sync_parameters_pressed(self, _event: wx.CommandEvent):
        selected_detector_label: str = self._detector_selector.selector.GetStringSelection()
        camera_parameters: list[KeyValueSimpleAny] = self.populate_key_value_list_from_dynamic_ui(
            parameter_uis=self._camera_parameter_uis)
        annotator_parameters: list[KeyValueSimpleAny] = self.populate_key_value_list_from_dynamic_ui(
            parameter_uis=self._annotator_parameter_uis)
        self._controller.detector_parameters_set(
            detector_label=selected_detector_label,
            camera_resolution=None,
            camera_parameters=camera_parameters,
            annotator_parameters=annotator_parameters,
            callback=self.on_response_detector_parameters_received)
        self._awaiting_user_task = True
        self._update_ui_controls()

    def on_ui_preview_image_settings_changed(self, _event: wx.CommandEvent):
        selected_detector_label: str = self._detector_selector.selector.GetStringSelection()
        if self._preview_image_checkbox.checkbox.GetValue():
            base_resolution: ImageResolution | None = \
                self._controller.get_live_detector_data(detector_label=selected_detector_label).camera_resolution
            scaled_resolution: ImageResolution | None = None
            if base_resolution is not None:
                scaled_resolution = ImageResolution(
                    x_px=round(self._preview_scale_factor.get_value() * base_resolution.x_px),
                    y_px=round(self._preview_scale_factor.get_value() * base_resolution.y_px))
            self._controller.set_detector_includes_images(
                enabled=True,
                image_format=ImageFormat.FORMAT_JPG,
                image_resolution=scaled_resolution)
            self._preview_panel.set_draw_image(enabled=True)
        else:
            self._controller.set_detector_includes_images(enabled=False)
            self._preview_panel.set_draw_image(enabled=False)
        do_detected_annotations: bool = self._annotate_detected_checkbox.checkbox.GetValue()
        # In normal use, Detector should always return the detected annotations, so we won't touch that setting here
        self._preview_panel.set_draw_annotations_detected(enabled=do_detected_annotations)
        do_rejected_annotations: bool = self._annotate_rejected_checkbox.checkbox.GetValue()
        self._controller.set_detector_includes_annotations_rejected(enabled=do_rejected_annotations)
        self._preview_panel.set_draw_annotations_rejected(enabled=do_rejected_annotations)

    # noinspection DuplicatedCode, PyUnusedLocal
    def on_response_detector_parameters_received(
        self,
        component_label: str | None = None,
        camera_resolution: ImageResolution | None = None,
        camera_parameters: list[KeyValueMetaAny] | None = None,
        annotator_parameters: list[KeyValueMetaAny] | None = None
    ):
        if camera_parameters is not None:
            self._camera_parameter_panel.Freeze()
            self._camera_parameter_sizer.Clear(True)
            self._camera_parameter_sizer = wx.BoxSizer(orient=wx.VERTICAL)
            self._camera_parameter_uis = self.populate_dynamic_ui_from_key_value_list(
                key_value_list=camera_parameters,
                containing_panel=self._camera_parameter_panel,
                containing_sizer=self._camera_parameter_sizer)
            self._camera_parameter_panel.SetSizer(self._camera_parameter_sizer)
            self._camera_parameter_panel.Thaw()
            self.Layout()
        if annotator_parameters is not None:
            self._annotator_parameter_panel.Freeze()
            self._annotator_parameter_sizer.Clear(True)
            self._annotator_parameter_sizer = wx.BoxSizer(orient=wx.VERTICAL)
            self._annotator_parameter_uis = self.populate_dynamic_ui_from_key_value_list(
                key_value_list=annotator_parameters,
                containing_panel=self._annotator_parameter_panel,
                containing_sizer=self._annotator_parameter_sizer)
            self._annotator_parameter_panel.SetSizer(self._annotator_parameter_sizer)
            self._annotator_parameter_panel.Thaw()
            self.Layout()

    def _set_display_controls_enabled(
        self,
        enable: bool
    ):
        self._preview_image_checkbox.Enable(enable=enable)
        self._preview_scale_factor.Enable(enable=enable)
        self._annotate_detected_checkbox.Enable(enable=enable)
        self._annotate_rejected_checkbox.Enable(enable=enable)

    def _set_parameter_controls_enabled(
        self,
        enable: bool
    ):
        for parameter_ui in self._camera_parameter_uis:
            parameter_ui.set_enabled(enable=enable)
        for parameter_ui in self._annotator_parameter_uis:
            parameter_ui.set_enabled(enable=enable)
        self._send_detector_parameters_button.Enable(enable=enable)

    def update_loop(self):
        super().update_loop()
        if self._awaiting_user_task:
            if not self._controller.is_user_task_running():
                self._awaiting_user_task = False
                self._update_ui_controls()
        selected_detector_label: str = self._detector_selector.selector.GetStringSelection()
        if selected_detector_label is not None and len(selected_detector_label) > 0:
            detector_live_data: MCTController.DetectorLiveData = \
                self._controller.get_live_detector_data(detector_label=selected_detector_label)
            self._preview_panel.update_image(
                capture_resolution=detector_live_data.camera_resolution,
                image_base64=detector_live_data.frame.image_base64,
                annotations=detector_live_data.frame.annotations)
        else:
            self._preview_panel.update_image()

    def _update_ui_controls(self):
        self._detector_selector.set_enabled(enable=False)
        self._set_display_controls_enabled(enable=False)
        self._set_parameter_controls_enabled(enable=False)
        if not self._controller.get_controller_state() == MCTController.State.RUNNING:
            return
        self._detector_selector.set_enabled(enable=True)
        selected_detector_label: str = self._detector_selector.selector.GetStringSelection()
        if selected_detector_label is None or len(selected_detector_label) <= 0:
            return
        if self._controller.is_user_task_running():
            return
        self._set_display_controls_enabled(enable=True)
        self._set_parameter_controls_enabled(enable=True)
