from .base_panel import \
    BasePanel
from .feedback import \
    ImagePanel
from .parameters import \
    ParameterBase, \
    ParameterCheckbox, \
    ParameterSpinboxFloat, \
    ParameterSelector
from src.common import \
    Annotation, \
    DetectorFrame, \
    ImageFormat, \
    ImageResolution, \
    ImageUtils, \
    KeyValueMetaAny, \
    KeyValueSimpleAny
from src.controller import \
    MCTController
import cv2
from io import BytesIO
import logging
import numpy
import wx


logger = logging.getLogger(__name__)


def _marker_snapshot_list_to_opencv_points(
    marker_snapshot_list: list[Annotation],
    scale: float
) -> numpy.ndarray:
    if len(marker_snapshot_list) <= 0:
        return numpy.asarray([], dtype=numpy.int32)
    return_value: list[list[list[float]]] = list()
    current_base_label: str | None = None
    current_shape_points: list[list[float]] | None = None
    for marker_snapshot in marker_snapshot_list:
        annotation_base_label = marker_snapshot.base_feature_label()
        # TODO: This is not robust when multiple unknown annotations are reported.
        #       Consider also looking at the number after Annotation.RELATION_CHARACTER
        #       It increases by exactly 1 when the annotations form a continuous shape
        if annotation_base_label != current_base_label:
            if current_shape_points is not None:
                return_value.append(current_shape_points)
            current_shape_points = list()
            current_base_label = annotation_base_label
        current_shape_points.append([
            marker_snapshot.x_px * scale,
            marker_snapshot.y_px * scale])
    return_value.append(current_shape_points)
    return_value: numpy.ndarray = numpy.asarray(return_value, dtype=numpy.int32)
    return return_value


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

    _image_panel: ImagePanel

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

        self._image_panel = ImagePanel(parent=self)
        self._image_panel.SetBackgroundColour(colour=wx.BLACK)
        horizontal_split_sizer.Add(
            window=self._image_panel,
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

    def on_ui_page_deselect(self):
        super().on_ui_page_deselect()
        if self._controller.get_controller_state() == MCTController.State.RUNNING and \
           self._controller.is_detector_image_collection_enabled():
            self._controller.disable_detector_image_collection()  # Save processing/bandwidth

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
            self._controller.enable_detector_image_collection(
                image_format=ImageFormat.FORMAT_JPG,
                image_resolution=scaled_resolution)
        else:
            self._controller.disable_detector_image_collection()
        self._update_ui_image()

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

        if self._preview_image_checkbox.checkbox.GetValue() or \
           self._annotate_detected_checkbox.checkbox.GetValue() or \
           self._annotate_rejected_checkbox.checkbox.GetValue():
            self._update_ui_image()

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

    def _update_ui_image(self):
        display_image: numpy.ndarray
        panel_size: wx.Size = self._image_panel.GetSize()
        panel_size_tuple: tuple[int, int] = (panel_size.x, panel_size.y)
        if not self._preview_image_checkbox.checkbox.GetValue():
            display_image = ImageUtils.black_image(resolution_px=panel_size_tuple)
        else:
            selected_detector_label: str = self._detector_selector.selector.GetStringSelection()
            detector_live_data: MCTController.DetectorLiveData = \
                self._controller.get_live_detector_data(detector_label=selected_detector_label)
            detector_frame: DetectorFrame = detector_live_data.frame
            scale: float | None
            if detector_frame.image_base64 is not None:
                opencv_image: numpy.ndarray = ImageUtils.base64_to_image(input_base64=detector_frame.image_base64)
                display_image: numpy.ndarray = ImageUtils.image_resize_to_fit(
                    opencv_image=opencv_image,
                    available_size=panel_size_tuple)
                cv2.cvtColor(display_image, cv2.COLOR_RGB2BGR, display_image)
                scale: float = self._preview_scale_factor.get_value() * display_image.shape[0] / opencv_image.shape[0]
            else:
                display_image = ImageUtils.black_image(resolution_px=panel_size_tuple)
                rescaled_resolution_px: tuple[int, int] = ImageUtils.scale_factor_for_available_space_px(
                    source_resolution_px=(detector_frame.image_resolution.x_px, detector_frame.image_resolution.y_px),
                    available_size_px=panel_size_tuple)
                scale: float = rescaled_resolution_px[1] / detector_frame.image_resolution.y_px

            if scale is not None:
                if self._annotate_detected_checkbox.checkbox.GetValue():
                    identified_annotations: list[Annotation] = [
                        annotation
                        for annotation in detector_frame.annotations
                        if annotation.feature_label != Annotation.UNIDENTIFIED_LABEL]
                    corners: numpy.ndarray = _marker_snapshot_list_to_opencv_points(
                        marker_snapshot_list=identified_annotations,
                        scale=scale)
                    cv2.polylines(
                        img=display_image,
                        pts=corners,
                        isClosed=True,
                        color=[255, 191, 127],  # blue in BGR
                        thickness=2)
                if self._annotate_rejected_checkbox.checkbox.GetValue():
                    unidentified_annotations: list[Annotation] = [
                        annotation
                        for annotation in detector_frame.annotations
                        if annotation.feature_label == Annotation.UNIDENTIFIED_LABEL]
                    corners: numpy.ndarray = _marker_snapshot_list_to_opencv_points(
                        marker_snapshot_list=unidentified_annotations,
                        scale=scale)
                    cv2.polylines(
                        img=display_image,
                        pts=corners,
                        isClosed=True,
                        color=[127, 191, 255],  # orange in BGR
                        thickness=2)

        image_buffer: bytes = ImageUtils.image_to_bytes(image_data=display_image, image_format=ImageFormat.FORMAT_JPG)
        image_buffer_io: BytesIO = BytesIO(image_buffer)
        # noinspection PyTypeChecker
        wx_image: wx.Image = wx.Image(image_buffer_io)
        wx_bitmap: wx.Bitmap = wx_image.ConvertToBitmap()
        self._image_panel.set_bitmap(wx_bitmap)
        self._image_panel.paint()
