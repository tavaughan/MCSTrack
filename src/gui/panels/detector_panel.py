from .base_panel import \
    BasePanel
from .feedback import \
    ImagePanel
from .parameters import \
    ParameterBase, \
    ParameterCheckbox, \
    ParameterSelector, \
    ParameterSpinboxFloat, \
    ParameterSpinboxInteger
from src.common import \
    ErrorResponse, \
    EmptyResponse, \
    ImageCoding, \
    ImageUtils, \
    MCTRequestSeries, \
    MCTResponse, \
    MCTResponseSeries, \
    StandardResolutions, \
    StatusMessageSource
from src.common.structures import \
    CaptureFormat, \
    DetectionParameters, \
    DetectorFrame, \
    ImageResolution, \
    KeyValueSimpleAbstract, \
    KeyValueSimpleBool, \
    KeyValueSimpleString, \
    KeyValueSimpleFloat, \
    KeyValueSimpleInt, \
    KeyValueMetaAbstract, \
    KeyValueMetaBool, \
    KeyValueMetaEnum, \
    KeyValueMetaFloat, \
    KeyValueMetaInt, \
    MarkerSnapshot
from src.controller import \
    MCTController
from src.detector.api import \
    CalibrationImageAddRequest, \
    CalibrationImageAddResponse, \
    CameraImageGetRequest, \
    CameraImageGetResponse, \
    CameraParametersGetRequest, \
    CameraParametersGetResponse, \
    CameraParametersSetRequest, \
    CameraParametersSetResponse, \
    MarkerParametersGetRequest, \
    MarkerParametersGetResponse, \
    MarkerParametersSetRequest
import cv2
from io import BytesIO
import logging
import numpy
from typing import Final, Optional
import uuid
import wx


logger = logging.getLogger(__name__)

_UPDATE_INTERVAL_MILLISECONDS: Final[int] = 16
_SUPPORTED_RESOLUTIONS: Final[list[ImageResolution]] = [
    StandardResolutions.RES_640X480,
    StandardResolutions.RES_1280X720,
    StandardResolutions.RES_1920X1080]
_SUPPORTED_FPS: Final[list[str]] = [
    "15",
    "30",
    "60"]
_SUPPORTED_CORNER_REFINEMENT_METHODS: Final[list[str]] = [
    "NONE",
    "SUBPIX",
    "CONTOUR",
    "APRILTAG"]
_CAPTURE_FORMAT: CaptureFormat = ".jpg"

_CAMERA_PARAMETER_SLOT_COUNT: Final[int] = 100


class DetectorPanel(BasePanel):

    _controller: MCTController

    _control_blocking_request_id: uuid.UUID | None
    _live_preview_request_id: uuid.UUID | None

    _live_image_base64: str | None
    _live_markers_detected: list[MarkerSnapshot]
    _live_markers_rejected: list[MarkerSnapshot]

    _detector_selector: ParameterSelector
    _preview_image_checkbox: ParameterCheckbox
    _annotate_detected_checkbox: ParameterCheckbox
    _annotate_rejected_checkbox: ParameterCheckbox
    _send_capture_parameters_button: wx.Button
    _send_detection_parameters_button: wx.Button

    _camera_parameter_panel: wx.Panel
    _camera_parameter_sizer: wx.BoxSizer
    _camera_parameter_uis: list[ParameterBase]

    # Look at https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
    # for documentation on individual parameters

    _detection_param_adaptive_thresh_win_size_min: ParameterSpinboxInteger
    _detection_param_adaptive_thresh_win_size_max: ParameterSpinboxInteger
    _detection_param_adaptive_thresh_win_size_step: ParameterSpinboxInteger
    _detection_param_adaptive_thresh_constant: ParameterSpinboxFloat
    _detection_param_min_marker_perimeter_rate: ParameterSpinboxFloat
    _detection_param_max_marker_perimeter_rate: ParameterSpinboxFloat
    _detection_param_polygonal_approx_accuracy_rate: ParameterSpinboxFloat
    _detection_param_min_corner_distance_rate: ParameterSpinboxFloat
    _detection_param_min_marker_distance_rate: ParameterSpinboxFloat
    _detection_param_min_distance_to_border: ParameterSpinboxInteger
    _detection_param_marker_border_bits: ParameterSpinboxInteger
    _detection_param_min_otsu_std_dev: ParameterSpinboxFloat
    _detection_param_persp_rem_px_per_cell: ParameterSpinboxInteger
    _detection_param_persp_rem_marg_per_cell: ParameterSpinboxFloat
    _detection_param_max_erroneous_bits_border_rate: ParameterSpinboxFloat
    _detection_param_error_correction_rate: ParameterSpinboxFloat
    _detection_param_detect_inverted_marker: ParameterCheckbox
    _detection_param_corner_refinement_method: ParameterSelector
    _detection_param_corner_refinement_win_size: ParameterSpinboxInteger
    _detection_param_corner_refinement_max_iterations: ParameterSpinboxInteger
    _detection_param_corner_refinement_min_accuracy: ParameterSpinboxFloat
    _detection_param_april_tag_critical_rad: ParameterSpinboxFloat
    _detection_param_april_tag_deglitch: ParameterCheckbox
    _detection_param_april_tag_max_line_fit_mse: ParameterSpinboxFloat
    _detection_param_april_tag_max_nmaxima: ParameterSpinboxInteger
    _detection_param_april_tag_min_cluster_pixels: ParameterSpinboxFloat
    _detection_param_april_tag_min_white_black_diff: ParameterSpinboxFloat
    _detection_param_april_tag_quad_decimate: ParameterSpinboxFloat
    _detection_param_april_tag_quad_sigma: ParameterSpinboxFloat
    _detection_param_use_aruco_3_detection: ParameterCheckbox
    _detection_param_min_side_length_canonical_img: ParameterSpinboxInteger
    _detection_param_min_marker_length_ratio_orig: ParameterSpinboxFloat
    _calibration_capture_button: wx.Button

    _image_panel: ImagePanel

    def __init__(
        self,
        parent: wx.Window,
        controller: MCTController,
        status_message_source: StatusMessageSource,
        name: str = "DetectorPanel"
    ):
        super().__init__(
            parent=parent,
            status_message_source=status_message_source,
            name=name)
        self._controller = controller
        self._capture_active = False
        self._live_preview_request_id = None
        self._control_blocking_request_id = None

        self._live_image_base64 = None
        self._live_markers_detected = list()
        self._live_markers_rejected = list()

        self._camera_parameter_uis = list()

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

        self._annotate_detected_checkbox = self.add_control_checkbox(
            parent=control_panel,
            sizer=control_sizer,
            label="Annotate Detected")

        self._annotate_rejected_checkbox = self.add_control_checkbox(
            parent=control_panel,
            sizer=control_sizer,
            label="Annotate Rejected")

        self._calibration_capture_button: wx.Button = self.add_control_button(
            parent=control_panel,
            sizer=control_sizer,
            label="Capture Calibration Image")

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

        self._send_capture_parameters_button: wx.Button = self.add_control_button(
            parent=control_panel,
            sizer=control_sizer,
            label="Send Capture Parameters")

        self.add_horizontal_line_to_spacer(
            parent=control_panel,
            sizer=control_sizer)

        self.add_text_label(
            parent=control_panel,
            sizer=control_sizer,
            label="Detection",
            font_size_delta=2,
            bold=True)

        self.add_text_label(
            parent=control_panel,
            sizer=control_sizer,
            label="Adaptive Thresholding",
            bold=True)

        self._detection_param_adaptive_thresh_win_size_min = self.add_control_spinbox_integer(
            parent=control_panel,
            sizer=control_sizer,
            label="Window Size Min (px)",
            minimum_value=1,
            maximum_value=99,
            initial_value=3)

        self._detection_param_adaptive_thresh_win_size_max = self.add_control_spinbox_integer(
            parent=control_panel,
            sizer=control_sizer,
            label="Window Size Max (px)",
            minimum_value=1,
            maximum_value=99,
            initial_value=23)

        self._detection_param_adaptive_thresh_win_size_step = self.add_control_spinbox_integer(
            parent=control_panel,
            sizer=control_sizer,
            label="Window Size Step (px)",
            minimum_value=1,
            maximum_value=99,
            initial_value=10)

        self._detection_param_adaptive_thresh_constant = self.add_control_spinbox_float(
            parent=control_panel,
            sizer=control_sizer,
            label="Threshold Constant",
            minimum_value=-256.0,
            maximum_value=256.0,
            initial_value=7.0,
            step_value=1.0)

        self.add_text_label(
            parent=control_panel,
            sizer=control_sizer,
            label="Contour Filtering",
            bold=True)

        self._detection_param_min_marker_perimeter_rate = self.add_control_spinbox_float(
            parent=control_panel,
            sizer=control_sizer,
            label="Min Size Ratio",
            minimum_value=0.0,
            maximum_value=8.0,
            initial_value=0.03,
            step_value=0.01)

        self._detection_param_max_marker_perimeter_rate = self.add_control_spinbox_float(
            parent=control_panel,
            sizer=control_sizer,
            label="Max Size Ratio",
            minimum_value=0.0,
            maximum_value=8.0,
            initial_value=4.0,
            step_value=0.01)

        self._detection_param_polygonal_approx_accuracy_rate = self.add_control_spinbox_float(
            parent=control_panel,
            sizer=control_sizer,
            label="Square Tolerance Ratio",
            minimum_value=0.0,
            maximum_value=1.0,
            initial_value=0.05,
            step_value=0.01)

        self._detection_param_min_corner_distance_rate = self.add_control_spinbox_float(
            parent=control_panel,
            sizer=control_sizer,
            label="Corner Separation Ratio",
            minimum_value=0.0,
            maximum_value=1.0,
            initial_value=0.05,
            step_value=0.01)

        self._detection_param_min_marker_distance_rate = self.add_control_spinbox_float(
            parent=control_panel,
            sizer=control_sizer,
            label="Marker Separation Ratio",
            minimum_value=0.0,
            maximum_value=1.0,
            initial_value=0.05,
            step_value=0.01)

        self._detection_param_min_distance_to_border = self.add_control_spinbox_integer(
            parent=control_panel,
            sizer=control_sizer,
            label="Border Distance (px)",
            minimum_value=0,
            maximum_value=512,
            initial_value=3)

        self.add_text_label(
            parent=control_panel,
            sizer=control_sizer,
            label="Bits Extraction",
            bold=True)

        self._detection_param_marker_border_bits = self.add_control_spinbox_integer(
            parent=control_panel,
            sizer=control_sizer,
            label="Border width (px)",
            minimum_value=1,
            maximum_value=9,
            initial_value=1)

        self._detection_param_min_otsu_std_dev = self.add_control_spinbox_float(
            parent=control_panel,
            sizer=control_sizer,
            label="Min Brightness Stdev",
            minimum_value=0.0,
            maximum_value=256.0,
            initial_value=5.0,
            step_value=1.0)

        self._detection_param_persp_rem_px_per_cell = self.add_control_spinbox_integer(
            parent=control_panel,
            sizer=control_sizer,
            label="Bit Sampling Rate",
            minimum_value=1,
            maximum_value=20,
            initial_value=1)

        self._detection_param_persp_rem_marg_per_cell = self.add_control_spinbox_float(
            parent=control_panel,
            sizer=control_sizer,
            label="Bit Margin Ratio",
            minimum_value=0.0,
            maximum_value=0.5,
            initial_value=0.13,
            step_value=0.01)

        self.add_text_label(
            parent=control_panel,
            sizer=control_sizer,
            label="Marker Identification",
            bold=True)

        self._detection_param_max_erroneous_bits_border_rate = self.add_control_spinbox_float(
            parent=control_panel,
            sizer=control_sizer,
            label="Border Error Rate",
            minimum_value=0.0,
            maximum_value=1.0,
            initial_value=0.35,
            step_value=0.01)

        self._detection_param_error_correction_rate = self.add_control_spinbox_float(
            parent=control_panel,
            sizer=control_sizer,
            label="Error Correction Rate",
            minimum_value=0.0,
            maximum_value=1.0,
            initial_value=0.6,
            step_value=0.01)

        self._detection_param_detect_inverted_marker = self.add_control_checkbox(
            parent=control_panel,
            sizer=control_sizer,
            label="Detect Inverted",
            value=False)

        self.add_text_label(
            parent=control_panel,
            sizer=control_sizer,
            label="Corner Refinement",
            bold=True)

        self._detection_param_corner_refinement_method = self.add_control_selector(
            parent=control_panel,
            sizer=control_sizer,
            label="Method",
            selectable_values=_SUPPORTED_CORNER_REFINEMENT_METHODS)

        self._detection_param_corner_refinement_win_size = self.add_control_spinbox_integer(
            parent=control_panel,
            sizer=control_sizer,
            label="Window Size (px)",
            minimum_value=1,
            maximum_value=9,
            initial_value=5)

        self._detection_param_corner_refinement_max_iterations = self.add_control_spinbox_integer(
            parent=control_panel,
            sizer=control_sizer,
            label="Maximum Iterations",
            minimum_value=1,
            maximum_value=100,
            initial_value=30)

        self._detection_param_corner_refinement_min_accuracy = self.add_control_spinbox_float(
            parent=control_panel,
            sizer=control_sizer,
            label="Minimum Error",
            minimum_value=0.0,
            maximum_value=5.0,
            initial_value=0.1,
            step_value=0.01)

        self.add_text_label(
            parent=control_panel,
            sizer=control_sizer,
            label="April Tag Only",
            bold=True)

        self._detection_param_april_tag_critical_rad = self.add_control_spinbox_float(
            parent=control_panel,
            sizer=control_sizer,
            label="Crit Angle (Rad)",
            minimum_value=0.0,
            maximum_value=numpy.pi,
            initial_value=numpy.pi * 10.0 / 180.0,
            step_value=0.01)

        self._detection_param_april_tag_deglitch = self.add_control_checkbox(
            parent=control_panel,
            sizer=control_sizer,
            label="Deglitch",
            value=False)

        self._detection_param_april_tag_max_line_fit_mse = self.add_control_spinbox_float(
            parent=control_panel,
            sizer=control_sizer,
            label="Max Line Fit MSE",
            minimum_value=0.0,
            maximum_value=512.0,
            initial_value=10.0,
            step_value=0.01)

        self._detection_param_april_tag_max_nmaxima = self.add_control_spinbox_integer(
            parent=control_panel,
            sizer=control_sizer,
            label="Max NMaxima",
            minimum_value=1,
            maximum_value=100,
            initial_value=10)

        self._detection_param_april_tag_min_cluster_pixels = self.add_control_spinbox_float(
            parent=control_panel,
            sizer=control_sizer,
            label="Min Cluster Size",
            minimum_value=0.0,
            maximum_value=512.0,
            initial_value=5.0,
            step_value=0.1)

        self._detection_param_april_tag_min_white_black_diff = self.add_control_spinbox_float(
            parent=control_panel,
            sizer=control_sizer,
            label="White-Black Diff",
            minimum_value=0.0,
            maximum_value=256.0,
            initial_value=5.0,
            step_value=0.1)

        self._detection_param_april_tag_quad_decimate = self.add_control_spinbox_float(
            parent=control_panel,
            sizer=control_sizer,
            label="Quad Decimate",
            minimum_value=0.0,
            maximum_value=1.0,
            initial_value=0.0,
            step_value=0.01)

        self._detection_param_april_tag_quad_sigma = self.add_control_spinbox_float(
            parent=control_panel,
            sizer=control_sizer,
            label="Quad Sigma",
            minimum_value=0.0,
            maximum_value=1.0,
            initial_value=0.0,
            step_value=0.01)

        self.add_text_label(
            parent=control_panel,
            sizer=control_sizer,
            label="Aruco 3",
            bold=True)

        self._detection_param_use_aruco_3_detection = self.add_control_checkbox(
            parent=control_panel,
            sizer=control_sizer,
            label="Use Aruco 3",
            value=False)

        self._detection_param_min_side_length_canonical_img = self.add_control_spinbox_integer(
            parent=control_panel,
            sizer=control_sizer,
            label="Min Size (px)",
            minimum_value=1,
            maximum_value=512,
            initial_value=32)

        self._detection_param_min_marker_length_ratio_orig = self.add_control_spinbox_float(
            parent=control_panel,
            sizer=control_sizer,
            label="Min Size Ratio",
            minimum_value=0.0,
            maximum_value=1.0,
            initial_value=0.0,
            step_value=0.01)

        self._send_detection_parameters_button = self.add_control_button(
            parent=control_panel,
            sizer=control_sizer,
            label="Send Detection Parameters")

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
            handler=self.on_detector_selected)
        self._preview_image_checkbox.checkbox.Bind(
            event=wx.EVT_CHECKBOX,
            handler=self.on_display_mode_changed)
        self._annotate_detected_checkbox.checkbox.Bind(
            event=wx.EVT_CHECKBOX,
            handler=self.on_display_mode_changed)
        self._annotate_rejected_checkbox.checkbox.Bind(
            event=wx.EVT_CHECKBOX,
            handler=self.on_display_mode_changed)
        self._send_capture_parameters_button.Bind(
            event=wx.EVT_BUTTON,
            handler=self.on_send_capture_parameters_pressed)
        self._send_detection_parameters_button.Bind(
            event=wx.EVT_BUTTON,
            handler=self.on_send_detection_parameters_pressed)
        self._calibration_capture_button.Bind(
            event=wx.EVT_BUTTON,
            handler=self.on_calibration_capture_pressed)

        self._update_ui_controls()

    def begin_capture_calibration(self) -> None:
        selected_detector_label: str = self._detector_selector.selector.GetStringSelection()
        request_series: MCTRequestSeries = MCTRequestSeries(series=[CalibrationImageAddRequest()])
        self._control_blocking_request_id = self._controller.request_series_push(
            connection_label=selected_detector_label,
            request_series=request_series)
        self._update_ui_controls()

    def begin_capture_snapshot(self):
        selected_detector_label: str = self._detector_selector.selector.GetStringSelection()
        request_series: MCTRequestSeries = MCTRequestSeries(
            series=[CameraImageGetRequest(format=_CAPTURE_FORMAT)])
        self._live_preview_request_id = self._controller.request_series_push(
            connection_label=selected_detector_label,
            request_series=request_series)

    def begin_get_capture_parameters(self):
        selected_detector_label: str = self._detector_selector.selector.GetStringSelection()
        request_series: MCTRequestSeries = MCTRequestSeries(
            series=[CameraParametersGetRequest()])
        self._control_blocking_request_id = self._controller.request_series_push(
            connection_label=selected_detector_label,
            request_series=request_series)
        self._update_ui_controls()

    def begin_set_capture_parameters(self):
        selected_detector_label: str = self._detector_selector.selector.GetStringSelection()
        key_values: list[KeyValueSimpleAbstract] = list()
        for parameter_ui in self._camera_parameter_uis:
            parameter_type: type[KeyValueSimpleAbstract]
            label: str = parameter_ui.label.GetLabelText()
            if isinstance(parameter_ui, ParameterCheckbox):
                parameter_type = KeyValueSimpleBool
            elif isinstance(parameter_ui, ParameterSelector):
                parameter_type = KeyValueSimpleString
            elif isinstance(parameter_ui, ParameterSpinboxFloat):
                parameter_type = KeyValueSimpleFloat
            elif isinstance(parameter_ui, ParameterSpinboxInteger):
                parameter_type = KeyValueSimpleInt
            else:
                self.status_message_source.enqueue_status_message(
                    severity="error",
                    message=f"Failed to determine parameter type from UI element for key {label}.")
                continue
            key_values.append(parameter_type(
                key=label,
                value=parameter_ui.get_value()))
        request_series: MCTRequestSeries = MCTRequestSeries(
            series=[
                CameraParametersSetRequest(parameters=key_values),
                CameraParametersGetRequest()])  # sync
        self._control_blocking_request_id = self._controller.request_series_push(
            connection_label=selected_detector_label,
            request_series=request_series)
        self._update_ui_controls()

    def begin_set_detection_parameters(self):
        selected_detector_label: str = self._detector_selector.selector.GetStringSelection()
        params: DetectionParameters = DetectionParameters(
            adaptive_thresh_win_size_min=self._detection_param_adaptive_thresh_win_size_min.spinbox.GetValue(),
            adaptive_thresh_win_size_max=self._detection_param_adaptive_thresh_win_size_max.spinbox.GetValue(),
            adaptive_thresh_win_size_step=self._detection_param_adaptive_thresh_win_size_step.spinbox.GetValue(),
            adaptive_thresh_constant=self._detection_param_adaptive_thresh_constant.spinbox.GetValue(),
            min_marker_perimeter_rate=self._detection_param_min_marker_perimeter_rate.spinbox.GetValue(),
            max_marker_perimeter_rate=self._detection_param_max_marker_perimeter_rate.spinbox.GetValue(),
            polygonal_approx_accuracy_rate=self._detection_param_polygonal_approx_accuracy_rate.spinbox.GetValue(),
            min_corner_distance_rate=self._detection_param_min_corner_distance_rate.spinbox.GetValue(),
            min_marker_distance_rate=self._detection_param_min_marker_distance_rate.spinbox.GetValue(),
            min_distance_to_border=self._detection_param_min_distance_to_border.spinbox.GetValue(),
            marker_border_bits=self._detection_param_marker_border_bits.spinbox.GetValue(),
            min_otsu_std_dev=self._detection_param_min_otsu_std_dev.spinbox.GetValue(),
            perspective_remove_pixel_per_cell=self._detection_param_persp_rem_px_per_cell.spinbox.GetValue(),
            perspective_remove_ignored_margin_per_cell=self._detection_param_persp_rem_marg_per_cell.spinbox.GetValue(),
            max_erroneous_bits_in_border_rate=self._detection_param_max_erroneous_bits_border_rate.spinbox.GetValue(),
            error_correction_rate=self._detection_param_error_correction_rate.spinbox.GetValue(),
            detect_inverted_marker=int(self._detection_param_detect_inverted_marker.checkbox.GetValue()),
            corner_refinement_method=self._detection_param_corner_refinement_method.selector.GetStringSelection(),
            corner_refinement_win_size=self._detection_param_corner_refinement_win_size.spinbox.GetValue(),
            corner_refinement_max_iterations=self._detection_param_corner_refinement_max_iterations.spinbox.GetValue(),
            corner_refinement_min_accuracy=self._detection_param_corner_refinement_min_accuracy.spinbox.GetValue(),
            april_tag_critical_rad=self._detection_param_april_tag_critical_rad.spinbox.GetValue(),
            april_tag_deglitch=int(self._detection_param_april_tag_deglitch.checkbox.GetValue()),
            april_tag_max_line_fit_mse=self._detection_param_april_tag_max_line_fit_mse.spinbox.GetValue(),
            april_tag_max_nmaxima=self._detection_param_april_tag_max_nmaxima.spinbox.GetValue(),
            april_tag_min_cluster_pixels=self._detection_param_april_tag_min_cluster_pixels.spinbox.GetValue(),
            april_tag_min_white_black_diff=self._detection_param_april_tag_min_white_black_diff.spinbox.GetValue(),
            april_tag_quad_decimate=self._detection_param_april_tag_quad_decimate.spinbox.GetValue(),
            april_tag_quad_sigma=self._detection_param_april_tag_quad_sigma.spinbox.GetValue(),
            use_aruco_3_detection=self._detection_param_use_aruco_3_detection.checkbox.GetValue(),
            min_marker_length_ratio_original_img=self._detection_param_min_marker_length_ratio_orig.spinbox.GetValue(),
            min_side_length_canonical_img=self._detection_param_min_side_length_canonical_img.spinbox.GetValue())
        request_series: MCTRequestSeries = MCTRequestSeries(series=[
            MarkerParametersSetRequest(parameters=params),
            MarkerParametersGetRequest()])  # sync
        self._control_blocking_request_id = self._controller.request_series_push(
            connection_label=selected_detector_label,
            request_series=request_series)
        self._update_ui_controls()

    def handle_response_series(
        self,
        response_series: MCTResponseSeries,
        task_description: Optional[str] = None,
        expected_response_count: Optional[int] = None
    ) -> None:
        response: MCTResponse
        for response in response_series.series:
            if isinstance(response, CalibrationImageAddResponse):
                self._handle_add_calibration_image_response(response=response)
            elif isinstance(response, CameraImageGetResponse):
                self._handle_capture_snapshot_response(response=response)
            elif isinstance(response, CameraParametersGetResponse):
                self._handle_get_capture_parameters_response(response=response)
            elif isinstance(response, MarkerParametersGetResponse):
                self._handle_get_detection_parameters_response(response=response)
            elif isinstance(response, ErrorResponse):
                self.handle_error_response(response=response)
            elif not isinstance(response, (EmptyResponse, CameraParametersSetResponse)):
                self.handle_unknown_response(response=response)

    def _handle_add_calibration_image_response(
        self,
        response: CalibrationImageAddResponse
    ):
        self.status_message_source.enqueue_status_message(
            severity="info",
            message=f"Added image {response.image_identifier}.")

    def _handle_capture_snapshot_response(
        self,
        response: CameraImageGetResponse
    ):
        if self._preview_image_checkbox.checkbox.GetValue():
            self._live_image_base64 = response.image_base64

    # noinspection DuplicatedCode
    def _handle_get_capture_parameters_response(
        self,
        response: CameraParametersGetResponse
    ):
        self._camera_parameter_uis.clear()
        self._camera_parameter_panel.Freeze()
        self._camera_parameter_sizer.Clear(True)
        self._camera_parameter_sizer = wx.BoxSizer(orient=wx.VERTICAL)
        key_value: KeyValueMetaAbstract
        for key_value in response.parameters:
            if isinstance(key_value, KeyValueMetaBool):
                self._camera_parameter_uis.append(self.add_control_checkbox(
                    parent=self._camera_parameter_panel,
                    sizer=self._camera_parameter_sizer,
                    label=key_value.key,
                    value=key_value.value))
            elif isinstance(key_value, KeyValueMetaEnum):
                self._camera_parameter_uis.append(self.add_control_selector(
                    parent=self._camera_parameter_panel,
                    sizer=self._camera_parameter_sizer,
                    label=key_value.key,
                    selectable_values=key_value.allowable_values,
                    value=key_value.value))
            elif isinstance(key_value, KeyValueMetaFloat):
                self._camera_parameter_uis.append(self.add_control_spinbox_float(
                    parent=self._camera_parameter_panel,
                    sizer=self._camera_parameter_sizer,
                    label=key_value.key,
                    minimum_value=key_value.range_minimum,
                    maximum_value=key_value.range_maximum,
                    initial_value=key_value.value,
                    step_value=key_value.range_step,
                    digit_count=key_value.digit_count))
            elif isinstance(key_value, KeyValueMetaInt):
                self._camera_parameter_uis.append(self.add_control_spinbox_integer(
                    parent=self._camera_parameter_panel,
                    sizer=self._camera_parameter_sizer,
                    label=key_value.key,
                    minimum_value=key_value.range_minimum,
                    maximum_value=key_value.range_maximum,
                    initial_value=key_value.value,
                    step_value=key_value.range_step))
            else:
                self.status_message_source.enqueue_status_message(
                    severity="error",
                    message=f"Unsupported parameter type {key_value.parsable_type} will not be handled")
        self._camera_parameter_panel.SetSizer(self._camera_parameter_sizer)
        self._camera_parameter_panel.Thaw()
        self.Layout()

    # noinspection DuplicatedCode
    def _handle_get_detection_parameters_response(
        self,
        response: MarkerParametersGetResponse
    ):
        params: DetectionParameters = response.parameters
        if params.adaptive_thresh_win_size_min is not None:
            self._detection_param_adaptive_thresh_win_size_min.spinbox.SetValue(
                params.adaptive_thresh_win_size_min)
        if params.adaptive_thresh_win_size_max is not None:
            self._detection_param_adaptive_thresh_win_size_max.spinbox.SetValue(
                params.adaptive_thresh_win_size_max)
        if params.adaptive_thresh_win_size_step is not None:
            self._detection_param_adaptive_thresh_win_size_step.spinbox.SetValue(
                params.adaptive_thresh_win_size_step)
        if params.adaptive_thresh_constant is not None:
            self._detection_param_adaptive_thresh_constant.spinbox.SetValue(params.adaptive_thresh_constant)
        if params.min_marker_perimeter_rate is not None:
            self._detection_param_min_marker_perimeter_rate.spinbox.SetValue(params.min_marker_perimeter_rate)
        if params.max_marker_perimeter_rate is not None:
            self._detection_param_max_marker_perimeter_rate.spinbox.SetValue(params.max_marker_perimeter_rate)
        if params.polygonal_approx_accuracy_rate is not None:
            self._detection_param_polygonal_approx_accuracy_rate.spinbox.SetValue(
                params.polygonal_approx_accuracy_rate)
        if params.min_corner_distance_rate is not None:
            self._detection_param_min_corner_distance_rate.spinbox.SetValue(params.min_corner_distance_rate)
        if params.min_marker_distance_rate is not None:
            self._detection_param_min_marker_distance_rate.spinbox.SetValue(params.min_marker_distance_rate)
        if params.min_distance_to_border is not None:
            self._detection_param_min_distance_to_border.spinbox.SetValue(params.min_distance_to_border)
        if params.marker_border_bits is not None:
            self._detection_param_marker_border_bits.spinbox.SetValue(params.marker_border_bits)
        if params.min_otsu_std_dev is not None:
            self._detection_param_min_otsu_std_dev.spinbox.SetValue(params.min_otsu_std_dev)
        if params.perspective_remove_pixel_per_cell is not None:
            self._detection_param_persp_rem_px_per_cell.spinbox.SetValue(
                params.perspective_remove_pixel_per_cell)
        if params.perspective_remove_ignored_margin_per_cell is not None:
            self._detection_param_persp_rem_marg_per_cell.spinbox.SetValue(
                params.perspective_remove_ignored_margin_per_cell)
        if params.max_erroneous_bits_in_border_rate is not None:
            self._detection_param_max_erroneous_bits_border_rate.spinbox.SetValue(
                params.max_erroneous_bits_in_border_rate)
        if params.error_correction_rate is not None:
            self._detection_param_error_correction_rate.spinbox.SetValue(params.error_correction_rate)
        if params.detect_inverted_marker is not None:
            self._detection_param_detect_inverted_marker.checkbox.SetValue(bool(params.detect_inverted_marker))
        if params.corner_refinement_method is not None:
            self._detection_param_corner_refinement_method.selector.SetStringSelection(
                params.corner_refinement_method)
        if params.corner_refinement_win_size is not None:
            self._detection_param_corner_refinement_win_size.spinbox.SetValue(params.corner_refinement_win_size)
        if params.corner_refinement_max_iterations is not None:
            self._detection_param_corner_refinement_max_iterations.spinbox.SetValue(
                params.corner_refinement_max_iterations)
        if params.corner_refinement_min_accuracy is not None:
            self._detection_param_corner_refinement_min_accuracy.spinbox.SetValue(
                params.corner_refinement_min_accuracy)
        if params.april_tag_critical_rad is not None:
            self._detection_param_april_tag_critical_rad.spinbox.SetValue(params.april_tag_critical_rad)
        if params.april_tag_deglitch is not None:
            self._detection_param_april_tag_deglitch.checkbox.SetValue(bool(params.april_tag_deglitch))
        if params.april_tag_max_line_fit_mse is not None:
            self._detection_param_april_tag_max_line_fit_mse.spinbox.SetValue(params.april_tag_max_line_fit_mse)
        if params.april_tag_max_nmaxima is not None:
            self._detection_param_april_tag_max_nmaxima.spinbox.SetValue(params.april_tag_max_nmaxima)
        if params.april_tag_min_cluster_pixels is not None:
            self._detection_param_april_tag_min_cluster_pixels.spinbox.SetValue(
                params.april_tag_min_cluster_pixels)
        if params.april_tag_min_white_black_diff is not None:
            self._detection_param_april_tag_min_white_black_diff.spinbox.SetValue(
                params.april_tag_min_white_black_diff)
        if params.april_tag_quad_decimate is not None:
            self._detection_param_april_tag_quad_decimate.spinbox.SetValue(params.april_tag_quad_decimate)
        if params.april_tag_quad_sigma is not None:
            self._detection_param_april_tag_quad_sigma.spinbox.SetValue(params.april_tag_quad_sigma)
        if params.use_aruco_3_detection is not None:
            self._detection_param_use_aruco_3_detection.checkbox.SetValue(params.use_aruco_3_detection)
        if params.min_marker_length_ratio_original_img is not None:
            self._detection_param_min_marker_length_ratio_orig.spinbox.SetValue(
                params.min_marker_length_ratio_original_img)
        if params.min_side_length_canonical_img is not None:
            self._detection_param_min_side_length_canonical_img.spinbox.SetValue(
                params.min_side_length_canonical_img)

    @staticmethod
    def _marker_snapshot_list_to_opencv_points(
        marker_snapshot_list: list[MarkerSnapshot],
        scale: float
    ) -> numpy.ndarray:
        corners: list[list[list[(float, float)]]] = [[[
            (corner_point.x_px * scale, corner_point.y_px * scale)
            for corner_point in marker.corner_image_points
        ]] for marker in marker_snapshot_list]
        return_value = numpy.array(corners, dtype=numpy.int32)
        return return_value

    def on_calibration_capture_pressed(self, _event: wx.CommandEvent):
        self.begin_capture_calibration()

    def on_detector_selected(self, _event: wx.CommandEvent):
        self._live_image_base64 = None
        self.begin_get_capture_parameters()
        self._update_ui_image()
        self._update_ui_controls()

    def on_display_mode_changed(self, _event: wx.CommandEvent):
        if not self._preview_image_checkbox.checkbox.GetValue():
            self._live_image_base64 = None
        self._update_ui_image()

    def on_page_select(self):
        super().on_page_select()
        available_detector_labels: list[str] = self._controller.get_active_detector_labels()
        self._detector_selector.set_options(option_list=available_detector_labels)
        self._update_ui_controls()

    def on_send_capture_parameters_pressed(self, _event: wx.CommandEvent):
        self.begin_set_capture_parameters()

    def on_send_detection_parameters_pressed(self, _event: wx.CommandEvent):
        self.begin_set_detection_parameters()

    def _set_display_controls_enabled(
        self,
        enable: bool
    ):
        self._preview_image_checkbox.Enable(enable=enable)
        self._annotate_detected_checkbox.Enable(enable=enable)
        self._annotate_rejected_checkbox.Enable(enable=enable)

    def _set_parameter_controls_enabled(
        self,
        enable: bool
    ):
        for parameter_ui in self._camera_parameter_uis:
            parameter_ui.set_enabled(enable=enable)
        self._detection_param_adaptive_thresh_win_size_min.set_enabled(enable=enable)
        self._detection_param_adaptive_thresh_win_size_max.set_enabled(enable=enable)
        self._detection_param_adaptive_thresh_win_size_step.set_enabled(enable=enable)
        self._detection_param_adaptive_thresh_constant.set_enabled(enable=enable)
        self._detection_param_min_marker_perimeter_rate.set_enabled(enable=enable)
        self._detection_param_max_marker_perimeter_rate.set_enabled(enable=enable)
        self._detection_param_polygonal_approx_accuracy_rate.set_enabled(enable=enable)
        self._detection_param_min_corner_distance_rate.set_enabled(enable=enable)
        self._detection_param_min_marker_distance_rate.set_enabled(enable=enable)
        self._detection_param_min_distance_to_border.set_enabled(enable=enable)
        self._detection_param_marker_border_bits.set_enabled(enable=enable)
        self._detection_param_min_otsu_std_dev.set_enabled(enable=enable)
        self._detection_param_persp_rem_px_per_cell.set_enabled(enable=enable)
        self._detection_param_persp_rem_marg_per_cell.set_enabled(enable=enable)
        self._detection_param_max_erroneous_bits_border_rate.set_enabled(enable=enable)
        self._detection_param_error_correction_rate.set_enabled(enable=enable)
        self._detection_param_detect_inverted_marker.set_enabled(enable=enable)
        self._detection_param_corner_refinement_method.set_enabled(enable=enable)
        self._detection_param_corner_refinement_win_size.set_enabled(enable=enable)
        self._detection_param_corner_refinement_max_iterations.set_enabled(enable=enable)
        self._detection_param_corner_refinement_min_accuracy.set_enabled(enable=enable)
        self._detection_param_april_tag_critical_rad.set_enabled(enable=enable)
        self._detection_param_april_tag_deglitch.set_enabled(enable=enable)
        self._detection_param_april_tag_max_line_fit_mse.set_enabled(enable=enable)
        self._detection_param_april_tag_max_nmaxima.set_enabled(enable=enable)
        self._detection_param_april_tag_min_cluster_pixels.set_enabled(enable=enable)
        self._detection_param_april_tag_min_white_black_diff.set_enabled(enable=enable)
        self._detection_param_april_tag_quad_decimate.set_enabled(enable=enable)
        self._detection_param_april_tag_quad_sigma.set_enabled(enable=enable)
        self._detection_param_use_aruco_3_detection.set_enabled(enable=enable)
        self._detection_param_min_side_length_canonical_img.set_enabled(enable=enable)
        self._detection_param_min_marker_length_ratio_orig.set_enabled(enable=enable)
        self._send_capture_parameters_button.Enable(enable=enable)
        self._send_detection_parameters_button.Enable(enable=enable)

    def update_loop(self):
        super().update_loop()

        response_series: MCTResponseSeries | None
        if self._control_blocking_request_id is not None:
            self._control_blocking_request_id, response_series = self._controller.response_series_pop(
                request_series_id=self._control_blocking_request_id)
            if response_series is not None:  # self._control_blocking_request_id will be None
                self.handle_response_series(response_series)
                self._update_ui_controls()
        elif self._live_preview_request_id is not None:
            self._live_preview_request_id, response_series = self._controller.response_series_pop(
                request_series_id=self._live_preview_request_id)
            if response_series is not None:
                self.handle_response_series(response_series)

        detector_label: str = self._detector_selector.selector.GetStringSelection()
        if detector_label is not None and len(detector_label) > 0:
            if self._preview_image_checkbox.checkbox.GetValue() and self._live_preview_request_id is None:
                self.begin_capture_snapshot()
            detector_frame: DetectorFrame | None = self._controller.get_live_detector_frame(
                detector_label=detector_label)
            if detector_frame is not None:
                self._live_markers_detected = detector_frame.detected_marker_snapshots
                self._live_markers_rejected = detector_frame.rejected_marker_snapshots

        if self._preview_image_checkbox.checkbox.GetValue() or \
           self._annotate_detected_checkbox.checkbox.GetValue() or \
           self._annotate_rejected_checkbox.checkbox.GetValue():
            self._update_ui_image()

    def _update_ui_controls(self):
        self._detector_selector.set_enabled(enable=False)
        self._set_display_controls_enabled(enable=False)
        self._set_parameter_controls_enabled(enable=False)
        self._calibration_capture_button.Enable(enable=False)
        if not self._controller.is_running():
            return
        self._detector_selector.set_enabled(enable=True)
        selected_detector_label: str = self._detector_selector.selector.GetStringSelection()
        if selected_detector_label is None or len(selected_detector_label) <= 0:
            return
        if self._control_blocking_request_id is not None:
            return
        self._set_display_controls_enabled(enable=True)
        self._set_parameter_controls_enabled(enable=True)
        self._calibration_capture_button.Enable(enable=True)

    def _update_ui_image(self):
        # TODO: The Detector should tell us the resolution of the image it operated on.
        resolution_str: str = str(StandardResolutions.RES_640X480)
        display_image: numpy.ndarray
        scale: float | None
        if self._live_image_base64 is not None:
            opencv_image: numpy.ndarray = ImageCoding.base64_to_image(input_base64=self._live_image_base64)
            display_image: numpy.ndarray = ImageUtils.image_resize_to_fit(
                opencv_image=opencv_image,
                available_size=self._image_panel.GetSize())
            scale: float = display_image.shape[0] / opencv_image.shape[0]  # note: opencv idx 0 corresponds to height
        elif resolution_str is not None and len(resolution_str) > 0:
            panel_size_px: tuple[int, int] = self._image_panel.GetSize()
            image_resolution: ImageResolution = ImageResolution.from_str(in_str=resolution_str)
            rescaled_resolution_px: tuple[int, int] = ImageUtils.scale_factor_for_available_space_px(
                source_resolution_px=(image_resolution.x_px, image_resolution.y_px),
                available_size_px=panel_size_px)
            display_image = ImageUtils.black_image(resolution_px=rescaled_resolution_px)
            scale: float = rescaled_resolution_px[1] / image_resolution.y_px
        else:
            display_image = ImageUtils.black_image(resolution_px=self._image_panel.GetSize())
            scale = None  # not available

        if scale is not None:
            if self._annotate_detected_checkbox.checkbox.GetValue():
                corners: numpy.ndarray = self._marker_snapshot_list_to_opencv_points(
                    marker_snapshot_list=self._live_markers_detected,
                    scale=scale)
                cv2.polylines(
                    img=display_image,
                    pts=corners,
                    isClosed=True,
                    color=[255, 191, 127],  # blue
                    thickness=2)
            if self._annotate_rejected_checkbox.checkbox.GetValue():
                corners: numpy.ndarray = self._marker_snapshot_list_to_opencv_points(
                    marker_snapshot_list=self._live_markers_rejected,
                    scale=scale)
                cv2.polylines(
                    img=display_image,
                    pts=corners,
                    isClosed=True,
                    color=[127, 191, 255],  # orange
                    thickness=2)

        image_buffer: bytes = ImageCoding.image_to_bytes(image_data=display_image, image_format=".jpg")
        image_buffer_io: BytesIO = BytesIO(image_buffer)
        wx_image: wx.Image = wx.Image(image_buffer_io)
        wx_bitmap: wx.Bitmap = wx_image.ConvertToBitmap()
        self._image_panel.set_bitmap(wx_bitmap)
        self._image_panel.paint()
