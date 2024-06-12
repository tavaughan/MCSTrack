from .base_panel import \
    BasePanel
from .parameters import \
    ParameterSelector, \
    ParameterSpinboxFloat, \
    ParameterSpinboxInteger
from .specialized import \
    GraphicsRenderer, \
    TrackingTable, \
    TrackingTableRow
from src.calibrator.api import \
    GetCalibrationResultRequest, \
    GetCalibrationResultResponse, \
    ListCalibrationDetectorResolutionsRequest, \
    ListCalibrationDetectorResolutionsResponse, \
    ListCalibrationResultMetadataRequest, \
    ListCalibrationResultMetadataResponse
from src.common import \
    ErrorResponse, \
    EmptyResponse, \
    MCastRequest, \
    MCastRequestSeries, \
    MCastResponse, \
    MCastResponseSeries, \
    StatusMessageSource
from src.common.structures import \
    DetectorResolution, \
    ImageResolution, \
    IntrinsicParameters, \
    MarkerSnapshot, \
    Matrix4x4, \
    Pose
from src.connector import \
    Connector
from src.detector.api import \
    GetCapturePropertiesRequest, \
    GetCapturePropertiesResponse, \
    GetMarkerSnapshotsRequest, \
    GetMarkerSnapshotsResponse, \
    StartCaptureRequest, \
    StopCaptureRequest
from src.pose_solver.api import \
    AddTargetMarkerRequest, \
    AddTargetMarkerResponse, \
    AddMarkerCornersRequest, \
    GetPosesRequest, \
    GetPosesResponse, \
    SetIntrinsicParametersRequest, \
    SetReferenceMarkerRequest, \
    StartPoseSolverRequest, \
    StopPoseSolverRequest
import datetime
import logging
from typing import Final, Optional
import uuid
import wx
import wx.grid


logger = logging.getLogger(__name__)

from src.connector.connector import ACTIVE_PHASE_IDLE, ACTIVE_PHASE_STARTING_CAPTURE, ACTIVE_PHASE_STARTING_FINAL, ACTIVE_PHASE_STARTING_GET_INTRINSICS, ACTIVE_PHASE_STARTING_GET_RESOLUTIONS, ACTIVE_PHASE_STARTING_LIST_INTRINSICS, ACTIVE_PHASE_STOPPING

POSE_REPRESENTATIVE_MODEL: Final[str] = "coordinate_axes"

# TODO: There is a lot of general logic that probably best be moved to the Connector class,
#       thereby allowing the pose solving functionality to be used without the UI.
class PoseSolverPanel(BasePanel):

    _connector: Connector

    _pose_solver_selector: ParameterSelector
    _reference_marker_id_spinbox: ParameterSpinboxInteger
    _reference_marker_diameter_spinbox: ParameterSpinboxFloat
    _reference_target_submit_button: wx.Button
    _tracked_marker_id_spinbox: ParameterSpinboxInteger
    _tracked_marker_diameter_spinbox: ParameterSpinboxFloat
    _tracked_target_submit_button: wx.Button
    _tracking_start_button: wx.Button
    _tracking_stop_button: wx.Button
    _tracking_table: TrackingTable

    _is_updating: bool
    _target_id_to_label: dict[str, str]

    def __init__(
        self,
        parent: wx.Window,
        connector: Connector,
        status_message_source: StatusMessageSource,
        name: str = "PoseSolverPanel"
    ):
        super().__init__(
            parent=parent,
            connector=connector,
            status_message_source=status_message_source,
            name=name)
        self._connector = connector

        self._passive_detecting_requests = dict()

        self._is_updating = False
        self._target_id_to_label = dict()

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

        self._pose_solver_selector = self.add_control_selector(
            parent=control_panel,
            sizer=control_sizer,
            label="Pose Solver",
            selectable_values=list())

        self.add_horizontal_line_to_spacer(
            parent=control_panel,
            sizer=control_sizer)

        self._reference_marker_id_spinbox = self.add_control_spinbox_integer(
            parent=control_panel,
            sizer=control_sizer,
            label="Reference Marker ID",
            minimum_value=0,
            maximum_value=99,
            initial_value=0)

        self._reference_marker_diameter_spinbox: ParameterSpinboxFloat = self.add_control_spinbox_float(
            parent=control_panel,
            sizer=control_sizer,
            label="Marker diameter (mm)",
            minimum_value=1.0,
            maximum_value=1000.0,
            initial_value=10.0,
            step_value=0.5)

        self._reference_target_submit_button: wx.Button = self.add_control_button(
            parent=control_panel,
            sizer=control_sizer,
            label="Set Reference Marker")

        self._tracked_marker_id_spinbox = self.add_control_spinbox_integer(
            parent=control_panel,
            sizer=control_sizer,
            label="Tracked Marker ID",
            minimum_value=0,
            maximum_value=99,
            initial_value=1)

        self._tracked_marker_diameter_spinbox: ParameterSpinboxFloat = self.add_control_spinbox_float(
            parent=control_panel,
            sizer=control_sizer,
            label="Marker diameter (mm)",
            minimum_value=1.0,
            maximum_value=1000.0,
            initial_value=10.0,
            step_value=0.5)

        self._tracked_target_submit_button: wx.Button = self.add_control_button(
            parent=control_panel,
            sizer=control_sizer,
            label="Add Tracked Marker")

        self.add_horizontal_line_to_spacer(
            parent=control_panel,
            sizer=control_sizer)

        self._tracking_start_button: wx.Button = self.add_control_button(
            parent=control_panel,
            sizer=control_sizer,
            label="Start Tracking")

        self._tracking_stop_button: wx.Button = self.add_control_button(
            parent=control_panel,
            sizer=control_sizer,
            label="Stop Tracking")

        self.add_horizontal_line_to_spacer(
            parent=control_panel,
            sizer=control_sizer)

        self._tracking_table = TrackingTable(parent=control_panel)
        control_sizer.Add(
            window=self._tracking_table,
            flags=wx.SizerFlags(0).Expand())
        control_sizer.AddSpacer(size=BasePanel.DEFAULT_SPACING_PX_VERTICAL)

        self._tracking_display_textbox = wx.TextCtrl(
            parent=control_panel,
            style=wx.TE_MULTILINE | wx.TE_READONLY | wx.TE_RICH)
        self._tracking_display_textbox.SetEditable(False)
        self._tracking_display_textbox.SetBackgroundColour(colour=wx.Colour(red=249, green=249, blue=249, alpha=255))
        control_sizer.Add(
            window=self._tracking_display_textbox,
            flags=wx.SizerFlags(1).Align(wx.EXPAND))

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

        # self._renderer = GraphicsRenderer(parent=self)
        # self._renderer.load_models_into_context_from_data_path()
        # self._renderer.add_scene_object("coordinate_axes", Matrix4x4())
        # horizontal_split_sizer.Add(
        #     window=self._renderer,
        #     flags=wx.SizerFlags(65).Expand())

        self.SetSizerAndFit(sizer=horizontal_split_sizer)

        self._pose_solver_selector.selector.Bind(
            event=wx.EVT_CHOICE,
            handler=self.on_pose_solver_select)
        self._reference_target_submit_button.Bind(
            event=wx.EVT_BUTTON,
            handler=self.on_reference_target_submit_pressed)
        self._tracked_target_submit_button.Bind(
            event=wx.EVT_BUTTON,
            handler=self.on_tracked_target_submit_pressed)
        self._tracking_start_button.Bind(
            event=wx.EVT_BUTTON,
            handler=self.on_tracking_start_pressed)
        self._tracking_stop_button.Bind(
            event=wx.EVT_BUTTON,
            handler=self.on_tracking_stop_pressed)
        self._tracking_table.table.Bind(
            event=wx.grid.EVT_GRID_SELECT_CELL,
            handler=self.on_tracking_row_selected)
        

    # The following two functions are now part of the connector class
    # and thus only called from within the connector
    # But when these functions are called, the pose solver panel needs to update
    # I'm not sure how exactly to approach this so I've just commented for now

    # def handle_response_get_poses(
    #     self,
    #     response: GetPosesResponse
    # ) -> None:
    #     
    #     # self._renderer.clear_scene_objects()
    #     # self._renderer.add_scene_object(  # Reference
    #     #     model_key=POSE_REPRESENTATIVE_MODEL,
    #     #     transform_to_world=Matrix4x4())
    #     table_rows: list[TrackingTableRow] = list()
    #     for pose in response.target_poses:
    #         label: str = str()
    #         if pose.target_id in self._target_id_to_label:
    #             label = self._target_id_to_label[pose.target_id]
    #         table_row: TrackingTableRow = TrackingTableRow(
    #             target_id=pose.target_id,
    #             label=label,
    #             x=pose.object_to_reference_matrix[0, 3],
    #             y=pose.object_to_reference_matrix[1, 3],
    #             z=pose.object_to_reference_matrix[2, 3])
    #         table_rows.append(table_row)
    #         # self._renderer.add_scene_object(
    #         #     model_key=POSE_REPRESENTATIVE_MODEL,
    #         #     transform_to_world=pose.object_to_reference_matrix)
    #     for pose in response.detector_poses:
    #         table_row: TrackingTableRow = TrackingTableRow(
    #             target_id=pose.target_id,
    #             label=pose.target_id,
    #             x=pose.object_to_reference_matrix[0, 3],
    #             y=pose.object_to_reference_matrix[1, 3],
    #             z=pose.object_to_reference_matrix[2, 3])
    #         table_rows.append(table_row)
    #         # self._renderer.add_scene_object(
    #         #     model_key=POSE_REPRESENTATIVE_MODEL,
    #         #     transform_to_world=pose.object_to_reference_matrix)
    #     self._tracking_table.update_contents(row_contents=table_rows)
    #     if len(table_rows) > 0:
    #         self._tracking_table.Enable(True)
    #     else:
    #         self._tracking_table.Enable(False)

    # def on_active_request_ids_processed(self) -> None:
    #     # TODO: change to getter function?
    #     if self._connector.current_phase == ACTIVE_PHASE_STOPPING:
    #         self._tracking_table.update_contents(list())
    #         self._tracking_table.Enable(False)
    #         self._tracking_display_textbox.SetValue(str())
    #         self._tracking_display_textbox.Enable(False)

    def on_page_select(self) -> None:
        super().on_page_select()
        selected_pose_solver_label: str = self._pose_solver_selector.selector.GetStringSelection()
        available_pose_solver_labels: list[str] = self._connector.get_connected_pose_solver_labels()
        self._pose_solver_selector.set_options(option_list=available_pose_solver_labels)
        if selected_pose_solver_label in available_pose_solver_labels:
            self._pose_solver_selector.selector.SetStringSelection(selected_pose_solver_label)
        else:
            self._pose_solver_selector.selector.SetStringSelection(str())
        self._connector.selected_pose_solver_label = selected_pose_solver_label
        self._update_controls()

    def on_pose_solver_select(self, _event: wx.CommandEvent) -> None:
        self._update_controls()

    def on_reference_target_submit_pressed(self, _event: wx.CommandEvent) -> None:
        marker_id=self._reference_marker_id_spinbox.spinbox.GetValue()
        marker_diameter=self._reference_marker_diameter_spinbox.spinbox.GetValue()
        self._connector.set_reference_target(
            marker_id=marker_id,
            marker_diameter=marker_diameter)
        self._update_controls()

    def on_tracked_target_submit_pressed(self, _event: wx.CommandEvent) -> None:
        marker_id=self._tracked_marker_id_spinbox.spinbox.GetValue()
        marker_diameter=self._tracked_marker_diameter_spinbox.spinbox.GetValue()
        self._connector.set_tracked_target(
            marker_id=marker_id,
            marker_diameter=marker_diameter)
        self._update_controls()

    def on_tracking_row_selected(self, _event: wx.grid.GridEvent) -> None:
        if self._is_updating:
            return
        selected_index: int | None = self._tracking_table.get_selected_row_index()
        if selected_index is not None:
            if 0 <= selected_index < len(self._connector.tracked_target_poses):
                display_text: str = self._connector.tracked_target_poses[selected_index].json(indent=4)
                self._tracking_display_textbox.SetValue(display_text)
            else:
                self.status_message_source.enqueue_status_message(
                    severity="error",
                    message=f"Target index {selected_index} is out of bounds. Selection will be set to None.")
                self._tracking_table.set_selected_row_index(None)
        self._update_controls()

    def on_tracking_start_pressed(self, _event: wx.CommandEvent) -> None:
        self._connector.start_tracking()
        self._update_controls()

    def on_tracking_stop_pressed(self, _event: wx.CommandEvent) -> None:
        self._connector.stop_tracking()
        self._update_controls()

    def update_loop(self) -> None:

        super().update_loop()
        # self._renderer.render()
        
        self._is_updating = True
        if self._connector.update_loop():
            self._update_controls()

        self._is_updating = False

    def _update_controls(self) -> None:
        self._pose_solver_selector.Enable(False)
        self._reference_marker_id_spinbox.Enable(False)
        self._reference_target_submit_button.Enable(False)
        self._tracked_marker_id_spinbox.Enable(False)
        self._tracked_target_submit_button.Enable(False)
        self._tracking_start_button.Enable(False)
        self._tracking_stop_button.Enable(False)
        self._tracking_table.Enable(False)
        self._tracking_display_textbox.Enable(False)
        if len(self._connector.active_request_ids) > 0:
            return  # We're waiting for something
        self._pose_solver_selector.Enable(True)
        self._reference_marker_id_spinbox.Enable(True)
        self._reference_target_submit_button.Enable(True)
        self._tracked_marker_id_spinbox.Enable(True)
        self._tracked_target_submit_button.Enable(True)
        if not self._connector.is_solving:
            self._tracking_start_button.Enable(True)
        else:
            self._tracking_stop_button.Enable(True)
        if len(self._connector.tracked_target_poses) > 0:
            self._tracking_table.Enable(True)
            tracked_target_index: int = self._tracking_table.get_selected_row_index()
            if tracked_target_index is not None:
                if tracked_target_index >= len(self._connector.tracked_target_poses):
                    self.status_message_source.enqueue_status_message(
                        severity="warning",
                        message=f"Selected tracked target index {tracked_target_index} is out of bounds. Setting to None.")
                    self._tracking_table.set_selected_row_index(None)
                else:
                    self._tracking_display_textbox.Enable(True)

