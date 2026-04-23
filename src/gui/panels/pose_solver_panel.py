from .base_panel import \
    BasePanel
from .parameters import \
    ParameterSelector
from .specialized import \
    GraphicsRenderer, \
    TargetTable, \
    TrackingTableRow
from src.common import \
    Matrix4x4, \
    StatusMessageSource
from src.controller import \
    MCTController
import logging
import platform
from typing import Final
import wx
import wx.grid


_CONTROL_MIN_WIDTH_PX: Final[int] = 760


logger = logging.getLogger(__name__)

POSE_REPRESENTATIVE_MODEL: Final[str] = "coordinate_axes"


class PoseSolverPanel(BasePanel):

    _controller: MCTController
    _status_message_source: StatusMessageSource

    _mixer_selector: ParameterSelector
    _target_table: TargetTable

    _awaiting_user_task: bool
    _in_update: bool
    _selected_target_label: str | None

    def __init__(
        self,
        parent: wx.Window,
        controller: MCTController,
        name: str = "PoseSolverPanel"
    ):
        super().__init__(
            parent=parent,
            name=name)
        self._controller = controller
        self._status_message_source = controller.get_status_message_source()

        self._awaiting_user_task = False
        self._in_update = False
        self._selected_target_label = None

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
        control_border_panel.SetMinSize(size=wx.Size(_CONTROL_MIN_WIDTH_PX, 0))
        control_panel.ShowScrollbars(
            horz=wx.SHOW_SB_NEVER,
            vert=wx.SHOW_SB_ALWAYS)

        control_sizer: wx.BoxSizer = wx.BoxSizer(orient=wx.VERTICAL)

        self._mixer_selector = self.add_control_selector(
            parent=control_panel,
            sizer=control_sizer,
            label="Mixer",
            selectable_values=list())

        self.add_horizontal_line_to_spacer(
            parent=control_panel,
            sizer=control_sizer)

        self._target_table = TargetTable(parent=control_panel)
        control_sizer.Add(
            window=self._target_table,
            flags=wx.SizerFlags(0).Expand())
        control_sizer.AddSpacer(size=BasePanel.DEFAULT_SPACING_PX_VERTICAL)

        self._target_display_textbox = wx.TextCtrl(
            parent=control_panel,
            style=wx.TE_MULTILINE | wx.TE_READONLY | wx.TE_RICH)
        self._target_display_textbox.SetEditable(False)
        self._target_display_textbox.SetBackgroundColour(colour=wx.Colour(red=249, green=249, blue=249, alpha=255))
        control_sizer.Add(
            window=self._target_display_textbox,
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

        if platform.system() == "Linux":
            logger.warning("OpenGL context creation does not currently work well in Linux. Rendering is disabled.")
            self._renderer = None
            horizontal_split_sizer.AddStretchSpacer()
            horizontal_split_sizer.Add(
                window=control_border_panel,
                flags=wx.SizerFlags(1).Expand())
            horizontal_split_sizer.AddStretchSpacer()
        else:
            self._renderer = GraphicsRenderer(parent=self)
            horizontal_split_sizer.Add(
                window=control_border_panel,
                flags=wx.SizerFlags(35).Expand())
            horizontal_split_sizer.Add(
                window=self._renderer,
                flags=wx.SizerFlags(65).Expand())
            self._renderer.load_models_into_context_from_data_path()
            self._renderer.add_scene_object("coordinate_axes", Matrix4x4())

        self.SetSizerAndFit(sizer=horizontal_split_sizer)

        self._mixer_selector.selector.Bind(
            event=wx.EVT_CHOICE,
            handler=self._on_ui_mixer_selected)
        self._target_table.table.Bind(
            event=wx.grid.EVT_GRID_SELECT_CELL,
            handler=self._on_ui_target_row_selected)

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

    def _on_ui_mixer_selected(self, _event: wx.CommandEvent) -> None:
        self._update_ui_controls()

    def _on_ui_target_row_selected(self, _event: wx.grid.GridEvent) -> None:
        if self._in_update:
            return
        self._selected_target_label = self._target_table.get_selected_target_label()

    def update_loop(self) -> None:
        super().update_loop()

        self._in_update = True

        if self._renderer is not None:
            self._renderer.clear_scene_objects()

        table_rows: list[TrackingTableRow] = list()
        selected_mixer_label: str = self._mixer_selector.selector.GetStringSelection()
        if selected_mixer_label is not None and len(selected_mixer_label) > 0:
            live_mixer_frame: MCTController.MixerLiveData = \
                self._controller.get_live_mixer_data(mixer_label=selected_mixer_label)
            for pose in live_mixer_frame.frame.target_poses:
                table_row: TrackingTableRow = TrackingTableRow(
                    target_id=pose.target_id,
                    label=pose.target_id,
                    x=pose.object_to_reference_matrix[0, 3],
                    y=pose.object_to_reference_matrix[1, 3],
                    z=pose.object_to_reference_matrix[2, 3])
                table_rows.append(table_row)
                if self._renderer is not None:
                    self._renderer.add_scene_object(
                        model_key=POSE_REPRESENTATIVE_MODEL,
                        transform_to_world=pose.object_to_reference_matrix)
            for pose in live_mixer_frame.frame.detector_poses:
                table_row: TrackingTableRow = TrackingTableRow(
                    target_id=pose.target_id,
                    label=pose.target_id,
                    x=pose.object_to_reference_matrix[0, 3],
                    y=pose.object_to_reference_matrix[1, 3],
                    z=pose.object_to_reference_matrix[2, 3])
                table_rows.append(table_row)
                if self._renderer is not None:
                    self._renderer.add_scene_object(
                        model_key=POSE_REPRESENTATIVE_MODEL,
                        transform_to_world=pose.object_to_reference_matrix)

        self._target_table.update_contents(row_contents=table_rows)
        if len(table_rows) > 0:
            self._target_table.Enable(True)
        else:
            self._target_table.Enable(False)

        if self._renderer is not None:
            self._renderer.render()

        self._in_update = False

    def _update_ui_controls(self) -> None:
        self._mixer_selector.Enable(False)
        self._target_table.Enable(False)
        self._target_display_textbox.Enable(False)
        if self._awaiting_user_task:
            return  # We're waiting for something
        self._mixer_selector.Enable(True)
        selected_mixer_label: str = self._mixer_selector.selector.GetStringSelection()
        if selected_mixer_label is None or len(selected_mixer_label) <= 0:
            return
        self._target_table.Enable(True)
        tracked_target_index: int = self._target_table.get_selected_row_index()
        if tracked_target_index is not None:
            self._target_display_textbox.Enable(True)
