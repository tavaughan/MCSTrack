from .base_panel import BasePanel
from .specialized import \
    ConnectionTable
from src.controller import \
    Connection, \
    MCTController
from typing import Final
import wx
import wx.grid


_CONTROL_MIN_WIDTH_PX: Final[int] = 760


class ControllerPanel(BasePanel):

    _controller: MCTController
    _load_configuration_button: wx.Button
    _start_button: wx.Button
    _stop_button: wx.Button
    _connection_table: ConnectionTable
    _controller_status_textbox: wx.TextCtrl

    _controller_state: str  # last status reported by MCTController
    _connection_reports: list[Connection.Report]
    _is_updating: bool  # Some things should only trigger during explicit user events

    def __init__(
        self,
        parent: wx.Window,
        controller: MCTController,
        name: str = "ControllerPanel"
    ):
        super().__init__(
            parent=parent,
            name=name)
        self._controller = controller

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

        self.add_horizontal_line_to_spacer(
            parent=control_panel,
            sizer=control_sizer)

        self._load_configuration_button: wx.Button = self.add_control_button(
            parent=control_panel,
            sizer=control_sizer,
            label="Load Configuration")

        self._start_button: wx.Button = self.add_control_button(
            parent=control_panel,
            sizer=control_sizer,
            label="Start")

        self._stop_button: wx.Button = self.add_control_button(
            parent=control_panel,
            sizer=control_sizer,
            label="Stop")

        self.add_horizontal_line_to_spacer(
            parent=control_panel,
            sizer=control_sizer)

        self._controller_status_textbox = wx.TextCtrl(
            parent=control_panel,
            style=wx.TE_READONLY | wx.TE_RICH)
        self._controller_status_textbox.SetEditable(False)
        self._controller_status_textbox.SetBackgroundColour(colour=wx.Colour(red=249, green=249, blue=249, alpha=255))
        control_sizer.Add(
            window=self._controller_status_textbox,
            flags=wx.SizerFlags(0).Expand())
        control_sizer.AddSpacer(size=BasePanel.DEFAULT_SPACING_PX_VERTICAL)

        self._connection_table = ConnectionTable(parent=control_panel)
        control_sizer.Add(
            window=self._connection_table,
            flags=wx.SizerFlags(0).Expand())
        control_sizer.AddSpacer(size=BasePanel.DEFAULT_SPACING_PX_VERTICAL)

        control_spacer_sizer: wx.BoxSizer = wx.BoxSizer(orient=wx.HORIZONTAL)
        control_sizer.Add(
            sizer=control_spacer_sizer,
            flags=wx.SizerFlags(1).Expand())

        control_panel.SetSizerAndFit(sizer=control_sizer)
        control_border_box.Add(
            window=control_panel,
            flags=wx.SizerFlags(1).Expand())
        control_border_panel.SetSizer(sizer=control_border_box)

        horizontal_split_sizer.AddStretchSpacer()
        horizontal_split_sizer.Add(
            window=control_border_panel,
            flags=wx.SizerFlags(1).Expand())
        horizontal_split_sizer.AddStretchSpacer()

        self.SetSizerAndFit(sizer=horizontal_split_sizer)

        self._load_configuration_button.Bind(
            event=wx.EVT_BUTTON,
            handler=self.on_ui_load_configuration_pressed)
        self._start_button.Bind(
            event=wx.EVT_BUTTON,
            handler=self.on_ui_start_pressed)
        self._stop_button.Bind(
            event=wx.EVT_BUTTON,
            handler=self.on_ui_stop_pressed)

        self._controller_state = str()
        self._connection_reports = list()
        self._is_updating = False

        self.update_controller_buttons()

    def on_ui_load_configuration_pressed(self, _event: wx.CommandEvent) -> None:
        dialog: wx.FileDialog = wx.FileDialog(
            parent=self,
            message="Select a configuration file",
            style=wx.FD_OPEN | wx.FD_FILE_MUST_EXIST)
        if dialog.ShowModal() == wx.ID_CANCEL:
            return
        self._controller.configure_from_filepath(dialog.GetPath())
        self.update_controller_buttons()

    def on_ui_start_pressed(self, _event: wx.CommandEvent) -> None:
        self._controller.start_up()
        self.update_controller_buttons()

    def on_ui_stop_pressed(self, _event: wx.CommandEvent) -> None:
        self._controller.shut_down()
        self.update_controller_buttons()

    def update_loop(self):
        super().update_loop()
        self._is_updating = True
        self.update_connection_table_display()
        controller_state: str = self._controller.get_controller_state()
        if controller_state != self._controller_state:
            self._controller_state = controller_state
            self._controller_status_textbox.SetValue(f"MCTController Status: {controller_state}")
            self.update_controller_buttons()
        self._is_updating = False

    def update_controller_buttons(self):
        self._load_configuration_button.Enable(enable=False)
        self._start_button.Enable(enable=False)
        self._stop_button.Enable(enable=False)
        if self._controller_state == MCTController.State.RUNNING:
            self._stop_button.Enable(enable=True)
        elif self._controller_state == MCTController.State.INITIAL:
            self._load_configuration_button.Enable(enable=True)
        elif self._controller_state == MCTController.State.CONFIGURED:
            self._load_configuration_button.Enable(enable=True)
            self._start_button.Enable(enable=True)

    def update_connection_table_display(self) -> None:
        # Return if there is no change
        connection_reports: list[Connection.Report] = self._controller.get_connection_reports()
        if len(connection_reports) == len(self._connection_reports):
            identical: bool = True
            for connection_report in connection_reports:
                contained: bool = connection_report in self._connection_reports
                if not contained:
                    identical = False
                    break
            if identical:
                return
        # There has been a change so update internal variables and UI
        self._connection_reports = connection_reports
        self._connection_table.update_contents(row_contents=self._connection_reports)
        selected_row_index: int | None = self._connection_table.get_selected_row_index()
        if selected_row_index is not None and selected_row_index >= len(self._connection_reports):
            selected_row_index = None
        self._connection_table.set_selected_row_index(selected_row_index)
