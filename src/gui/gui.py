from src.gui.panels.specialized import LogPanel
from src.gui.panels import \
    BasePanel, \
    BoardBuilderPanel, \
    IntrinsicsPanel, \
    ExtrinsicsPanel, \
    ControllerPanel, \
    DetectorPanel, \
    PoseSolverPanel
from src.common import SeverityLabel, StatusMessage, StatusMessageSource
from src.controller import MCTController
import logging
import wx
from typing import Final


CONTROLLER_LABEL: Final[str] = "Controller"
DETECTOR_LABEL: Final[str] = "Detector"
INTRINSIC_CALIBRATOR_LABEL: Final[str] = "Intrinsic Calibrator"
EXTRINSIC_CALIBRATOR_LABEL: Final[str] = "Extrinsic Calibrator"
BOARD_BUILDER_LABEL: Final[str] = "Board Builder"
POSE_SOLVER_LABEL: Final[str] = "Pose Solver"

_STATUS_BAR_FIELD_INDEX_TEXT: Final[int] = 0
_STATUS_BAR_FIELD_INDEX_BUTTON: Final[int] = 1
_STATUS_BAR_FIELD_COUNT: Final[int] = 2
_STATUS_BAR_HEIGHT_PX: Final[int] = 40
_STATUS_LOG_HEIGHT_PX: Final[int] = 120
_STATUS_MESSAGE_TABLE_SUBSCRIBER_LABEL: Final[str] = "status_message_table"


# noinspection PyMethodMayBeStatic
class ControllerFrame(wx.Frame):

    _controller: MCTController
    _status_message_source: StatusMessageSource

    _notebook: wx.Notebook
    _panels: set[BasePanel]
    _controller_panel: ControllerPanel
    _detector_panel: DetectorPanel
    _intrinsics_panel: IntrinsicsPanel
    _extrinsics_panel: ExtrinsicsPanel
    _board_builder_panel: BoardBuilderPanel
    _pose_solver_panel: PoseSolverPanel

    _frame_main_panel: wx.Panel
    _frame_main_sizer: wx.Sizer
    _status_bar: wx.StatusBar
    _status_button: wx.Button
    _status_log_visible: bool

    def __init__(
        self,
        controller: MCTController,
        *args,
        **kwargs
    ):
        super().__init__(*args, **kwargs)

        self._controller = controller
        self._panels = set()

        self._status_message_source = self._controller.get_status_message_source()
        self._status_message_source.add_status_subscriber(subscriber_label=_STATUS_MESSAGE_TABLE_SUBSCRIBER_LABEL)

        self.SetMinSize(wx.Size(800, 600))

        self._frame_main_panel = wx.Panel(parent=self)

        self._notebook = wx.Notebook(parent=self._frame_main_panel)

        self._log_panel = LogPanel(parent=self._frame_main_panel)
        self._log_panel.SetBackgroundColour(colour=wx.BLACK)

        self._frame_main_sizer: wx.BoxSizer = wx.BoxSizer(wx.VERTICAL)
        self._frame_main_sizer.Add(
            window=self._notebook,
            flags=wx.SizerFlags(100).Expand())
        self._frame_main_sizer.Add(
            window=self._log_panel,
            flags=wx.SizerFlags(0).Expand())
        self._frame_main_panel.SetSizerAndFit(self._frame_main_sizer)

        self._controller_panel = ControllerPanel(
            parent=self._notebook,
            controller=self._controller)
        self._notebook.AddPage(
            page=self._controller_panel,
            text=CONTROLLER_LABEL,
            select=True)
        self._controller_panel.panel_is_selected = True
        self._panels.add(self._controller_panel)

        self._detector_panel = DetectorPanel(
            parent=self._notebook,
            controller=self._controller)
        self._notebook.AddPage(
            page=self._detector_panel,
            text=DETECTOR_LABEL,
            select=False)
        self._panels.add(self._detector_panel)

        self._intrinsics_panel = IntrinsicsPanel(
            parent=self._notebook,
            controller=self._controller)
        self._notebook.AddPage(
            page=self._intrinsics_panel,
            text=INTRINSIC_CALIBRATOR_LABEL,
            select=False)
        self._panels.add(self._intrinsics_panel)

        self._extrinsics_panel = ExtrinsicsPanel(
            parent=self._notebook,
            controller=self._controller)
        self._notebook.AddPage(
            page=self._extrinsics_panel,
            text=EXTRINSIC_CALIBRATOR_LABEL,
            select=False)
        self._panels.add(self._extrinsics_panel)

        # self._board_builder_panel = BoardBuilderPanel(
        #     parent=self._notebook,
        #     controller=self._controller)
        # self._notebook.AddPage(
        #     page=self._board_builder_panel,
        #     text=BOARD_BUILDER_LABEL,
        #     select=False)
        # self._panels.add(self._board_builder_panel)
        #
        # self._pose_solver_panel = PoseSolverPanel(
        #     parent=self._notebook,
        #     controller=self._controller)
        # self._notebook.AddPage(
        #     page=self._pose_solver_panel,
        #     text=POSE_SOLVER_LABEL,
        #     select=False)
        # self._panels.add(self._pose_solver_panel)

        self._notebook.Bind(
            event=wx.EVT_BOOKCTRL_PAGE_CHANGED,
            handler=self.on_page_changed)

        self._status_bar = self.CreateStatusBar()
        self._status_bar.SetMinSize(size=wx.Size(0, _STATUS_BAR_HEIGHT_PX))
        self._status_bar.SetFieldsCount(number=_STATUS_BAR_FIELD_COUNT, widths=[-1, _STATUS_BAR_HEIGHT_PX])
        self.SetStatusText(text="", number=_STATUS_BAR_FIELD_INDEX_TEXT)
        self.SetStatusText(text="", number=_STATUS_BAR_FIELD_INDEX_BUTTON)
        self._status_button = wx.Button(
            parent=self._status_bar,
            label="Log")
        self._status_button.Bind(
            event=wx.EVT_BUTTON,
            handler=self.on_status_button_pressed)
        self._status_bar.Bind(
            event=wx.EVT_SIZE,
            handler=self.on_status_bar_sized)
        self._status_log_visible = False
        self.on_status_bar_sized()

        self.Bind(
            event=wx.EVT_IDLE,
            handler=self.update)

    def on_status_bar_sized(self, _event: wx.SizeEvent | None = None):
        field_rect: wx.Rect = self._status_bar.GetFieldRect(_STATUS_BAR_FIELD_INDEX_BUTTON)
        self._status_button.SetSize(rect=field_rect)

    def on_status_button_pressed(self, _event: wx.CommandEvent):
        if self._status_log_visible:
            self._status_log_visible = False
            self._log_panel.SetMinSize(size=wx.Size(0, 0))
        else:
            self._status_log_visible = True
            self._log_panel.SetMinSize(size=wx.Size(0, _STATUS_LOG_HEIGHT_PX))
        self._frame_main_panel.Layout()

    def on_page_changed(self, event: wx.BookCtrlEvent):
        for page in self._panels:
            page_index: int = self._notebook.FindPage(page)
            if page_index == event.GetOldSelection():
                page.on_ui_page_deselect()
                break
        for page in self._panels:
            page_index: int = self._notebook.FindPage(page)
            if page_index == event.GetSelection():
                page.on_ui_page_select()
                break

    def update(self, *_args):
        # noinspection PyBroadException
        try:
            self._controller.update()
            self._update_log_table()
        except Exception as e:
            self._status_message_source.enqueue_status_message(
                severity=SeverityLabel.ERROR,
                message=f"Exception occurred in controller loop: {str(e)}")

    def _update_log_table(self):
        status_messages: list[StatusMessage] = self._status_message_source.pop_new_status_messages(
            subscriber_label=_STATUS_MESSAGE_TABLE_SUBSCRIBER_LABEL)
        self._log_panel.output_status_messages(status_messages=status_messages)


def main():
    logging.basicConfig(level=logging.INFO)
    controller = MCTController(
        controller_name="controller",
        send_status_messages_to_logger=True)
    app: wx.App = wx.App()
    frame: ControllerFrame = ControllerFrame(
        controller=controller,
        parent=None,
        title="MCT Controller")
    frame.Show()
    app.MainLoop()


if __name__ == "__main__":
    main()
