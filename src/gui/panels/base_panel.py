from .parameters import \
    ParameterBase, \
    ParameterCheckbox, \
    ParameterSelector, \
    ParameterSpinboxFloat, \
    ParameterSpinboxInteger, \
    ParameterText
from src.common import \
    KeyValueSimpleAbstract, \
    KeyValueSimpleAny, \
    KeyValueSimpleBool, \
    KeyValueSimpleString, \
    KeyValueSimpleFloat, \
    KeyValueSimpleInt, \
    KeyValueMetaAbstract, \
    KeyValueMetaAny, \
    KeyValueMetaBool, \
    KeyValueMetaEnum, \
    KeyValueMetaFloat, \
    KeyValueMetaInt, \
    SeverityLabel, \
    StatusMessageSource
from typing import Final
import wx


_UPDATE_INTERVAL_MILLISECONDS: Final[int] = 16


class BasePanel(wx.Panel):

    panel_is_selected: bool
    DEFAULT_SPACING_PX_VERTICAL: Final[int] = 4
    DEFAULT_SPACING_PX_LINE_TOP_BOTTOM: Final[int] = 8

    _update_loop_running: bool

    def __init__(
        self,
        parent: wx.Window,
        name: str
    ):
        super().__init__(parent=parent, name=name)
        self.panel_is_selected = False

        self._update_loop_running = True
        wx.CallLater(_UPDATE_INTERVAL_MILLISECONDS, self.update_loop)

    @staticmethod
    def populate_key_value_list_from_dynamic_ui(
        parameter_uis: list[ParameterBase],
        status_message_source: StatusMessageSource | None = None
    ) -> list[KeyValueSimpleAny]:
        key_values: list[KeyValueSimpleAny] = list()
        for parameter_ui in parameter_uis:
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
                if status_message_source is not None:
                    status_message_source.enqueue_status_message(
                        severity=SeverityLabel.ERROR,
                        message=f"Failed to determine parameter type from UI element for key {label}.")
                continue
            key_values.append(parameter_type(
                key=label,
                value=parameter_ui.get_value()))
        return key_values

    @staticmethod
    def populate_dynamic_ui_from_key_value_list(
        key_value_list: list[KeyValueMetaAny],
        containing_panel: wx.Panel,
        containing_sizer: wx.BoxSizer,
        status_message_source: StatusMessageSource | None = None
    ) -> list[ParameterBase]:
        return_value: list[ParameterBase] = list()
        key_value: KeyValueMetaAbstract
        for key_value in key_value_list:
            if isinstance(key_value, KeyValueMetaBool):
                return_value.append(BasePanel.add_control_checkbox(
                    parent=containing_panel,
                    sizer=containing_sizer,
                    label=key_value.key,
                    value=key_value.value))
            elif isinstance(key_value, KeyValueMetaEnum):
                return_value.append(BasePanel.add_control_selector(
                    parent=containing_panel,
                    sizer=containing_sizer,
                    label=key_value.key,
                    selectable_values=key_value.allowable_values,
                    value=key_value.value))
            elif isinstance(key_value, KeyValueMetaFloat):
                return_value.append(BasePanel.add_control_spinbox_float(
                    parent=containing_panel,
                    sizer=containing_sizer,
                    label=key_value.key,
                    minimum_value=key_value.range_minimum,
                    maximum_value=key_value.range_maximum,
                    initial_value=key_value.value,
                    step_value=key_value.range_step,
                    digit_count=key_value.digit_count))
            elif isinstance(key_value, KeyValueMetaInt):
                return_value.append(BasePanel.add_control_spinbox_integer(
                    parent=containing_panel,
                    sizer=containing_sizer,
                    label=key_value.key,
                    minimum_value=key_value.range_minimum,
                    maximum_value=key_value.range_maximum,
                    initial_value=key_value.value,
                    step_value=key_value.range_step))
            else:
                if status_message_source is not None:
                    status_message_source.enqueue_status_message(
                        severity=SeverityLabel.ERROR,
                        message=f"Unsupported parameter type {key_value.parsable_type} will not be handled")
        return return_value

    def on_ui_page_select(self):
        self.panel_is_selected = True
        if not self._update_loop_running:
            self._update_loop_running = True
            self.update_loop()

    def on_ui_page_deselect(self):
        self.panel_is_selected = False

    def update_loop(self) -> None:
        """
        Overload for anything that should be updated approximately once per GUI frame
        """
        if not self.panel_is_selected:
            self._update_loop_running = False
            return
        wx.CallLater(_UPDATE_INTERVAL_MILLISECONDS, self.update_loop)

    # -------------------------------------------------------------------------------------

    @staticmethod
    def add_control_button(
        parent: wx.Window,
        sizer: wx.BoxSizer,
        label: str
    ) -> wx.Button:
        button = wx.Button(
            parent=parent,
            label=label)
        sizer.Add(
            window=button,
            flags=wx.SizerFlags(0).Expand())
        sizer.AddSpacer(size=BasePanel.DEFAULT_SPACING_PX_VERTICAL)
        return button

    @staticmethod
    def add_control_checkbox(
        parent: wx.Window,
        sizer: wx.BoxSizer,
        label: str,
        value: bool = False
    ) -> ParameterCheckbox:
        checkbox = ParameterCheckbox(
            parent=parent,
            label=label,
            value=value)
        sizer.Add(
            window=checkbox,
            flags=wx.SizerFlags(0).Expand())
        sizer.AddSpacer(size=BasePanel.DEFAULT_SPACING_PX_VERTICAL)
        return checkbox

    @staticmethod
    def add_control_selector(
        parent: wx.Window,
        sizer: wx.BoxSizer,
        label: str,
        selectable_values: list[str],
        value: str | None = None
    ) -> ParameterSelector:
        selector = ParameterSelector(
            parent=parent,
            label=label,
            selectable_values=selectable_values,
            value=value)
        sizer.Add(
            window=selector,
            flags=wx.SizerFlags(0).Expand())
        sizer.AddSpacer(size=BasePanel.DEFAULT_SPACING_PX_VERTICAL)
        return selector

    @staticmethod
    def add_control_spinbox_float(
        parent: wx.Window,
        sizer: wx.BoxSizer,
        label: str,
        minimum_value: float,
        maximum_value: float,
        initial_value: float,
        step_value: float,
        digit_count: int = 2
    ) -> ParameterSpinboxFloat:
        spinbox = ParameterSpinboxFloat(
            parent=parent,
            label=label,
            minimum_value=minimum_value,
            maximum_value=maximum_value,
            initial_value=initial_value,
            step_value=step_value,
            digit_count=digit_count)
        sizer.Add(
            window=spinbox,
            flags=wx.SizerFlags(0).Expand())
        sizer.AddSpacer(size=BasePanel.DEFAULT_SPACING_PX_VERTICAL)
        return spinbox

    @staticmethod
    def add_control_spinbox_integer(
        parent: wx.Window,
        sizer: wx.BoxSizer,
        label: str,
        minimum_value: int,
        maximum_value: int,
        initial_value: int,
        step_value: int = 1
    ) -> ParameterSpinboxInteger:
        spinbox = ParameterSpinboxInteger(
            parent=parent,
            label=label,
            minimum_value=minimum_value,
            maximum_value=maximum_value,
            initial_value=initial_value,
            step_value=step_value)
        sizer.Add(
            window=spinbox,
            flags=wx.SizerFlags(0).Expand())
        sizer.AddSpacer(size=BasePanel.DEFAULT_SPACING_PX_VERTICAL)
        return spinbox

    @staticmethod
    def add_control_text_input(
        parent: wx.Window,
        sizer: wx.BoxSizer,
        label: str,
        value: str = str()
    ) -> ParameterText:
        textbox = ParameterText(
            parent=parent,
            label=label,
            value=value)
        sizer.Add(
            window=textbox,
            flags=wx.SizerFlags(0).Expand())
        sizer.AddSpacer(size=BasePanel.DEFAULT_SPACING_PX_VERTICAL)
        return textbox

    @staticmethod
    def add_horizontal_line_to_spacer(
        parent: wx.Window,
        sizer: wx.BoxSizer
    ) -> None:
        line: wx.StaticLine = wx.StaticLine(parent=parent)
        sizer.Add(
            window=line,
            flags=wx.SizerFlags(0).Border(
                direction=(wx.TOP | wx.BOTTOM),
                borderinpixels=BasePanel.DEFAULT_SPACING_PX_LINE_TOP_BOTTOM).Expand())

    @staticmethod
    def add_text_label(
        parent: wx.Window,
        sizer: wx.BoxSizer,
        label: str,
        font_size_delta: int | None = None,
        bold: bool | None = None
    ) -> None:
        line: wx.StaticText = wx.StaticText(parent=parent, label=label)
        font: wx.Font = line.GetFont()
        if font_size_delta is not None:
            font.SetPointSize(pointSize=font.GetPointSize() + font_size_delta)
        if bold:
            font.MakeBold()
        line.SetFont(font)
        sizer.Add(
            window=line,
            flags=wx.SizerFlags(0).Border(
                direction=(wx.TOP | wx.BOTTOM),
                borderinpixels=BasePanel.DEFAULT_SPACING_PX_LINE_TOP_BOTTOM).Expand())

    @staticmethod
    def add_toggle_button(
        parent: wx.Window,
        sizer: wx.BoxSizer,
        label: str
    ) -> wx.ToggleButton:
        toggle_button = wx.ToggleButton(
            parent=parent,
            label=label)
        sizer.Add(
            window=toggle_button,
            flags=wx.SizerFlags(0).Expand())
        sizer.AddSpacer(size=BasePanel.DEFAULT_SPACING_PX_VERTICAL)
        return toggle_button
