from .parameter_base import ParameterBase
import wx


class ParameterSelector(ParameterBase):
    selector: wx.Choice

    def __init__(
        self,
        parent: wx.Window,
        label: str,
        selectable_values: list[str],
        value: str | None = None,
        space_px: int = ParameterBase.SPACE_PX_DEFAULT
    ):
        super().__init__(parent=parent)
        sizer: wx.BoxSizer = wx.BoxSizer(orient=wx.HORIZONTAL)
        self.add_label_and_space(sizer=sizer, label_text=label, space_px=space_px)
        self.selector: wx.Choice = wx.Choice(parent=self)
        self.selector.AppendItems(items=selectable_values)
        if value is not None:
            self.selector.SetStringSelection(value)
        else:
            self.selector.SetSelection(n=0)
        sizer.Add(window=self.selector, flags=wx.SizerFlags(1))
        self.SetSizerAndFit(sizer=sizer)

    def get_value(self) -> str:
        return self.selector.GetStringSelection()

    def set_enabled(
        self,
        enable: bool
    ) -> None:
        super().set_enabled(enable=enable)
        self.selector.Enable(enable=enable)

    def set_options(
        self,
        option_list: list[str]
    ) -> None:
        self.selector.Clear()
        self.selector.Append(items=option_list)
