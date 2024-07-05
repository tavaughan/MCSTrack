from .parameter_base import ParameterBase
import wx


class ParameterText(ParameterBase):
    textbox: wx.TextCtrl

    def __init__(
        self,
        parent: wx.Window,
        label: str,
        value: str = str(),
        space_px: int = ParameterBase.SPACE_PX_DEFAULT
    ):
        super().__init__(parent=parent)
        sizer: wx.BoxSizer = wx.BoxSizer(orient=wx.HORIZONTAL)
        self.add_label_and_space(sizer=sizer, label_text=label, space_px=space_px)
        self.textbox: wx.TextCtrl = wx.TextCtrl(parent=self, value=value)
        sizer.Add(window=self.textbox, flags=wx.SizerFlags(1))
        self.SetSizerAndFit(sizer=sizer)

    def get_value(self) -> str:
        return str(self.textbox.GetValue())

    def set_enabled(
        self,
        enable: bool
    ):
        super().set_enabled(enable=enable)
        self.textbox.Enable(enable=enable)
