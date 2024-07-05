from .parameter_base import ParameterBase
import wx


class ParameterCheckbox(ParameterBase):
    checkbox: wx.CheckBox

    def __init__(
        self,
        parent: wx.Window,
        label: str,
        value: bool = False,
        space_px: int = ParameterBase.SPACE_PX_DEFAULT
    ):
        super().__init__(parent=parent)
        sizer: wx.BoxSizer = wx.BoxSizer(orient=wx.HORIZONTAL)
        self.add_label_and_space(sizer=sizer, label_text=label, space_px=space_px)
        self.checkbox: wx.CheckBox = wx.CheckBox(parent=self)
        self.checkbox.SetValue(state=value)
        sizer.Add(window=self.checkbox, flags=wx.SizerFlags(1))
        self.SetSizerAndFit(sizer=sizer)

    def get_value(self) -> bool:
        return bool(self.checkbox.GetValue())

    def set_enabled(
        self,
        enable: bool
    ) -> None:
        super().set_enabled(enable=enable)
        self.checkbox.Enable(enable=enable)
