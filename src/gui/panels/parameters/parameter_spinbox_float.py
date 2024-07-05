from .parameter_base import ParameterBase
import wx


# noinspection DuplicatedCode
class ParameterSpinboxFloat(ParameterBase):
    spinbox: wx.SpinCtrlDouble

    def __init__(
        self,
        parent: wx.Window,
        label: str,
        minimum_value: float,
        maximum_value: float,
        initial_value: float,
        step_value: float,
        space_px: int = ParameterBase.SPACE_PX_DEFAULT
    ):
        super().__init__(parent=parent)
        sizer: wx.BoxSizer = wx.BoxSizer(orient=wx.HORIZONTAL)
        self.add_label_and_space(sizer=sizer, label_text=label, space_px=space_px)
        self.spinbox: wx.SpinCtrlDouble = wx.SpinCtrlDouble(
            parent=self,
            min=minimum_value,
            max=maximum_value,
            initial=initial_value,
            inc=step_value)
        sizer.Add(window=self.spinbox, flags=wx.SizerFlags(1))
        self.SetSizerAndFit(sizer=sizer)

    def get_value(self) -> float:
        return float(self.spinbox.GetValue())

    def set_enabled(
        self,
        enable: bool
    ) -> None:
        super().set_enabled(enable=enable)
        self.spinbox.Enable(enable=enable)
