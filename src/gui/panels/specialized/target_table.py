from .row_selection_table import RowSelectionTable
from pydantic import BaseModel, Field
from typing import Final
import wx


_COL_IDX_TARGET_ID: Final[int] = 0
_COL_IDX_LABEL: Final[int] = 1
_COL_IDX_X: Final[int] = 2
_COL_IDX_Y: Final[int] = 3
_COL_IDX_Z: Final[int] = 4
_COL_COUNT: Final[int] = 5
_COL_LABELS: Final[list[str]] = ["Target ID", "Label", "X", "Y", "Z"]


class TrackingTableRow(BaseModel):
    target_id: str = Field()
    label: str = Field()
    x: float = Field()
    y: float = Field()
    z: float = Field()


class TargetTable(RowSelectionTable[TrackingTableRow]):

    def __init__(
        self,
        parent: wx.Window,
        height_px: int = RowSelectionTable.DEFAULT_HEIGHT_PX
    ):
        super().__init__(
            parent=parent,
            col_labels=_COL_LABELS,
            height_px=height_px)

    def get_selected_target_label(self) -> str | None:
        row_indices: list[int] = self.table.GetSelectedRows()
        if len(row_indices) <= 0:
            return None
        return self.table.GetCellValue(
            row=row_indices[0],
            col=_COL_IDX_LABEL)

    def _set_row_contents(
        self,
        row_index: int,
        row_content: TrackingTableRow
    ):
        self.table.SetCellValue(
            row=row_index,
            col=_COL_IDX_TARGET_ID,
            s=row_content.target_id)
        self.table.SetCellValue(
            row=row_index,
            col=_COL_IDX_LABEL,
            s=row_content.label)
        self.table.SetCellValue(
            row=row_index,
            col=_COL_IDX_X,
            s=str(row_content.x))
        self.table.SetCellValue(
            row=row_index,
            col=_COL_IDX_Y,
            s=str(row_content.y))
        self.table.SetCellValue(
            row=row_index,
            col=_COL_IDX_Z,
            s=str(row_content.z))
