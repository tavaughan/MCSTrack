from .row_selection_table import RowSelectionTable
from src.detector.structures import CalibrationResultMetadata
from typing import Final
import wx


_COL_IDX_IDENTIFIER: Final[int] = 0
_COL_IDX_LABEL: Final[int] = 1
_COL_IDX_TIMESTAMP: Final[int] = 2
_COL_IDX_STATUS: Final[int] = 3
_COL_COUNT: Final[int] = 4
_COL_LABELS: Final[list[str]] = ["Identifier", "Label", "Timestamp", "Status"]


class CalibrationResultTable(RowSelectionTable[CalibrationResultMetadata]):
    def __init__(
        self,
        parent: wx.Window,
        height_px: int = RowSelectionTable.DEFAULT_HEIGHT_PX
    ):
        super().__init__(
            parent=parent,
            col_labels=_COL_LABELS,
            height_px=height_px)

    def _set_row_contents(
        self,
        row_index: int,
        row_content: CalibrationResultMetadata
    ):
        self.table.SetCellValue(
            row=row_index,
            col=_COL_IDX_IDENTIFIER,
            s=row_content.identifier)
        self.table.SetCellValue(
            row=row_index,
            col=_COL_IDX_LABEL,
            s=row_content.label)
        self.table.SetCellValue(
            row=row_index,
            col=_COL_IDX_TIMESTAMP,
            s=str(row_content.timestamp_utc_iso8601))
        self.table.SetCellValue(
            row=row_index,
            col=_COL_IDX_STATUS,
            s=row_content.state.name)
