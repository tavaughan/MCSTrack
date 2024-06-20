from .row_selection_table import RowSelectionTable
from src.connector import \
    ConnectionReport
from typing import Final
import wx


_COL_IDX_LABEL: Final[int] = 0
_COL_IDX_ROLE: Final[int] = 1
_COL_IDX_IP_ADDRESS: Final[int] = 2
_COL_IDX_PORT: Final[int] = 3
_COL_IDX_STATUS: Final[int] = 4
_COL_COUNT: Final[int] = 5
_COL_LABELS: Final[list[str]] = ["Label", "Role", "IP Address", "Port", "Status"]


class ConnectionTable(RowSelectionTable[ConnectionReport]):
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
        row_content: ConnectionReport
    ):
        self.table.SetCellValue(
            row=row_index,
            col=_COL_IDX_LABEL,
            s=row_content.label)
        self.table.SetCellValue(
            row=row_index,
            col=_COL_IDX_ROLE,
            s=row_content.role)
        self.table.SetCellValue(
            row=row_index,
            col=_COL_IDX_IP_ADDRESS,
            s=str(row_content.ip_address))
        self.table.SetCellValue(
            row=row_index,
            col=_COL_IDX_PORT,
            s=str(row_content.port))
        self.table.SetCellValue(
            row=row_index,
            col=_COL_IDX_STATUS,
            s=row_content.status)
