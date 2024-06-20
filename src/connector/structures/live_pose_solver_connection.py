from .live_connection import LiveConnection
from src.common.structures import Pose
import datetime
import uuid


class LivePoseSolverConnection(LiveConnection):
    request_id: uuid.UUID | None
    detector_poses: list[Pose]
    target_poses: list[Pose]
    detector_timestamps: dict[str, datetime.datetime]  # access by detector_label
    poses_timestamp: datetime.datetime

    def reset(self):
        super().reset()
        self.request_id = None
        self.detector_poses = list()
        self.target_poses = list()
        self.detector_timestamps = dict()
        self.poses_timestamp = datetime.datetime.min
