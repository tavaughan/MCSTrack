from src.common.structures import \
    CharucoBoardSpecification, \
    TargetBoard
from src.common.structures.target import _Marker


board: CharucoBoardSpecification = CharucoBoardSpecification()
points: list[list[float]] = board.get_marker_corner_points()
markers: list[_Marker] = list()
POINTS_PER_MARKER: int = 4
marker_count: int = round(int(len(points) / POINTS_PER_MARKER))
for marker_index in range(marker_count):
    point_start_index: int = marker_index * POINTS_PER_MARKER
    marker_points = points[point_start_index: point_start_index + POINTS_PER_MARKER]
    markers.append(_Marker(
        marker_id=f"{marker_index}",
        points=marker_points))
target: TargetBoard = TargetBoard(
    target_id="reference",
    markers=markers)
with open("temp.json", 'w') as outfile:
    outfile.write(target.json(exclude_unset=True, indent=2))
