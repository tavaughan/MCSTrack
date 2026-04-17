from src.gui.panels.feedback.image_panel import ImagePanel
from src.common import \
    Annotation, \
    DetectorFrame, \
    ImageFormat, \
    ImageResolution, \
    ImageUtils
import cv2
from io import BytesIO
import numpy
import wx


def _marker_snapshot_list_to_opencv_points(
    marker_snapshot_list: list[Annotation],
    scale: float
) -> numpy.ndarray:
    if len(marker_snapshot_list) <= 0:
        return numpy.asarray([], dtype=numpy.int32)
    return_value: list[list[list[float]]] = list()
    current_base_label: str | None = None
    current_sequence_number: int = -1
    current_shape_points: list[list[float]] | None = None
    for marker_snapshot in marker_snapshot_list:
        annotation_base_label: str = marker_snapshot.base_feature_label()
        annotation_sequence_number: int = marker_snapshot.sequence_number()
        if annotation_base_label != current_base_label or \
           annotation_sequence_number != current_sequence_number + 1:
            if current_shape_points is not None:
                return_value.append(current_shape_points)
            current_shape_points = list()
            current_base_label = annotation_base_label
        current_shape_points.append([
            marker_snapshot.x_px * scale,
            marker_snapshot.y_px * scale])
        current_sequence_number = annotation_sequence_number
    return_value.append(current_shape_points)
    return_value: numpy.ndarray = numpy.asarray(return_value, dtype=numpy.int32)
    return return_value


class DetectorSingleFramePanel(ImagePanel):

    draw_image: bool
    draw_annotations_detected: bool
    draw_annotations_rejected: bool

    def __init__(
        self,
        parent: wx.Window
    ):
        super().__init__(parent=parent)
        self.draw_image = False
        self.draw_annotations_detected = False
        self.draw_annotations_rejected = False

    def set_draw_image(self, enabled):
        self.draw_image = enabled

    def set_draw_annotations_detected(self, enabled):
        self.draw_annotations_detected = enabled

    def set_draw_annotations_rejected(self, enabled):
        self.draw_annotations_rejected = enabled

    def update_image(
        self,
        capture_resolution: ImageResolution | None = None,
        frame: DetectorFrame | None = None
    ) -> None:
        """
        Draw the specified frame according to the settings in this class and the available data.
        If no image data is available (frame.image_base64 is None), then
        capture_resolution must be provided for the annotations to be drawn.
        If insufficient data is provided, or if the class is configured to draw nothing,
        then the preview will be black.
        :param capture_resolution: The resolution of the capture. Required for scale information.
        :param frame: The data to be drawn.
        """
        panel_size: wx.Size = self.GetSize()
        panel_size_tuple: tuple[int, int] = (panel_size.x, panel_size.y)
        display_image: numpy.ndarray
        if (
            (not self.draw_image and not self.draw_annotations_detected and not self.draw_annotations_rejected) or
            (frame is None) or
            (capture_resolution is None and frame.image_base64 is None)
        ):
            display_image = ImageUtils.black_image(resolution_px=panel_size_tuple)
        else:
            scale: float
            if self.draw_image and frame.image_base64 is not None:
                opencv_image: numpy.ndarray = ImageUtils.base64_to_image(input_base64=frame.image_base64)
                display_image: numpy.ndarray = ImageUtils.image_resize_to_fit(
                    opencv_image=opencv_image,
                    available_size=panel_size_tuple)
                cv2.cvtColor(display_image, cv2.COLOR_RGB2BGR, display_image)
                scale: float = display_image.shape[0] / opencv_image.shape[0]
            else:
                display_image = ImageUtils.black_image(resolution_px=panel_size_tuple)
                rescaled_resolution_px: tuple[int, int] = ImageUtils.scale_factor_for_available_space_px(
                    source_resolution_px=(capture_resolution.x_px, capture_resolution.y_px),
                    available_size_px=panel_size_tuple)
                scale: float = rescaled_resolution_px[1] / capture_resolution.y_px

            if self.draw_annotations_detected:
                identified_annotations: list[Annotation] = [
                    annotation
                    for annotation in frame.annotations
                    if annotation.base_feature_label() != Annotation.UNIDENTIFIED_LABEL]
                corners: numpy.ndarray = _marker_snapshot_list_to_opencv_points(
                    marker_snapshot_list=identified_annotations,
                    scale=scale)
                cv2.polylines(
                    img=display_image,
                    pts=corners,
                    isClosed=True,
                    color=[255, 191, 127],  # blue in BGR
                    thickness=2)
            if self.draw_annotations_rejected:
                unidentified_annotations: list[Annotation] = [
                    annotation
                    for annotation in frame.annotations
                    if annotation.base_feature_label() == Annotation.UNIDENTIFIED_LABEL]
                corners: numpy.ndarray = _marker_snapshot_list_to_opencv_points(
                    marker_snapshot_list=unidentified_annotations,
                    scale=scale)
                cv2.polylines(
                    img=display_image,
                    pts=corners,
                    isClosed=True,
                    color=[127, 191, 255],  # orange in BGR
                    thickness=2)

        image_buffer: bytes = ImageUtils.image_to_bytes(image_data=display_image, image_format=ImageFormat.FORMAT_JPG)
        image_buffer_io: BytesIO = BytesIO(image_buffer)
        # noinspection PyTypeChecker
        wx_image: wx.Image = wx.Image(image_buffer_io)
        wx_bitmap: wx.Bitmap = wx_image.ConvertToBitmap()
        self.set_bitmap(wx_bitmap)
        self.paint()
