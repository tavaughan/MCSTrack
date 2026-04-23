from src.gui.panels.feedback.image_panel import ImagePanel
from src.common import \
    ImageFormat, \
    ImageUtils
from io import BytesIO
import numpy
import wx


class DetectorMultiFramePanel(ImagePanel):

    _draw_image: bool

    def __init__(
        self,
        parent: wx.Window
    ):
        super().__init__(parent=parent)
        self._draw_image = False

    def set_draw_image(self, enabled):
        self._draw_image = enabled

    def update_image(
        self,
        images_base64: list[str | None] | None = None
    ) -> None:
        """
        Draw the specified frames according to the settings in this class and the available data.
        If no image data is available (images_base64 is None), then the preview will be black.
        :param images_base64:
        """
        if images_base64 is None:
            images_base64 = list()

        panel_size: wx.Size = self.GetSize()
        panel_size_tuple: tuple[int, int] = (panel_size.x, panel_size.y)
        display_image: numpy.ndarray

        display_image = ImageUtils.black_image(resolution_px=panel_size_tuple)
        if self._draw_image and len(images_base64) > 0:
            image_dimensions: tuple[int, int]
            image_positions: list[tuple[int, int]]
            image_dimensions, image_positions = ImageUtils.partition_rect(
                available_size_px=panel_size_tuple,
                partition_count=len(images_base64))
            for image_index, image_base64 in enumerate(images_base64):
                if image_base64 is None:
                    continue  # No data available (yet)
                image: numpy.ndarray = ImageUtils.base64_to_image(input_base64=image_base64)
                image = ImageUtils.image_resize_to_fit(
                    opencv_image=image,
                    available_size=image_dimensions)
                offset_y_px: int = image_positions[image_index][1] + (image_dimensions[1] - image.shape[0]) // 2
                offset_x_px: int = image_positions[image_index][0] + (image_dimensions[0] - image.shape[1]) // 2
                display_image[
                    offset_y_px:offset_y_px + image.shape[0],
                    offset_x_px:offset_x_px + image.shape[1]
                ] = image

        image_buffer: bytes = ImageUtils.image_to_bytes(image_data=display_image, image_format=ImageFormat.FORMAT_JPG)
        image_buffer_io: BytesIO = BytesIO(image_buffer)
        # noinspection PyTypeChecker
        wx_image: wx.Image = wx.Image(image_buffer_io)
        wx_bitmap: wx.Bitmap = wx_image.ConvertToBitmap()
        self.set_bitmap(wx_bitmap)
        self.paint()
