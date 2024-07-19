from pydantic import BaseModel, Field
import math


class IntrinsicParameters(BaseModel):
    """
    Camera intrinsic parameters (focal length, optical center, distortion coefficients).
    See OpenCV's documentation: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
    See Wikipedia article: https://en.wikipedia.org/wiki/Distortion_%28optics%29
    """

    focal_length_x_px: float = Field()
    focal_length_y_px: float = Field()
    optical_center_x_px: float = Field()
    optical_center_y_px: float = Field()

    radial_distortion_coefficients: list[float] = Field()  # k1, k2, k3 etc in OpenCV

    tangential_distortion_coefficients: list[float] = Field()  # p1, p2 in OpenCV

    def as_array(self) -> list[float]:
        return_value: list[float] = [
            self.focal_length_x_px,
            self.focal_length_y_px,
            self.optical_center_x_px,
            self.optical_center_y_px]
        return_value += self.get_distortion_coefficients()
        return return_value

    def get_matrix(self) -> list[list[float]]:
        """calibration matrix expected by OpenCV in some operations"""
        return \
            [[self.focal_length_x_px, 0.0, self.optical_center_x_px],
             [0.0, self.focal_length_y_px, self.optical_center_y_px],
             [0.0, 0.0, 1.0]]

    def get_distortion_coefficients(self) -> list[float]:
        """
        Distortion coefficients in array format expected by OpenCV in some operations.
        See https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#ga3207604e4b1a1758aa66acb6ed5aa65d
        calibrateCamera() documentation describes order of distortion coefficients that OpenCV works with
        """
        coefficients: list[float] = [
            self.radial_distortion_coefficients[0],
            self.radial_distortion_coefficients[1],
            self.tangential_distortion_coefficients[0],
            self.tangential_distortion_coefficients[1]]
        coefficients += self.radial_distortion_coefficients[2:]
        return coefficients

    @staticmethod
    def generate_zero_parameters(
        resolution_x_px: int,
        resolution_y_px: int,
        fov_x_degrees: float = 45.0,
        fov_y_degrees: float = 45.0
    ) -> "IntrinsicParameters":
        optical_center_x_px: int = int(round(resolution_x_px/2.0))
        fov_x_radians: float = fov_x_degrees * math.pi / 180.0
        focal_length_x_px = (resolution_x_px / 2.0) / math.tan(fov_x_radians / 2.0)
        optical_center_y_px: int = int(round(resolution_y_px/2.0))
        fov_y_radians: float = fov_y_degrees * math.pi / 180.0
        focal_length_y_px = (resolution_y_px / 2.0) / math.tan(fov_y_radians / 2.0)
        return IntrinsicParameters(
            focal_length_x_px=focal_length_x_px,
            focal_length_y_px=focal_length_y_px,
            optical_center_x_px=optical_center_x_px,
            optical_center_y_px=optical_center_y_px,
            radial_distortion_coefficients=[0.0, 0.0, 0.0],
            tangential_distortion_coefficients=[0.0, 0.0])
