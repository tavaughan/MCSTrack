from ..exceptions import MCTDetectorRuntimeError
from ..interfaces import AbstractMarker
from ..structures import \
    MarkerConfiguration, \
    MarkerStatus
from src.common import StatusMessageSource
from src.common.structures import \
    CornerRefinementMethod, \
    CORNER_REFINEMENT_METHOD_DICTIONARY_INT_TO_TEXT, \
    CORNER_REFINEMENT_METHOD_DICTIONARY_TEXT_TO_INT, \
    DetectionParameters, \
    MarkerSnapshot, \
    MarkerCornerImagePoint
import cv2.aruco
import datetime
import logging
import numpy
from typing import Any


logger = logging.getLogger(__name__)


class ArucoOpenCVMarker(AbstractMarker):
    _marker_dictionary: Any | None  # created by OpenCV, type cv2.aruco.Dictionary
    _marker_parameters: Any  # created by OpenCV, type cv2.aruco.DetectorParameters
    _marker_label_reverse_dictionary: dict[int, str]
    _marker_detected_snapshots: list[MarkerSnapshot]
    _marker_rejected_snapshots: list[MarkerSnapshot]
    _marker_timestamp_utc: datetime.datetime

    def __init__(
        self,
        configuration: MarkerConfiguration,
        status_message_source: StatusMessageSource
    ):
        super().__init__(
            configuration=configuration,
            status_message_source=status_message_source)

        self._marker_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        self._marker_parameters = cv2.aruco.DetectorParameters_create()
        self._marker_label_reverse_dictionary = dict()
        self._marker_detected_snapshots = list()  # Markers that are determined to be valid, and are identified
        self._marker_rejected_snapshots = list()  # Things that looked at first like markers but got later filtered out
        self._marker_timestamp_utc = datetime.datetime.min
        self.set_status(MarkerStatus.STOPPED)

        # TODO: DEBUGGING
        self.set_status(MarkerStatus.RUNNING)

    def get_changed_timestamp(self) -> datetime.datetime:
        return self._marker_timestamp_utc

    def get_markers_detected(self) -> list[MarkerSnapshot]:
        return self._marker_detected_snapshots

    def get_markers_rejected(self) -> list[MarkerSnapshot]:
        return self._marker_rejected_snapshots

    def get_parameters(self) -> DetectionParameters:
        if self._marker_parameters.cornerRefinementMethod not in CORNER_REFINEMENT_METHOD_DICTIONARY_INT_TO_TEXT:
            raise MCTDetectorRuntimeError(
                message=f"Corner refinement method appears to be set to an invalid value: "
                        f"{self._marker_parameters.corner_refinement_method}.")
        corner_refinement_method_text: CornerRefinementMethod = \
            CORNER_REFINEMENT_METHOD_DICTIONARY_INT_TO_TEXT[self._marker_parameters.cornerRefinementMethod]
        params: DetectionParameters = DetectionParameters(
            adaptive_thresh_constant=self._marker_parameters.adaptiveThreshConstant,
            adaptive_thresh_win_size_max=self._marker_parameters.adaptiveThreshWinSizeMax,
            adaptive_thresh_win_size_min=self._marker_parameters.adaptiveThreshWinSizeMin,
            adaptive_thresh_win_size_step=self._marker_parameters.adaptiveThreshWinSizeStep,
            april_tag_critical_rad=self._marker_parameters.aprilTagCriticalRad,
            april_tag_deglitch=self._marker_parameters.aprilTagDeglitch,
            april_tag_max_line_fit_mse=self._marker_parameters.aprilTagMaxLineFitMse,
            april_tag_max_nmaxima=self._marker_parameters.aprilTagMaxNmaxima,
            april_tag_min_cluster_pixels=self._marker_parameters.aprilTagMinClusterPixels,
            april_tag_min_white_black_diff=self._marker_parameters.aprilTagMinWhiteBlackDiff,
            april_tag_quad_decimate=self._marker_parameters.aprilTagQuadDecimate,
            april_tag_quad_sigma=self._marker_parameters.aprilTagQuadSigma,
            corner_refinement_max_iterations=self._marker_parameters.cornerRefinementMaxIterations,
            corner_refinement_method=corner_refinement_method_text,
            corner_refinement_min_accuracy=self._marker_parameters.cornerRefinementMinAccuracy,
            corner_refinement_win_size=self._marker_parameters.cornerRefinementWinSize,
            detect_inverted_marker=self._marker_parameters.detectInvertedMarker,
            error_correction_rate=self._marker_parameters.errorCorrectionRate,
            marker_border_bits=self._marker_parameters.markerBorderBits,
            max_erroneous_bits_in_border_rate=self._marker_parameters.maxErroneousBitsInBorderRate,
            max_marker_perimeter_rate=self._marker_parameters.maxMarkerPerimeterRate,
            min_corner_distance_rate=self._marker_parameters.minCornerDistanceRate,
            min_distance_to_border=self._marker_parameters.minDistanceToBorder,
            min_marker_distance_rate=self._marker_parameters.minMarkerDistanceRate,
            min_marker_perimeter_rate=self._marker_parameters.minMarkerPerimeterRate,
            min_otsu_std_dev=self._marker_parameters.minOtsuStdDev,
            perspective_remove_ignored_margin_per_cell=self._marker_parameters.perspectiveRemoveIgnoredMarginPerCell,
            perspective_remove_pixel_per_cell=self._marker_parameters.perspectiveRemovePixelPerCell,
            polygonal_approx_accuracy_rate=self._marker_parameters.polygonalApproxAccuracyRate)
        return params

    @staticmethod
    def get_type_identifier() -> str:
        return "aruco_opencv"

    @staticmethod
    def _marker_corner_image_point_list_from_embedded_list(
        corner_image_points_px: list[list[float]]
    ) -> list[MarkerCornerImagePoint]:
        corner_image_point_list: list[MarkerCornerImagePoint] = list()
        assert len(corner_image_points_px) == 4
        for corner_image_point_px in corner_image_points_px:
            corner_image_point_list.append(MarkerCornerImagePoint(
                x_px=corner_image_point_px[0],
                y_px=corner_image_point_px[1]))
        return corner_image_point_list

    # noinspection DuplicatedCode
    def set_parameters(
        self,
        parameters: DetectionParameters
    ) -> None:

        p: DetectionParameters = parameters

        if p.adaptive_thresh_win_size_min:
            self._marker_parameters.adaptiveThreshWinSizeMin = p.adaptive_thresh_win_size_min
        if p.adaptive_thresh_win_size_max:
            self._marker_parameters.adaptiveThreshWinSizeMax = p.adaptive_thresh_win_size_max
        if p.adaptive_thresh_win_size_step:
            self._marker_parameters.adaptiveThreshWinSizeStep = p.adaptive_thresh_win_size_step
        if p.adaptive_thresh_constant:
            self._marker_parameters.adaptiveThreshConstant = p.adaptive_thresh_constant

        if p.min_marker_perimeter_rate:
            self._marker_parameters.minMarkerPerimeterRate = p.min_marker_perimeter_rate
        if p.max_marker_perimeter_rate:
            self._marker_parameters.maxMarkerPerimeterRate = p.max_marker_perimeter_rate
        if p.polygonal_approx_accuracy_rate:
            self._marker_parameters.polygonalApproxAccuracyRate = p.polygonal_approx_accuracy_rate
        if p.min_corner_distance_rate:
            self._marker_parameters.minCornerDistanceRate = p.min_corner_distance_rate
        if p.min_marker_distance_rate:
            self._marker_parameters.minMarkerDistanceRate = p.min_marker_distance_rate
        if p.min_distance_to_border:
            self._marker_parameters.minDistanceToBorder = p.min_distance_to_border

        if p.corner_refinement_max_iterations:
            self._marker_parameters.cornerRefinementMaxIterations = p.corner_refinement_max_iterations
        if p.corner_refinement_method:
            if p.corner_refinement_method in CORNER_REFINEMENT_METHOD_DICTIONARY_TEXT_TO_INT:
                self._marker_parameters.cornerRefinementMethod = \
                    CORNER_REFINEMENT_METHOD_DICTIONARY_TEXT_TO_INT[p.corner_refinement_method]
            else:
                raise MCTDetectorRuntimeError(
                    message=f"Failed to find corner refinement method {p.corner_refinement_method}.")
        if p.corner_refinement_min_accuracy:
            self._marker_parameters.cornerRefinementMinAccuracy = p.corner_refinement_min_accuracy
        if p.corner_refinement_win_size:
            self._marker_parameters.cornerRefinementWinSize = p.corner_refinement_win_size

        if p.marker_border_bits:
            self._marker_parameters.markerBorderBits = p.marker_border_bits
        if p.min_otsu_std_dev:
            self._marker_parameters.minOtsuStdDev = p.min_otsu_std_dev
        if p.perspective_remove_pixel_per_cell:
            self._marker_parameters.perspectiveRemovePixelPerCell = p.perspective_remove_pixel_per_cell
        if p.perspective_remove_ignored_margin_per_cell:
            self._marker_parameters.perspectiveRemoveIgnoredMarginPerCell = \
                p.perspective_remove_ignored_margin_per_cell

        if p.max_erroneous_bits_in_border_rate:
            self._marker_parameters.maxErroneousBitsInBorderRate = p.max_erroneous_bits_in_border_rate
        if p.error_correction_rate:
            self._marker_parameters.errorCorrectionRate = p.error_correction_rate
        if p.detect_inverted_marker:
            self._marker_parameters.detectInvertedMarker = p.detect_inverted_marker

        if p.april_tag_critical_rad:
            self._marker_parameters.aprilTagCriticalRad = p.april_tag_critical_rad
        if p.april_tag_deglitch:
            self._marker_parameters.aprilTagDeglitch = p.april_tag_deglitch
        if p.april_tag_max_line_fit_mse:
            self._marker_parameters.aprilTagMaxLineFitMse = p.april_tag_max_line_fit_mse
        if p.april_tag_max_nmaxima:
            self._marker_parameters.aprilTagMaxNmaxima = p.april_tag_max_nmaxima
        if p.april_tag_min_cluster_pixels:
            self._marker_parameters.aprilTagMinClusterPixels = p.april_tag_min_cluster_pixels
        if p.april_tag_min_white_black_diff:
            self._marker_parameters.aprilTagMinWhiteBlackDiff = p.april_tag_min_white_black_diff
        if p.april_tag_quad_decimate:
            self._marker_parameters.aprilTagQuadDecimate = p.april_tag_quad_decimate
        if p.april_tag_quad_sigma:
            self._marker_parameters.aprilTagQuadSigma = p.april_tag_quad_sigma

        # Note: a relatively recent addition to OpenCV, may not be available in some python versions
        if hasattr(self._marker_parameters, "useAruco3Detection"):
            if p.use_aruco_3_detection:
                self._marker_parameters.useAruco3Detection = p.use_aruco_3_detection
            if p.min_side_length_canonical_img:
                self._marker_parameters.minSideLengthCanonicalImg = p.min_side_length_canonical_img
            if p.min_marker_length_ratio_original_img:
                self._marker_parameters.minMarkerLengthRatioOriginalImg = p.min_marker_length_ratio_original_img

    def update(
        self,
        image: numpy.ndarray
    ) -> None:
        if self._marker_dictionary is None:
            message: str = "No marker dictionary has been set."
            self.add_status_message(severity="error", message=message)
            self.set_status(MarkerStatus.FAILURE)
            return

        image_greyscale = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        (detected_corner_points_raw, detected_dictionary_indices, rejected_corner_points_raw) = cv2.aruco.detectMarkers(
            image=image_greyscale,
            dictionary=self._marker_dictionary,
            parameters=self._marker_parameters)

        self._marker_detected_snapshots = list()
        # note: detected_indices is (inconsistently) None sometimes if no markers are detected
        if detected_dictionary_indices is not None and len(detected_dictionary_indices) > 0:
            detected_marker_count = detected_dictionary_indices.size
            # Shape of some output was previously observed to (also) be inconsistent... make it consistent here:
            detected_corner_points_px = numpy.array(detected_corner_points_raw).reshape((detected_marker_count, 4, 2))
            detected_dictionary_indices = list(detected_dictionary_indices.reshape(detected_marker_count))
            for detected_marker_index, detected_marker_id in enumerate(detected_dictionary_indices):
                if False:  # TODO: Re-enable
                    if detected_marker_id not in self._marker_label_reverse_dictionary:
                        message: str = \
                            f"Found a marker with index {detected_marker_id} "\
                            "but it does not appear in the dictionary."
                        self.add_status_message(severity="error", message=message)
                        self.set_status(MarkerStatus.FAILURE)
                        return
                    marker_label: str = self._marker_label_reverse_dictionary[detected_marker_id]
                else:
                    marker_label: str = str(detected_marker_id)
                corner_image_points_px = detected_corner_points_px[detected_marker_index]
                corner_image_points: list[MarkerCornerImagePoint] = \
                    self._marker_corner_image_point_list_from_embedded_list(
                        corner_image_points_px=corner_image_points_px.tolist())
                self._marker_detected_snapshots.append(MarkerSnapshot(
                    label=marker_label,
                    corner_image_points=corner_image_points))

        self._marker_rejected_snapshots = list()
        if rejected_corner_points_raw:
            rejected_corner_points_px = numpy.array(rejected_corner_points_raw).reshape((-1, 4, 2))
            for rejected_marker_index in range(rejected_corner_points_px.shape[0]):
                corner_image_points_px = rejected_corner_points_px[rejected_marker_index]
                corner_image_points: list[MarkerCornerImagePoint] = \
                    self._marker_corner_image_point_list_from_embedded_list(
                        corner_image_points_px=corner_image_points_px.tolist())
                self._marker_rejected_snapshots.append(MarkerSnapshot(
                    label=f"unknown",
                    corner_image_points=corner_image_points))

        self._marker_timestamp_utc = datetime.datetime.utcnow()
