from src.common import \
    client_identifier_from_connection, \
    EmptyResponse, \
    ErrorResponse
from src.detector import \
    Detector, \
    DetectorConfiguration
from src.detector.api import \
    CalibrationResultGetActiveResponse, \
    CameraImageGetRequest, \
    CameraImageGetResponse, \
    CameraParametersGetResponse, \
    DetectorFrameGetRequest, \
    DetectorFrameGetResponse, \
    MarkerParametersGetResponse, \
    MarkerParametersSetRequest
from src.detector.interfaces import \
    AbstractCamera, \
    AbstractMarker
import base64
from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.websockets import WebSocket
from fastapi_utils.tasks import repeat_every
import hjson
import logging
import os


logger = logging.getLogger(__name__)


def create_app() -> FastAPI:
    detector_configuration_filepath: str = \
        os.path.join(os.path.dirname(__file__), "..", "..", "data", "detector_config.json")
    detector_configuration: DetectorConfiguration
    with open(detector_configuration_filepath, 'r') as infile:
        detector_configuration_file_contents: str = infile.read()
        detector_configuration_dict = hjson.loads(detector_configuration_file_contents)
        detector_configuration = DetectorConfiguration(**detector_configuration_dict)

    # Eventually it would be preferable to put the initialization logic/mapping below into an abstract factory,
    # and allow end-users to register custom classes that are not necessarily shipped within this library.

    camera_type: type[AbstractCamera]
    if detector_configuration.camera_configuration.driver == "opencv_capture_device":
        from src.detector.implementations.camera_opencv_capture_device import OpenCVCaptureDeviceCamera
        camera_type = OpenCVCaptureDeviceCamera
    elif detector_configuration.camera_configuration.driver == "picamera2":
        from src.detector.implementations.camera_picamera2 import Picamera2Camera
        camera_type = Picamera2Camera
    else:
        raise RuntimeError(f"Unsupported camera driver {detector_configuration.camera_configuration.driver}.")

    marker_type: type[AbstractMarker]
    if detector_configuration.marker_configuration.method == "aruco_opencv":
        from src.detector.implementations.marker_aruco_opencv import ArucoOpenCVMarker
        marker_type = ArucoOpenCVMarker
    else:
        raise RuntimeError(f"Unsupported marker method {detector_configuration.marker_configuration.method}.")
    
    detector = Detector(
        detector_configuration=detector_configuration,
        camera_type=camera_type,
        marker_type=marker_type)
    detector_app = FastAPI()

    # CORS Middleware
    origins = ["http://localhost"]
    detector_app.add_middleware(
        CORSMiddleware,
        allow_origins=origins,
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"])

    @detector_app.head("/detector/start")
    async def detector_start(
        http_request: Request
    ) -> None:
        client_identifier: str = client_identifier_from_connection(connection=http_request)
        detector.detector_start(client_identifier=client_identifier)

    @detector_app.head("/detector/stop")
    async def detector_stop(
        http_request: Request
    ) -> None:
        client_identifier: str = client_identifier_from_connection(connection=http_request)
        detector.detector_stop(client_identifier=client_identifier)

    @detector_app.post("/detector/get_frame")
    async def detector_get_frame(
        request: DetectorFrameGetRequest
    ) -> DetectorFrameGetResponse:
        return detector.detector_frame_get(
            request=request)

    @detector_app.get("/calibration/get_result_active")
    async def calibration_get_result_active() -> CalibrationResultGetActiveResponse:
        return detector.calibration_result_get_active()

    @detector_app.get("/camera/get_image")
    async def camera_get_image() -> CameraImageGetResponse:
        result: CameraImageGetResponse = detector.camera_image_get(
            request=CameraImageGetRequest(format=".png"))
        image_bytes = base64.b64decode(result.image_base64)
        with open("test.png", "wb") as image_file:
            image_file.write(image_bytes)
        return result

    @detector_app.get("/camera/get_parameters")
    async def camera_get_parameters() -> CameraParametersGetResponse:
        result: CameraParametersGetResponse = detector.camera_parameters_get()
        return result

    @detector_app.get("/marker/get_parameters")
    async def marker_get_parameters() -> MarkerParametersGetResponse | ErrorResponse:
        return detector.marker_parameters_get()

    @detector_app.post("/marker/set_parameters")
    async def marker_set_parameters(
        request: MarkerParametersSetRequest
    ) -> EmptyResponse | ErrorResponse:
        return detector.marker_parameters_set(
            request=request)

    @detector_app.websocket("/websocket")
    async def websocket_handler(websocket: WebSocket) -> None:
        await detector.websocket_handler(websocket=websocket)

    @detector_app.on_event("startup")
    @repeat_every(seconds=0.001)
    async def internal_update() -> None:
        await detector.update()

    return detector_app


app = create_app()
