from src.common import \
    client_identifier_from_connection, \
    EmptyResponse, \
    ErrorResponse
from src.detector import \
    Detector, \
    DetectorConfiguration
from src.detector.api import \
    CameraImageGetRequest, \
    CameraImageGetResponse, \
    CameraParametersGetResponse, \
    DetectorFrameGetRequest, \
    DetectorFrameGetResponse, \
    MarkerParametersGetResponse, \
    MarkerParametersSetRequest
from src.detector.structures.detector_configuration import OPENCV, PICAMERA
from src.detector.interfaces import \
    AbstractCameraInterface, \
    AbstractMarkerInterface
from src.detector.implementations.aruco_marker_implementation import ArucoMarker
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

    camera_interface: AbstractCameraInterface
    if detector_configuration.camera_implementation == OPENCV:
        from src.detector.implementations.usb_webcam_implementation import USBWebcamWithOpenCV
        camera_interface = USBWebcamWithOpenCV(detector_configuration.camera_connection.usb_id)
    elif detector_configuration.camera_implementation == PICAMERA:
        from src.detector.implementations.picamera2_implementation import PiCamera
        camera_interface = PiCamera()
    else:
        raise RuntimeError(f"Unsupported AbstractCameraInterface {detector_configuration.camera_implementation}")

    marker_interface: AbstractMarkerInterface
    marker_interface = ArucoMarker()
    
    detector = Detector(
        detector_configuration=detector_configuration,
        marker_interface=marker_interface,
        camera_interface=camera_interface)
    detector_app = FastAPI()

    # CORS Middleware
    origins = ["http://localhost"]
    detector_app.add_middleware(
        CORSMiddleware,
        allow_origins=origins,
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"])

    @detector_app.get("/get_capture_image")
    async def get_capture_image() -> CameraImageGetResponse:
        result: CameraImageGetResponse = detector.get_capture_image(
            request=CameraImageGetRequest(format=".png"))
        image_bytes = base64.b64decode(result.image_base64)
        with open("test.png", "wb") as image_file:
            image_file.write(image_bytes)
        return result

    @detector_app.get("/get_capture_properties")
    async def get_capture_properties() -> CameraParametersGetResponse:
        result: CameraParametersGetResponse = detector.get_capture_properties()
        return result

    @detector_app.get("/get_detection_parameters")
    async def get_detection_parameters() -> MarkerParametersGetResponse | ErrorResponse:
        return detector.get_detection_parameters()

    @detector_app.post("/get_marker_snapshots")
    async def get_marker_snapshots(
        request: DetectorFrameGetRequest
    ) -> DetectorFrameGetResponse:
        return detector.get_marker_snapshots(
            request=request)

    @detector_app.post("/set_detection_parameters")
    async def set_detection_parameters(
        request: MarkerParametersSetRequest
    ) -> EmptyResponse | ErrorResponse:
        return detector.set_detection_parameters(
            request=request)

    @detector_app.head("/start_capture")
    async def start_capture(
        http_request: Request
    ) -> None:
        client_identifier: str = client_identifier_from_connection(connection=http_request)
        detector.start_capture(client_identifier=client_identifier)

    @detector_app.head("/stop_capture")
    async def stop_capture(
        http_request: Request
    ) -> None:
        client_identifier: str = client_identifier_from_connection(connection=http_request)
        detector.stop_capture(client_identifier=client_identifier)

    @detector_app.websocket("/websocket")
    async def websocket_handler(websocket: WebSocket) -> None:
        await detector.websocket_handler(websocket=websocket)

    @detector_app.on_event("startup")
    @repeat_every(seconds=0.001)
    async def internal_update() -> None:
        await detector.internal_update()

    return detector_app


app = create_app()
