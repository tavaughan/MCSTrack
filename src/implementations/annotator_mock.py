from src.common import \
    Annotator, \
    Annotation, \
    KeyValueMetaAny, \
    KeyValueSimpleAny, \
    MCTAnnotatorRuntimeError, \
    StatusMessageSource
import datetime
import numpy


class MockAnnotator(Annotator):
    """
    The Mock classes are relatively simple implementations made for testing connectivity functionality.
    """

    _update_timestamp_utc: datetime.datetime
    _annotations_detected: list[Annotation]
    _annotations_rejected: list[Annotation]

    def __init__(
        self,
        configuration: Annotator.Configuration,
        status_message_source: StatusMessageSource
    ):
        super().__init__(
            configuration=configuration,
            status_message_source=status_message_source)
        self._update_timestamp_utc = datetime.datetime.min.replace(tzinfo=datetime.timezone.utc)
        self._annotations_detected = list()
        self._annotations_rejected = list()
        self.set_status(Annotator.Status.RUNNING)

    def get_changed_timestamp(self) -> datetime.datetime:
        return self._update_timestamp_utc

    def get_markers_detected(self) -> list[Annotation]:
        return self._annotations_detected

    def get_markers_rejected(self) -> list[Annotation]:
        return self._annotations_rejected

    def get_parameters(self) -> list[KeyValueMetaAny]:
        return list()

    @staticmethod
    def get_type_identifier() -> str:
        return "mock"

    # noinspection DuplicatedCode
    def set_parameters(
        self,
        parameters: list[KeyValueSimpleAny]
    ) -> None:
        mismatched_keys: list[str] = [parameter.key for parameter in parameters]
        if len(mismatched_keys) > 0:
            raise MCTAnnotatorRuntimeError(
                message=f"The following parameters could not be applied due to key mismatch: {str(mismatched_keys)}")

    def update(
        self,
        image: numpy.ndarray
    ) -> None:
        half_height_px: int = image.shape[0] // 2
        half_width_px: int = image.shape[1] // 2
        self._annotations_detected = [
            Annotation(
                feature_label=f"Mock{Annotation.RELATION_CHARACTER}{number}",
                x_px=(half_width_px if 1 <= number <= 2 else 0),
                y_px=(number//2) * half_height_px)
            for number in range(0,4)]
        self._annotations_rejected = [
            Annotation(
                feature_label=f"{Annotation.UNIDENTIFIED_LABEL}{Annotation.RELATION_CHARACTER}{number}",
                x_px=(half_width_px if 1 <= number <= 2 else 0) + half_width_px,
                y_px=(number//2) * half_height_px + half_height_px)
            for number in range(0,4)]
        self._update_timestamp_utc = datetime.datetime.now(tz=datetime.timezone.utc)
