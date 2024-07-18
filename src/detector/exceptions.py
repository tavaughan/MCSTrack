from src.common.exceptions import MCTError


class MCTDetectorRuntimeError(MCTError):
    message: str

    def __init__(self, *args, message: str):
        super().__init__(*args)
        self.message = message
