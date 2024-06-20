from .mcast_error import MCastError


class ParsingError(MCastError):
    message: str

    def __init__(
        self,
        message: str,
        *args
    ):
        super().__init__(args)
        self.message = message
