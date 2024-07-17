from pydantic import BaseModel, Field


class ImageResolution(BaseModel):
    x_px: int = Field()
    y_px: int = Field()

    def __eq__(self, other) -> bool:
        if type(self) is not type(other):
            return False
        return \
            self.x_px == other.x_px and \
            self.y_px == other.y_px

    def __hash__(self) -> int:
        return hash(str(self))

    def __lt__(self, other):
        if not isinstance(other, ImageResolution):
            raise ValueError()
        if self.x_px < other.x_px:
            return True
        elif self.x_px > other.x_px:
            return False
        elif self.y_px < other.y_px:
            return True
        else:
            return False

    def __str__(self):
        return f"{self.x_px}x{self.y_px}"

    @staticmethod
    def from_str(in_str: str) -> 'ImageResolution':
        if 'x' not in in_str:
            raise ValueError("in_str is expected to contain delimiter 'x'.")
        parts: list[str] = in_str.split('x')
        if len(parts) > 2:
            raise ValueError("in_str is expected to contain exactly one 'x'.")
        x_px = int(parts[0])
        y_px = int(parts[1])
        return ImageResolution(x_px=x_px, y_px=y_px)
