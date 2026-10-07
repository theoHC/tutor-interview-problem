import numpy as np

from objects import Pallet, Robot

WIDTH, HEIGHT = 60, 40
EMPTY = -1


class ReservationTable:
    """Space-time occupation table indexed [x, y, t]; each cell holds an object ID or EMPTY."""

    def __init__(self, entities: list[Robot | Pallet], width: int = WIDTH, height: int = HEIGHT):
        self.table = np.full((width, height, 1), EMPTY, dtype=np.int32)
        for e in entities:
            x, y = e.pos
            self.table[x, y, 0] = e.id

    @property
    def horizon(self) -> int:
        return self.table.shape[2]

    def extend(self, t: int) -> None:
        """Grow the time axis so timestep t is valid, copying the last layer forward."""
        while self.horizon <= t:
            self.table = np.concatenate([self.table, self.table[:, :, -1:]], axis=2)
