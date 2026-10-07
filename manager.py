from collections import deque
from enum import Enum, auto

from navigation import ReservationTable
from objects import Pallet, Robot


class TaskType(Enum):
    ORDER = auto()
    REPLENISH = auto()


# (ORDER, tuple of SKUs) or (REPLENISH, pallet)
Task = tuple[TaskType, tuple[int, ...] | Pallet]


class Manager:
    def __init__(self, path: str):
        self.entities: list[Robot | Pallet] = []  # index == id
        self.tasks: deque[Task] = deque()
        self._load(path)
        self.reservations = ReservationTable(self.entities)

    def _load(self, path: str) -> None:
        with open(path) as f:
            tokens = [line.split() for line in f.read().splitlines()]
        lines = iter(tokens)
        for _ in range(int(next(lines)[0])):
            x, y = map(int, next(lines))
            self.entities.append(Robot(pos=(x, y)))
        capacity = [int(next(lines)[0]) for _ in range(int(next(lines)[0]))]
        for _ in range(int(next(lines)[0])):
            x, y, sku = map(int, next(lines))
            self.entities.append(Pallet(sku=sku, pos=(x, y), count=capacity[sku], max_count=capacity[sku]))
        for _ in range(int(next(lines)[0])):
            self.tasks.append((TaskType.ORDER, tuple(map(int, next(lines)))))
        assert all(e.id == i for i, e in enumerate(self.entities))

    @property
    def robots(self) -> list[Robot]:
        return [e for e in self.entities if isinstance(e, Robot)]

    @property
    def pallets(self) -> list[Pallet]:
        return [e for e in self.entities if isinstance(e, Pallet)]
