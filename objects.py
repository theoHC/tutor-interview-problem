from dataclasses import dataclass, field
from enum import Enum, auto

Coords = tuple[int, int]

_next_id = 0


def new_id() -> int:
    global _next_id
    i = _next_id
    _next_id += 1
    return i


class Action(Enum):
    MOVE = auto()
    PICK = auto()
    DOCK = auto()
    UNDOCK = auto()
    FULFILL = auto()


# (timestep, action, optional coords)
PlannedAction = tuple[int, Action, Coords | None]

# (inventory as of latest planned state, timestep); inventory is a sorted tuple of SKUs
Frontier = tuple[tuple[int, ...], int]


@dataclass
class Pallet:
    sku: int
    pos: Coords
    planned_count: int # this is the anticipated count AFTER all currently reserved picks happen
    max_count: int
    id: int = field(default_factory=new_id)
    locked_until: int = -1 # when being carried like for restocking, this is set to the planned undocking timestep.
    last_pick: int = -1 # latest timestep at which any robot has a pick reserved from this pallet


@dataclass
class Robot:
    pos: Coords
    carried: list[int] = field(default_factory=list)
    id: int = field(default_factory=new_id)
    actions: list[PlannedAction] = field(default_factory=list)
    frontier: list[Frontier] = field(default_factory=lambda: [((), 0)])
