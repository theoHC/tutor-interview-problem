"""Cooperative (space-time) A*.

Robots are planned one at a time against the ReservationTable, which already contains every earlier robot's
reserved trajectory, so a new plan can never collide with an existing one. States are (x, y, t); each step is
either a move to a 4-neighbour or a wait, both costing one timestep.

A plan may move a *footprint*: the robot plus any pallets docked to it, given as fixed offsets from the robot.
"""

import heapq
from typing import Callable

from navigation import EMPTY, NEIGHBOURS, ReservationTable, in_bounds
from objects import Coords

MAX_EXPANSIONS = 200_000


class NoPath(Exception):
    pass


def plan(
    rt: ReservationTable,
    start: Coords,
    t0: int,
    group: tuple[int, ...],
    offsets: tuple[Coords, ...],
    is_goal: Callable[[Coords], bool],
    hold: Callable[[int], int | None],
    heuristic: Callable[[Coords], int],
) -> list[Coords]:
    """Find the earliest-arriving path for `group` (robot id first, then docked pallet ids).

    `offsets` are the footprint cells relative to the robot ((0, 0) first, then one per docked pallet).
    On reaching a goal cell at time t the footprint must then be able to stay put until `hold(t)` (None = forever),
    which is how picking, docking and parking durations are folded into the search.

    Returns the robot position for every timestep from t0 to the arrival time inclusive.
    """
    allowed = (EMPTY, *group)

    def cells_ok(x: int, y: int, t: int) -> bool:
        # Conservative "no following" rule, enforced in both directions: occupying a cell at layer t+1 needs it free
        # of other entities at layers t, t+1 and t+2. So we never enter a cell someone is leaving in that timestep,
        # and nobody enters a cell we are leaving. Then we never depend on how the simulator orders moves.
        for dx, dy in offsets:
            cx, cy = x + dx, y + dy
            if not in_bounds(cx, cy):
                return False
            for k in (0, 1, 2):
                if rt.get(cx, cy, t + k) not in allowed:
                    return False
        return True

    def can_wait(x: int, y: int, t: int) -> bool:
        for dx, dy in offsets:
            if rt.get(x + dx, y + dy, t + 1) not in allowed or rt.get(x + dx, y + dy, t + 2) not in allowed:
                return False
        return True

    def can_hold(x: int, y: int, t: int) -> bool:
        t_end = hold(t)
        t_end = None if t_end is None else t_end + 1  # see cells_ok: the layer after we leave must be free too
        return all(rt.free_for(x + dx, y + dy, t, t_end, group) for dx, dy in offsets)

    open_heap = [(heuristic(start), -t0, start, t0)]
    parent: dict[tuple[Coords, int], tuple[Coords, int] | None] = {(start, t0): None}
    expansions = 0
    while open_heap:
        _, _, pos, t = heapq.heappop(open_heap)
        if is_goal(pos) and can_hold(*pos, t):
            path = []
            node = (pos, t)
            while node is not None:
                path.append(node[0])
                node = parent[node]
            return path[::-1]
        expansions += 1
        if expansions > MAX_EXPANSIONS:
            break
        x, y = pos
        for dx, dy in ((0, 0), *NEIGHBOURS):
            nxt = (x + dx, y + dy)
            key = (nxt, t + 1)
            if key in parent:
                continue
            if dx == dy == 0:
                ok = can_wait(x, y, t)
            else:
                ok = cells_ok(*nxt, t)
            if ok:
                parent[key] = (pos, t)
                heapq.heappush(open_heap, (t + 1 + heuristic(nxt), -(t + 1), nxt, t + 1))
    raise NoPath(f"no path for {group} from {start}@{t0}")
