from collections import deque

import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import shortest_path

from objects import Coords, Pallet, Robot

WIDTH, HEIGHT = 60, 40
EMPTY = -1
FULFILL_ROW = 0
REPLENISH_ROW = HEIGHT - 1
NEIGHBOURS = ((1, 0), (-1, 0), (0, 1), (0, -1))
CHUNK = 1024  # time layers added per extension (amortises np.concatenate cost)


def in_bounds(x: int, y: int) -> bool:
    return 0 <= x < WIDTH and 0 <= y < HEIGHT


def adjacent(c: Coords) -> list[Coords]:
    x, y = c
    return [(x + dx, y + dy) for dx, dy in NEIGHBOURS if in_bounds(x + dx, y + dy)]


class ReservationTable:
    """Space-time occupation table indexed [x, y, t]; each cell holds an object ID or EMPTY.

    Layer t is the world state at the *start* of timestep t (i.e. after the actions of timestep t-1).
    `permanent` is a 2D layer of things that stay put indefinitely (resting pallets, parked robots); it is
    what new time layers are initialised from and what lookups beyond the horizon fall back to.
    """

    def __init__(self, entities: list[Robot | Pallet], width: int = WIDTH, height: int = HEIGHT):
        self.permanent = np.full((width, height), EMPTY, dtype=np.int16)
        for e in entities:
            x, y = e.pos
            self.permanent[x, y] = e.id
        self.table = np.repeat(self.permanent[:, :, None], CHUNK, axis=2)

    @property
    def horizon(self) -> int:
        return self.table.shape[2]

    def extend(self, t: int) -> None:
        """Grow the time axis so timestep t is valid, filling new layers from the permanent layer."""
        if self.horizon <= t:
            n = ((t - self.horizon) // CHUNK + 1) * CHUNK
            self.table = np.concatenate([self.table, np.repeat(self.permanent[:, :, None], n, axis=2)], axis=2)

    def get(self, x: int, y: int, t: int) -> int:
        if t < self.horizon:
            return self.table.item(x, y, t)
        return self.permanent.item(x, y)

    def free_for(self, x: int, y: int, t0: int, t1: int | None, ids: tuple[int, ...]) -> bool:
        """True if (x, y) is EMPTY or owned by one of `ids` for every t in [t0, t1]; t1=None means forever."""
        if t1 is None:
            if self.permanent.item(x, y) not in (EMPTY, *ids):
                return False
            t1 = self.horizon - 1
        span = self.table[x, y, t0 : t1 + 1]
        ok = span == EMPTY
        for i in ids:
            ok |= span == i
        if not ok.all():
            return False
        return t1 < self.horizon or self.permanent.item(x, y) in (EMPTY, *ids)

    def reserve(self, x: int, y: int, t0: int, t1: int, eid: int) -> None:
        """Mark (x, y) as owned by eid for every t in [t0, t1]."""
        self.extend(t1)
        self.table[x, y, t0 : t1 + 1] = eid

    def park(self, x: int, y: int, t: int, eid: int) -> None:
        """eid occupies (x, y) from timestep t onwards, indefinitely (until `unpark`)."""
        self.extend(t)
        self.table[x, y, t:] = eid
        self.permanent[x, y] = eid

    def unpark(self, x: int, y: int, t: int) -> None:
        """Release an indefinite occupation of (x, y) from timestep t onwards."""
        self.extend(t)
        self.table[x, y, t:] = EMPTY
        self.permanent[x, y] = EMPTY


def bfs(blocked: np.ndarray, sources: list[Coords]) -> np.ndarray:
    """Static shortest-path distances (ignoring other robots and time) from a set of source cells."""
    dist = np.full((WIDTH, HEIGHT), 10**6, dtype=np.int32)
    q = deque()
    for s in sources:
        dist[s] = 0
        q.append(s)
    while q:
        x, y = q.popleft()
        d = dist[x, y] + 1
        for nx, ny in adjacent((x, y)):
            if not blocked[nx, ny] and dist[nx, ny] > d:
                dist[nx, ny] = d
                q.append((nx, ny))
    return dist


# Rows pallets may be slotted into (in their original columns): two blocks with a 3-row gap (y=17-19) between them
TOP_BLOCK = range(3, 17)
BOTTOM_BLOCK = range(20, 33)
SLOT_ROWS = (*TOP_BLOCK, *BOTTOM_BLOCK)


class StaticMap:
    """Distance lookups keyed by *slot* (a cell a pallet may stand in), robots ignored.

    Pallets are re-slotted when replenished, so every table is keyed by cell rather than by pallet, and `relayout`
    recomputes the distances whenever the set of occupied slots changes.
    Used both as A* heuristics and for cheap route-cost estimates during task selection.
    """

    def __init__(self, pallets: list[Pallet], reslot: bool = True):
        columns = sorted({p.pos[0] for p in pallets})
        if reslot:
            self.slots: list[Coords] = [(x, y) for y in SLOT_ROWS for x in columns]
        else:  # pallets only ever stand where they started
            self.slots = sorted({p.pos for p in pallets}, key=lambda s: (s[1], s[0]))
        self.by_row: dict[int, list[Coords]] = {}
        for s in self.slots:
            self.by_row.setdefault(s[1], []).append(s)
        assert all(p.pos in set(self.slots) for p in pallets), "a pallet starts outside the slot grid"
        self.slot = np.zeros((WIDTH, HEIGHT), dtype=bool)
        for s in self.slots:
            self.slot[s] = True
        # cells a robot can stand on to pick from / dock to a pallet in each slot: never another slot, as that may
        # hold a pallet by the time the robot gets there
        self.access: dict[Coords, list[Coords]] = {s: [c for c in adjacent(s) if not self.slot[c]] for s in self.slots}
        # side access: slots stand in 2-wide column pairs, so each has exactly one access cell level with it.
        # Docking from there puts the pallet on the robot's west or east side, and it keeps that offset, so a
        # pallet can only ever be re-slotted into a column on the same side of its pair.
        self.side = {s: next(c for c in self.access[s] if c[1] == s[1]) for s in self.slots}
        self.side_off = {s: (s[0] - self.side[s][0], 0) for s in self.slots}
        # parking: open cells well away from any slot so a resting robot never blocks an access cell
        self.parking = np.zeros((WIDTH, HEIGHT), dtype=bool)
        for x in range(WIDTH):
            for y in range(HEIGHT - 1):
                lo_x, hi_x, lo_y, hi_y = max(0, x - 3), x + 4, max(0, y - 3), y + 4
                self.parking[x, y] = not self.slot[lo_x:hi_x, lo_y:hi_y].any()
        # cells whose whole 3x3 neighbourhood is open, away from the parked rows at the edges: a robot can undock a
        # pallet here and walk round it to re-dock from another side
        self.open3 = np.zeros((WIDTH, HEIGHT), dtype=bool)
        for x in range(1, WIDTH - 1):
            for y in range(3, HEIGHT - 2):
                self.open3[x, y] = not self.slot[x - 1 : x + 2, y - 1 : y + 2].any()
        # every access cell once, grouped by slot (for the all-sources BFS in `relayout`)
        self._acc = [c for s in self.slots for c in self.access[s]]
        self._acc_start = np.cumsum([0] + [len(self.access[s]) for s in self.slots[:-1]])
        self.relayout({p.pos for p in pallets})

    def relayout(self, occupied: set[Coords]) -> None:
        """Recompute every distance table with exactly the slots in `occupied` blocked."""
        self.blocked = np.zeros((WIDTH, HEIGHT), dtype=bool)
        for c in occupied:
            self.blocked[c] = True
        # unweighted 4-connected grid graph over open cells; node id = x * HEIGHT + y (ravel order of [x, y] arrays)
        idx = np.arange(WIDTH * HEIGHT).reshape(WIDTH, HEIGHT)
        free = ~self.blocked
        h = free[:-1, :] & free[1:, :]
        v = free[:, :-1] & free[:, 1:]
        src = np.concatenate([idx[:-1, :][h], idx[:, :-1][v]])
        dst = np.concatenate([idx[1:, :][h], idx[:, 1:][v]])
        graph = csr_matrix((np.ones(len(src)), (src, dst)), shape=(WIDTH * HEIGHT, WIDTH * HEIGHT))
        acc = [x * HEIGHT + y for x, y in self._acc]
        d = shortest_path(graph, directed=False, unweighted=True, indices=acc)
        d = np.where(np.isinf(d), 10**6, d).astype(np.int32)
        # nearest access cell of each slot, from every cell
        per_slot = np.minimum.reduceat(d, self._acc_start, axis=0)  # (slots, cells)
        self.to_pallet = {s: per_slot[i].reshape(WIDTH, HEIGHT) for i, s in enumerate(self.slots)}
        # slot-to-slot travel: from the best access cell of a to the nearest access cell of b
        between = np.minimum.reduceat(per_slot[:, acc], self._acc_start, axis=1).T.tolist()
        self.between = {a: dict(zip(self.slots, row)) for a, row in zip(self.slots, between)}
        self.to_fulfill = bfs(self.blocked, [(x, FULFILL_ROW) for x in range(WIDTH)])
        self.to_replenish = bfs(self.blocked, [(x, REPLENISH_ROW) for x in range(WIDTH)])
        self.to_parking = bfs(self.blocked, list(zip(*np.nonzero(self.parking))))
        # fulfilment row -> slot travel, and slot -> replenishment row
        self.from_top = {a: min(int(self.to_fulfill[c]) for c in self.access[a]) for a in self.slots}
        self.to_bottom = {a: min(int(self.to_replenish[c]) for c in self.access[a]) for a in self.slots}

    def dist_to_pallet(self, slot: Coords, c: Coords) -> int:
        return int(self.to_pallet[slot][c])
