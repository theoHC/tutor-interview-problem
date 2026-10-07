"""Queue-based task allocation.

Loop: take the robot whose frontier (the timestep at which its last planned task finishes) is smallest, give it a
task, plan that task with cooperative A* against everything already reserved, commit, repeat.

Before the loop every robot docks one pallet of a different top-demand SKU and carries it for the whole run. Between
tasks it trails directly below the robot: that still lets the robot stand on the fulfilment row and at every
pallet's side access cell.

Task choice for that robot:
  * ORDER     - among the first `window` orders in the queue that current/incoming stock can satisfy, take the one
                whose estimated travel (picks from the carried pallet, then a greedy nearest-pallet tour, ending on
                the fulfilment row) is shortest.
  * REPLENISH - a trip to the replenishment row. It always refills the carried pallet, and may also take up to two
                static pallets (one per side). Triggered when the carried pallet can't cover the chosen order, or
                when *no* order in the queue can be satisfied; in that case a seed pallet is picked that most reduces
                the estimated movement cost of the orders at the queue head. Extra pallets are then added greedily
                by `ALPHA * share of remaining demand refilled - marginal trip length`, subject to the caps below.
"""

from collections import Counter, deque
from enum import Enum, auto
from itertools import permutations

from coopastar import NoPath, plan
from navigation import FULFILL_ROW, HEIGHT, REPLENISH_ROW, ReservationTable, StaticMap, bfs
from objects import Action, Coords, Pallet, Robot

UNMET_PENALTY = 1000  # estimated cost per item an order cannot currently get (used when ranking replenishments)
IDLE_STEP = 10  # how far a robot's frontier advances when it has nothing it can do right now

BELOW = (0, 1)  # carried pallet's offset during orders
ABOVE = (0, -1)  # ...and on replenishment trips: the robot must stand on row 39, and both side slots stay free
SIDES = ((1, 0), (-1, 0))
FLIP_SPAN = 64  # max timesteps a flip may leave the carried pallet standing alone

# Batching extra pallets onto a replenishment trip. A candidate's score is ALPHA * (share of the SKU's remaining
# demand that refilling it would newly cover) - (marginal trip length in steps). Caps, all of which must pass:
ALPHA = 300  # steps of detour that refilling 100% of a SKU's remaining demand is worth
MAX_DETOUR = 60  # never add more than this many estimated steps for one extra pallet
MAX_FILL = 0.5  # ignore pallets fuller than this fraction: refilling them gains little and they won't block soon
FUTURE_TRIP = 1.0  # extra's detour must be <= this * (the dedicated trip it saves * chance that trip is ever needed)
MIN_ITEMS_PER_STEP = 1.0  # extra must newly cover at least this many items of demand per step of detour
END_WEIGHT = 0.5  # how much the trip's end position (distance back to the fulfilment row) counts in its length


Foot = list[tuple[int, Coords]]  # (entity id, offset from robot); robot first at (0, 0)


def add(a: Coords, b: Coords) -> Coords:
    return a[0] + b[0], a[1] + b[1]


def sub(a: Coords, b: Coords) -> Coords:
    return a[0] - b[0], a[1] - b[1]


class TaskType(Enum):
    ORDER = auto()
    REPLENISH = auto()
    CARRY = auto()


# (ORDER, tuple of SKUs), (REPLENISH, (pallets to dock in order, fallback if that can't be planned)) or (CARRY, pallet)
Task = tuple[TaskType, object]

# A tentative plan is committed only once every stage of it has been found:
# (entity id, cell, first layer, last layer or None for "forever"), and (robot timestep, action, coords)
Reservation = tuple[int, Coords, int, int | None]


class Manager:
    def __init__(self, path: str, window: int = 10, carry: bool = True, alpha: float = ALPHA,
                 max_detour: float = MAX_DETOUR, max_fill: float = MAX_FILL, future_trip: float = FUTURE_TRIP,
                 min_items_per_step: float = MIN_ITEMS_PER_STEP):
        self.entities: list[Robot | Pallet] = []  # index == id
        self.tasks: deque[Task] = deque()
        self.window = window
        self.carry = carry
        self.alpha, self.max_detour, self.max_fill = alpha, max_detour, max_fill
        self.future_trip, self.min_items_per_step = future_trip, min_items_per_step
        self._load(path)
        self.reservations = ReservationTable(self.entities)
        self.map = StaticMap(self.pallets)
        self.by_sku: dict[int, list[Pallet]] = {}
        for p in self.pallets:
            self.by_sku.setdefault(p.sku, []).append(p)
        self.demand = Counter(sku for _, skus in self.tasks for sku in skus)  # items still to be planned, per SKU
        self._cell_bfs: dict[Coords, object] = {}
        self._spot_bfs: dict[tuple, object] = {}
        self.trips = 0
        self.replenishments = 0  # pallets refilled (the carried one included)

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
            self.entities.append(Pallet(sku=sku, pos=(x, y), planned_count=capacity[sku], max_count=capacity[sku]))
        for _ in range(int(next(lines)[0])):
            self.tasks.append((TaskType.ORDER, tuple(map(int, next(lines)))))
        assert all(e.id == i for i, e in enumerate(self.entities))

    @property
    def robots(self) -> list[Robot]:
        return [e for e in self.entities if isinstance(e, Robot)]

    @property
    def pallets(self) -> list[Pallet]:
        return [e for e in self.entities if isinstance(e, Pallet)]

    def _foot(self, robot: Robot, off: Coords = BELOW) -> Foot:
        return [(robot.id, (0, 0))] + ([(robot.carry, off)] if robot.carry is not None else [])

    # ------------------------------------------------------------------ main loop

    def solve(self, verbose: bool = True) -> int:
        robots = self.robots
        if self.carry:
            self.assign_carries()
        done = 0
        while self.tasks:
            robot = min(robots, key=lambda r: r.frontier[-1][1])
            task = self.select_task(robot)
            ok = False
            if task is not None:
                ok = self.execute(robot, task)
            if not ok:
                # Nothing doable (or no path found): wait in place and let other robots move the world forward.
                inv, t = robot.frontier[-1]
                robot.frontier.append((inv, t + IDLE_STEP))
                continue
            if task[0] is TaskType.ORDER:
                self.tasks.remove(task)
                self.demand.subtract(task[1])
                done += 1
                if verbose and done % 50 == 0:
                    print(f"  {done} orders planned, frontier t={robot.frontier[-1][1]}, "
                          f"trips={self.trips}, replenishments={self.replenishments}")
        return self.makespan

    @property
    def makespan(self) -> int:
        return max((t for r in self.robots for t, _, _ in r.actions), default=-1) + 1

    def assign_carries(self) -> None:
        """Give each robot a pallet of a different top-demand SKU, minimising the total walk to pick them up."""
        robots = self.robots
        top = [sku for sku, _ in self.demand.most_common(len(robots))]
        best = None
        for perm in permutations(top):
            picks = [min(self.by_sku[s], key=lambda p: self.map.dist_to_pallet(p.id, r.pos)) for r, s in zip(robots, perm)]
            cost = sum(self.map.dist_to_pallet(p.id, r.pos) for r, p in zip(robots, picks))
            if best is None or cost < best[0]:
                best = (cost, picks)
        for r, p in zip(robots, best[1]):
            if not self.execute(r, (TaskType.CARRY, p)):
                print(f"  robot {r.id} could not pick up pallet {p.id} (sku {p.sku}); it carries nothing")

    # ------------------------------------------------------------------ task selection

    def stock(self, robot: Robot | None = None) -> Counter:
        """Items per SKU that `robot` can reach (or will, once in-flight replenishments land): static pallets plus
        its own carried one. robot=None counts every pallet."""
        s = Counter()
        for p in self.pallets:
            if robot is None or p.carrier is None or p.carrier == robot.id:
                s[p.sku] += p.planned_count
        return s

    def select_task(self, robot: Robot) -> Task | None:
        stock = self.stock(robot)
        candidates = []
        for task in self.tasks:
            need = Counter(task[1])
            if all(stock[sku] >= n for sku, n in need.items()):
                candidates.append(task)
                if len(candidates) == self.window:
                    break
        carried = self.entities[robot.carry] if robot.carry is not None else None
        if candidates:
            t0 = robot.frontier[-1][1]
            # shortest estimated path wins
            task = min(candidates, key=lambda task: self.route(Counter(task[1]), robot.pos, t0, robot=robot)[1])
            if carried is not None and task[1].count(carried.sku) > carried.planned_count:
                return self.replenish_task(robot, None)
            return task
        pallet = self.choose_replenishment(robot)
        if pallet is None and (carried is None or carried.planned_count == carried.max_count):
            return None
        return self.replenish_task(robot, pallet)

    def route(self, need: Counter, start: Coords | None, t0: int = 0, override: dict | None = None,
              robot: Robot | None = None):
        """Picks from `robot`'s carried pallet, then a greedy nearest-pallet tour collecting the rest of `need`,
        finishing on the fulfilment row. Other robots' carried pallets are never used.

        start=None means "starting from the fulfilment row". `override` maps pallet id -> pretend available count.
        Returns (stops [(pallet, qty)], travel estimate, unmet item count). Waiting for a pallet that is still out
        being replenished is counted as travel time.
        """
        need = dict(need)
        stops, travel, cur, t = [], 0, None, t0
        if robot is not None and robot.carry is not None:
            c = self.entities[robot.carry]
            qty = min(need.get(c.sku, 0), override.get(c.id, c.planned_count) if override else c.planned_count)
            if qty > 0:
                stops.append((c, qty))
                t += qty
                need[c.sku] -= qty
                if need[c.sku] == 0:
                    del need[c.sku]
        avail = {}
        for sku in need:
            for p in self.by_sku.get(sku, ()):
                if p.carrier is not None:
                    continue
                a = override.get(p.id, p.planned_count) if override else p.planned_count
                if a > 0:
                    avail[p.id] = a
        while avail:
            best, best_cost = None, None
            for pid, a in avail.items():
                if cur is None:
                    d = self.map.from_top[pid] if start is None else self.map.dist_to_pallet(pid, start)
                else:
                    d = self.map.between[cur][pid]
                d = max(d, self.entities[pid].locked_until + 1 - t)
                if best_cost is None or d < best_cost:
                    best, best_cost = pid, d
            p = self.entities[best]
            qty = min(need[p.sku], avail.pop(best))
            stops.append((p, qty))
            travel += best_cost
            t += best_cost + qty
            cur = best
            need[p.sku] -= qty
            if need[p.sku] == 0:
                del need[p.sku]
                for q in self.by_sku[p.sku]:
                    avail.pop(q.id, None)
        if cur is not None:
            travel += self.map.from_top[cur]
        return stops, travel, sum(need.values())

    def choose_replenishment(self, robot: Robot) -> Pallet | None:
        """Pick the static pallet whose refill (together with the robot's carried one, which every trip refills)
        most reduces the estimated movement cost of the orders at the queue head."""
        head = [Counter(task[1]) for task, _ in zip(self.tasks, range(self.window))]
        stock = self.stock(robot)
        short = {sku for need in head for sku, n in need.items() if stock[sku] < n}
        wanted = short or {sku for need in head for sku in need}
        candidates = [
            p for sku in wanted for p in self.by_sku[sku]
            if p.carrier is None and p.planned_count < p.max_count and p.locked_until < self.min_frontier
        ]
        if not candidates:
            return None
        base = {robot.carry: self.entities[robot.carry].max_count} if robot.carry is not None else {}

        def cost(override):
            total = 0
            for need in head:
                _, travel, unmet = self.route(need, None, override=override, robot=robot)
                total += travel + UNMET_PENALTY * unmet
            return total

        return min(candidates, key=lambda p: cost({**base, p.id: p.max_count}))

    @property
    def min_frontier(self) -> int:
        return min(r.frontier[-1][1] for r in self.robots)

    # ------------------------------------------------------------------ batching pallets onto a replenishment trip

    def refill_gain(self, p: Pallet, stock: Counter) -> int:
        """Items of the SKU's remaining demand that refilling `p` newly covers (0 if stock already covers it all)."""
        return min(p.max_count - p.planned_count, max(0, self.demand[p.sku] - stock[p.sku]))

    def dedicated_trip(self, p: Pallet) -> float:
        """Estimated length of a trip from the fulfilment row that refills only `p` (as `trip_cost` measures it)."""
        m, side = self.map, self.map.side[p.id]
        return m.from_top[p.id] + 2 * m.to_bottom[p.id] + int(m.to_parking[side]) + END_WEIGHT * int(m.to_fulfill[side])

    def extra_ok(self, p: Pallet, gain: int, marginal: float) -> bool:
        """Caps an extra pallet must pass on top of a positive score. If the detour is free, accept it."""
        if marginal <= 0:
            return True
        # 4. it must cost less than the dedicated trip it would save, discounted by the chance that trip is ever
        #    needed: certain if the remaining shortfall uses the whole refill, proportionally less if not.
        needed = gain / (p.max_count - p.planned_count)
        # 5. measured in items, not demand share: share/detour >= k would just restate score > 0 with k = 1/ALPHA
        return (marginal <= self.max_detour and marginal <= self.future_trip * needed * self.dedicated_trip(p)
                and gain / marginal >= self.min_items_per_step)

    def trip_cost(self, start: Coords, seq: list[Pallet]) -> float:
        """Estimated steps for: dock `seq` in order, reach row 39, return them in reverse order, park. Plus a
        END_WEIGHT share of the walk from where the trip ends back to the fulfilment row."""
        m = self.map
        if not seq:
            return int(m.to_replenish[start]) + END_WEIGHT * (HEIGHT - 1)
        inner = sum(m.between[a.id][b.id] for a, b in zip(seq, seq[1:])) + m.to_bottom[seq[-1].id]
        end = m.side[seq[0].id]
        return m.dist_to_pallet(seq[0].id, start) + 2 * inner + int(m.to_parking[end]) + END_WEIGHT * int(m.to_fulfill[end])

    def replenish_task(self, robot: Robot, seed: Pallet | None) -> Task | None:
        """Build a trip around `seed` (or around just the carried pallet), adding extras while they score well."""
        seq = [seed] if seed is not None else []
        if not seq and robot.carry is None:
            return None
        fallback = list(seq)
        # the carried pallet rides above the robot, so the two side slots are free; the seed takes one of them
        # (with no carried pallet, the seed and extras still only use the side slots)
        slots = set(SIDES) - {self.map.side_off[p.id] for p in seq}
        t0 = robot.frontier[-1][1]
        stock = self.stock()
        base = self.trip_cost(robot.pos, seq)
        while slots:
            best = None
            for q in self.pallets:
                if (q in seq or q.carrier is not None or q.locked_until >= self.min_frontier or q.last_pick >= t0
                        or self.map.side_off[q.id] not in slots or q.planned_count > self.max_fill * q.max_count):
                    continue
                gain = self.refill_gain(q, stock)
                if gain <= 0:
                    continue
                share = gain / self.demand[q.sku]
                for i in range(len(seq) + 1):
                    trial = seq[:i] + [q] + seq[i:]
                    marginal = self.trip_cost(robot.pos, trial) - base
                    score = self.alpha * share - marginal
                    if score > 0 and self.extra_ok(q, gain, marginal) and (best is None or score > best[0]):
                        best = (score, trial, q)
            if best is None:
                break
            _, seq, q = best
            slots.discard(self.map.side_off[q.id])
            base = self.trip_cost(robot.pos, seq)
            stock[q.sku] += q.max_count - q.planned_count  # a second pallet of this SKU only scores what's still short
        return TaskType.REPLENISH, (tuple(seq), tuple(fallback))

    # ------------------------------------------------------------------ planning

    def execute(self, robot: Robot, task: Task) -> bool:
        """Plan `task` for `robot` from its frontier; commit reservations and actions only if every stage succeeds."""
        rt = self.reservations
        inv, t0 = robot.frontier[-1]
        foot = self._foot(robot)
        for _, off in foot:
            rt.unpark(*add(robot.pos, off), t0)
        if task[0] is TaskType.ORDER:
            attempts = [lambda: self._plan_order(robot, Counter(task[1]), t0)]
        elif task[0] is TaskType.CARRY:
            attempts = [lambda: self._plan_carry(robot, task[1], t0)]
        else:
            seq, fallback = task[1]
            seqs = [seq] if seq == fallback else [seq, fallback]
            attempts = [lambda s=s: self._plan_replenish(robot, list(s), t0) for s in seqs]
        for attempt in attempts:
            try:
                result = attempt()
                break
            except NoPath:
                continue
        else:
            for eid, off in foot:
                rt.park(*add(robot.pos, off), t0, eid)
            return False
        reservations, actions, frontiers, end_pos, commit = result
        commit()  # task-specific bookkeeping (stock, locks, freeing a docked pallet's home cell)
        for eid, (x, y), a, b in reservations:
            if b is None:
                rt.park(x, y, a, eid)
            elif b >= a:
                rt.reserve(x, y, a, b, eid)
        robot.actions.extend(actions)
        robot.frontier.extend(frontiers)
        robot.pos = end_pos
        return True

    def _heuristic_cell(self, target: Coords):
        if target not in self._cell_bfs:
            self._cell_bfs[target] = bfs(self.map.blocked, [target])
        return self._cell_bfs[target]

    def _walk(self, foot: Foot, pos, t, goal, hold, h):
        """One A* leg for the robot and everything docked to it. Returns (path, arrival time)."""
        group = tuple(e for e, _ in foot)
        offsets = tuple(o for _, o in foot)
        path = plan(self.reservations, pos, t, group, offsets, goal, hold, lambda c: h.item(c))
        return path, t + len(path) - 1

    @staticmethod
    def _moves(path, t, actions, reservations, foot: Foot):
        """Emit a `move` per changed cell and reserve every footprint cell for every layer of the path."""
        for i in range(1, len(path)):
            if path[i] != path[i - 1]:
                actions.append((t + i - 1, Action.MOVE, path[i]))
        for i, pos in enumerate(path):
            for eid, off in foot:
                reservations.append((eid, add(pos, off), t + i, t + i))

    @staticmethod
    def _hold(reservations, foot: Foot, pos: Coords, a: int, b: int | None):
        for eid, off in foot:
            reservations.append((eid, add(pos, off), a, b))

    def _flip(self, foot: Foot, pos: Coords, t: int, to_off: Coords, actions, reservations, final: bool = False):
        """Move the carried pallet (foot[1]) to another side of the robot: drag it to an open spot, undock, walk
        round it and dock again. final=True: end on a parking cell and stay there indefinitely.
        Returns (new footprint, robot position, next free timestep)."""
        rt = self.reservations
        (rid, _), (cid, off) = foot
        key = (off, to_off, final)
        if key not in self._spot_bfs:
            m = self.map
            ok = lambda c: m.open3[add(c, off)] and (not final or m.parking[sub(add(c, off), to_off)])
            cells = [(x, y) for x in range(m.open3.shape[0]) for y in range(m.open3.shape[1])
                     if 0 <= x + off[0] < m.open3.shape[0] and 0 <= y + off[1] < m.open3.shape[1] and ok((x, y))]
            self._spot_bfs[key] = (ok, bfs(m.blocked, cells))
        ok, h = self._spot_bfs[key]
        path, ta = self._walk(foot, pos, t, ok, lambda ta: ta + 1, h)
        self._moves(path, t, actions, reservations, foot)
        c = path[-1]
        pallet = add(c, off)
        target = sub(pallet, to_off)
        actions.append((ta, Action.UNDOCK, pallet))
        reservations.append((rid, c, ta, ta + 1))
        if not rt.free_for(*pallet, ta, None if final else ta + FLIP_SPAN, (cid,)):
            raise NoPath("flip: pallet cell not free")
        # the robot alone must walk round its (now standing) pallet, so block that cell while searching
        rt.extend(ta + FLIP_SPAN)
        saved = rt.table[pallet[0], pallet[1], ta : ta + FLIP_SPAN].copy()
        rt.table[pallet[0], pallet[1], ta : ta + FLIP_SPAN] = cid
        try:
            hold = (lambda tb: None) if final else (lambda tb: tb + 1)
            rob = [(rid, (0, 0))]
            walk, tb = self._walk(rob, c, ta + 1, lambda x: x == target, hold, self._heuristic_cell(target))
        finally:
            rt.table[pallet[0], pallet[1], ta : ta + FLIP_SPAN] = saved
        if tb >= ta + FLIP_SPAN - 2:
            raise NoPath("flip: took too long")
        self._moves(walk, ta + 1, actions, reservations, rob)
        actions.append((tb, Action.DOCK, pallet))
        reservations.append((cid, pallet, ta, None if final else tb + 1))
        reservations.append((rid, target, tb, None if final else tb + 1))
        return [(rid, (0, 0)), (cid, to_off)], target, tb + 1

    def _plan_carry(self, robot: Robot, pallet: Pallet, t0: int):
        """Fetch the pallet this robot will carry for the whole run, then park with it docked below."""
        actions, reservations, frontiers = [], [], []
        rid, side = robot.id, self.map.side[pallet.id]
        rob = [(rid, (0, 0))]
        path, ta = self._walk(rob, robot.pos, t0, lambda c: c == side, lambda ta: ta + 1, self._heuristic_cell(side))
        self._moves(path, t0, actions, reservations, rob)
        actions.append((ta, Action.DOCK, pallet.pos))
        reservations.append((rid, side, ta, ta + 1))
        foot = rob + [(pallet.id, self.map.side_off[pallet.id])]
        foot, pos, t = self._flip(foot, side, ta + 1, BELOW, actions, reservations, final=True)
        frontiers.append(((), t))

        def commit():
            self.reservations.unpark(*pallet.pos, ta + 1)
            pallet.carrier = rid
            robot.carry = pallet.id

        return reservations, actions, frontiers, pos, commit

    def _plan_order(self, robot: Robot, need: Counter, t0: int):
        stops, _, unmet = self.route(need, robot.pos, t0, robot=robot)
        assert unmet == 0
        actions, reservations, frontiers = [], [], []
        foot = self._foot(robot)
        pos, t, inv = robot.pos, t0, []
        for p, qty in stops:
            if p.carrier == robot.id:
                # pick from the carried pallet in place (we start parked, so the cells are ours)
                self._hold(reservations, foot, pos, t, t + qty)
                for k in range(qty):
                    actions.append((t + k, Action.PICK, add(pos, BELOW)))
                t += qty
            else:
                access = set(self.map.access[p.id])
                # occupy the access cell from arrival until the pallet is available, then for `qty` picks
                hold = lambda ta, p=p, qty=qty: max(ta, p.locked_until + 1) + qty
                path, ta = self._walk(foot, pos, t, access.__contains__, hold, self.map.to_pallet[p.id])
                self._moves(path, t, actions, reservations, foot)
                pos, start = path[-1], max(ta, p.locked_until + 1)
                self._hold(reservations, foot, pos, ta, start + qty)
                for k in range(qty):
                    actions.append((start + k, Action.PICK, p.pos))
                t = start + qty
            inv += [p.sku] * qty
            frontiers.append((tuple(sorted(inv)), t))
        # head for the fulfilment row and stay there (parked) until the next task is planned
        path, ta = self._walk(foot, pos, t, lambda c: c[1] == FULFILL_ROW, lambda ta: None, self.map.to_fulfill)
        self._moves(path, t, actions, reservations, foot)
        self._hold(reservations, foot, path[-1], ta, None)
        actions.append((ta, Action.FULFILL, path[-1]))
        frontiers.append(((), ta + 1))

        def commit():
            # stock is only deducted once the whole plan is known to be feasible
            for p, qty in stops:
                p.planned_count -= qty
                if p.carrier is None:
                    p.last_pick = max(p.last_pick, max(t for t, a, c in actions if a is Action.PICK and c == p.pos))

        return reservations, actions, frontiers, path[-1], commit

    def _plan_replenish(self, robot: Robot, seq: list[Pallet], t0: int):
        """Trip to the replenishment row: flip the carried pallet above the robot, dock `seq` in order (each from
        its side access cell, into the slot that leaves it at its home offset), refill, then undock them in reverse
        order. Reversing makes every undock footprint identical to the matching dock footprint, which was valid.
        Finally flip the carried pallet back below and park."""
        actions, reservations, frontiers = [], [], []
        rid = robot.id
        foot = self._foot(robot)
        pos, t = robot.pos, t0

        # 1. if the first pallet still has picks reserved far ahead, wait here (parked, safe) rather than squatting
        #    in the aisle, which both blocks other robots and makes the space-time search explode.
        if seq:
            t = max(t0, seq[0].last_pick + 1 - self.map.dist_to_pallet(seq[0].id, pos))
            self._hold(reservations, foot, pos, t0, t)
        if robot.carry is not None:
            foot, pos, t = self._flip(foot, pos, t, ABOVE, actions, reservations)

        # 2. collect each pallet; dock only after the last pick already reserved on it has happened
        docked = []
        for q in seq:
            side = self.map.side[q.id]
            dock_at = lambda ta, q=q: max(ta, q.last_pick + 1)
            path, ta = self._walk(foot, pos, t, lambda c, s=side: c == s, lambda ta, d=dock_at: d(ta) + 1,
                                  self._heuristic_cell(side))
            self._moves(path, t, actions, reservations, foot)
            td = dock_at(ta)
            self._hold(reservations, foot, side, ta, td + 1)
            actions.append((td, Action.DOCK, q.pos))
            foot = foot + [(q.id, self.map.side_off[q.id])]
            docked.append((q, td))
            pos, t = side, td + 1

        # 3. drag everything to the replenishment row. The refill fires at the end of the timestep in which the
        #    robot moves onto row 39, so it can turn straight round on the next timestep.
        path, ta = self._walk(foot, pos, t, lambda c: c[1] == REPLENISH_ROW, lambda ta: ta, self.map.to_replenish)
        self._moves(path, t, actions, reservations, foot)
        pos, t = path[-1], ta

        # 4. put each pallet back in its home slot, last docked first
        returned = []
        for q, td in reversed(docked):
            side = self.map.side[q.id]
            path, ta = self._walk(foot, pos, t, lambda c, s=side: c == s, lambda ta: ta + 1, self._heuristic_cell(side))
            self._moves(path, t, actions, reservations, foot)
            actions.append((ta, Action.UNDOCK, q.pos))
            self._hold(reservations, foot, side, ta, ta + 1)
            reservations.append((q.id, q.pos, ta, None))
            foot = [f for f in foot if f[0] != q.id]
            returned.append((q, td, ta))
            pos, t = side, ta + 1
        frontiers.append(((), t))

        # 5. clear the aisle: park somewhere open so this robot never blocks another robot's access cell
        if robot.carry is not None:
            foot, pos, t = self._flip(foot, pos, t, BELOW, actions, reservations, final=True)
        else:
            path, ta = self._walk(foot, pos, t, lambda c: self.map.parking[c], lambda ta: None, self.map.to_parking)
            self._moves(path, t, actions, reservations, foot)
            self._hold(reservations, foot, path[-1], ta, None)
            pos, t = path[-1], ta
        frontiers.append(((), t))

        def commit():
            # each home cell is free while its pallet is out; the reservations above re-park it there on return.
            # Other robots may not pick from it until it is back (locked_until), by which time it is full again.
            for q, td, tu in returned:
                self.reservations.unpark(*q.pos, td + 1)
                q.planned_count = q.max_count
                q.locked_until = tu
            if robot.carry is not None:
                self.entities[robot.carry].planned_count = self.entities[robot.carry].max_count
            self.trips += 1
            self.replenishments += len(returned) + (robot.carry is not None)

        return reservations, actions, frontiers, pos, commit

    # ------------------------------------------------------------------ output

    def write_solution(self, path: str) -> None:
        lines = []
        for idx, r in enumerate(self.robots):
            for t, action, (x, y) in r.actions:
                lines.append((t, idx, action.name.lower(), x, y))
        lines.sort()
        with open(path, "w") as f:
            for t, idx, a, x, y in lines:
                f.write(f"{t} {idx} {a} {x} {y}\n")
