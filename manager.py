"""Queue-based task allocation.

Loop: take the robot whose frontier (the timestep at which its last planned task finishes) is smallest, give it a
task, plan that task with cooperative A* against everything already reserved, commit, repeat.

Optionally (carry=True; off by default, as it measured slower) every robot first docks one pallet of a different
top-demand SKU and carries it for the whole run. Between tasks it trails directly below the robot: that still lets the robot stand on the fulfilment row and at every
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

from scipy.optimize import linear_sum_assignment

from coopastar import NoPath, plan
from navigation import FULFILL_ROW, HEIGHT, REPLENISH_ROW, SLOT_ROWS, ReservationTable, StaticMap, bfs
from objects import Action, Coords, Pallet, Robot

UNMET_PENALTY = 1000  # estimated cost per item an order cannot currently get (used when ranking replenishments)
IDLE_STEP = 10  # how far a robot's frontier advances when it has nothing it can do right now

BELOW = (0, 1)  # carried pallet's offset during orders
ABOVE = (0, -1)  # on replenishment trips it can't stay below (the robot must stand on row 39). Either it rides above,
#                  leaving both side slots for other pallets, or (trip_slot="side") it takes a side slot: half the
#                  walk round to flip it, but one slot fewer
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

# Presort phase (see `relocate_all`)
PAIR_MARGIN = 0  # a 2-pallet trip must be estimated at least this many steps shorter than doing the moves one by one

# Re-slotting a replenished pallet (see `choose_slot`)
MAX_SHIFT = 7  # columns of horizontal shift accepted without the detour-vs-savings test (7 = one column pair)
FILL_WEIGHT = 1.0  # steps saved per order served, per step a slot is closer to the fulfilment row


Foot = list[tuple[int, Coords]]  # (entity id, offset from robot); robot first at (0, 0)


def add(a: Coords, b: Coords) -> Coords:
    return a[0] + b[0], a[1] + b[1]


def sub(a: Coords, b: Coords) -> Coords:
    return a[0] - b[0], a[1] - b[1]


class TaskType(Enum):
    ORDER = auto()
    REPLENISH = auto()
    CARRY = auto()
    RELOCATE = auto()


# (ORDER, tuple of SKUs), (CARRY, pallet),
# (REPLENISH, (pallets to dock in order, fallback if that can't be planned, carried pallet's offset on the trip)) or
# (RELOCATE, options to try in turn, each (pallets to dock in order, (pallet, new slot) to undock in order))
Task = tuple[TaskType, object]

# A tentative plan is committed only once every stage of it has been found:
# (entity id, cell, first layer, last layer or None for "forever"), and (robot timestep, action, coords)
Reservation = tuple[int, Coords, int, int | None]


class Manager:
    def __init__(self, path: str, window: int = 10, carry: bool = False, alpha: float = ALPHA,
                 max_detour: float = MAX_DETOUR, max_fill: float = MAX_FILL, future_trip: float = FUTURE_TRIP,
                 min_items_per_step: float = MIN_ITEMS_PER_STEP, trip_slot: str = "above",
                 tour_mode: str = "best", rank_tour: bool = False, reslot: str = "off",
                 max_shift: float = MAX_SHIFT, fill_weight: float = FILL_WEIGHT, live_map: bool = True,
                 presort: bool = False, pair: bool = True, pair_margin: float = PAIR_MARGIN,
                 barrier: bool = False, rank_by: str = "visits", fill_order: str = "middle",
                 select: str = "raw", assign: str = "frontier", slack: int = 0, lag_weight: float = 1.0):
        self.entities: list[Robot | Pallet] = []  # index == id
        self.tasks: deque[Task] = deque()
        self.window = window
        self.carry = carry
        self.alpha, self.max_detour, self.max_fill = alpha, max_detour, max_fill
        self.future_trip, self.min_items_per_step = future_trip, min_items_per_step
        self.trip_slot = trip_slot
        self.tour_mode = tour_mode
        self.rank_tour = rank_tour
        self.reslot, self.max_shift, self.fill_weight = reslot, max_shift, fill_weight
        self.live_map = live_map  # recompute distance tables whenever a pallet changes slot (else: starting layout)
        self.presort, self.pair, self.pair_margin = presort, pair, pair_margin
        self.barrier = barrier  # no robot starts an order until every presort move has finished
        self.rank_by, self.fill_order = rank_by, fill_order  # target_layout variants
        self.select, self.assign, self.slack, self.lag_weight = select, assign, slack, lag_weight
        assert not (presort and carry), "presorting assumes no robot carries a pallet"
        self._load(path)
        self.reservations = ReservationTable(self.entities)
        self.map = StaticMap(self.pallets, reslot=reslot != "off" or presort)
        self.by_sku: dict[int, list[Pallet]] = {}
        for p in self.pallets:
            self.by_sku.setdefault(p.sku, []).append(p)
        self.demand = Counter(sku for _, skus in self.tasks for sku in skus)  # items still to be planned, per SKU
        self.visits = Counter(sku for _, skus in self.tasks for sku in set(skus))  # orders still to plan, per SKU
        self.target_row = self._target_rows()
        self._cell_bfs: dict[Coords, object] = {}
        self._spot_bfs: dict[tuple, object] = {}
        self.trips = 0
        self.replenishments = 0  # pallets refilled (the carried one included)
        self.moved = 0  # refills that put the pallet in a different slot from the one it came from
        self.relocations = 0  # pallets moved during the presort phase
        self.relocation_trips = 0
        self.presort_end = 0  # timestep by which every robot has finished the presort phase

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
        if self.presort:
            self.relocate_all(verbose)
        done = 0
        while self.tasks:
            robot = min(robots, key=lambda r: r.frontier[-1][1])
            if self.assign == "central":
                robot, task = self.select_joint(robot)
            else:
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
                self.visits.subtract(set(task[1]))
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
            picks = [min(self.by_sku[s], key=lambda p: self.map.dist_to_pallet(p.pos, r.pos)) for r, s in zip(robots, perm)]
            cost = sum(self.map.dist_to_pallet(p.pos, r.pos) for r, p in zip(robots, picks))
            if best is None or cost < best[0]:
                best = (cost, picks)
        for r, p in zip(robots, best[1]):
            if not self.execute(r, (TaskType.CARRY, p)):
                print(f"  robot {r.id} could not pick up pallet {p.id} (sku {p.sku}); it carries nothing")

    def _target_rows(self) -> dict[int, int]:
        """Row each SKU's pallets are aimed at when re-slotted. SKUs are ranked by how many orders use them (ties: by
        items). packed: walk down the slot rows, giving each SKU the row its pallets would land on if the blocks were
        filled in rank order. linear: spread the ranks evenly from the top allowable row to the bottom one."""
        if self.reslot == "off":
            return {}
        ranked = sorted(self.by_sku, key=lambda s: (-self.visits[s], -self.demand[s]))
        per_row = len(self.map.by_row[SLOT_ROWS[0]])
        rows, seen = {}, 0
        for i, sku in enumerate(ranked):
            if self.reslot == "linear":
                k = round(i * (len(SLOT_ROWS) - 1) / max(1, len(ranked) - 1))
            else:
                k = min(seen // per_row, len(SLOT_ROWS) - 1)
            rows[sku] = SLOT_ROWS[k]
            seen += len(self.by_sku[sku])
        return rows

    # ------------------------------------------------------------------ task selection

    def stock(self, robot: Robot | None = None) -> Counter:
        """Items per SKU that `robot` can reach (or will, once in-flight replenishments land): static pallets plus
        its own carried one. robot=None counts every pallet."""
        s = Counter()
        for p in self.pallets:
            if robot is None or p.carrier is None or p.carrier == robot.id:
                s[p.sku] += p.planned_count
        return s

    def _candidates(self, robot: Robot) -> list[Task]:
        stock = self.stock(robot)
        candidates = []
        for task in self.tasks:
            need = Counter(task[1])
            if all(stock[sku] >= n for sku, n in need.items()):
                candidates.append(task)
                if len(candidates) == self.window:
                    break
        return candidates

    def _score(self, task: Task, robot: Robot, t0: int, base: dict) -> float:
        """Lower is better. raw: estimated travel from the robot. marginal: that minus what the order would cost from
        the fulfilment row (the extra travel this robot's position adds). per-item: travel per item."""
        need = Counter(task[1])
        estimate = self.tour if self.rank_tour else lambda need, pos, t, r: self.route(need, pos, t, robot=r)
        travel = estimate(need, robot.pos, t0, robot)[1]
        if self.select == "marginal":
            if task not in base:
                base[task] = self.route(need, None)[1]
            return travel - base[task]
        if self.select == "per-item":
            return travel / len(task[1])
        return travel

    def select_joint(self, first: Robot) -> tuple[Robot, Task | None]:
        """Choose robot and order together: every robot within `slack` of the earliest frontier is scored against every
        candidate order; its lag behind the earliest robot (x lag_weight) is added to the score. When no order is
        feasible, fall back to the earliest robot's replenishment."""
        candidates = self._candidates(first)
        if not candidates:
            return first, self.select_task(first)
        tmin = first.frontier[-1][1]
        base: dict = {}
        best = None
        for r in self.robots:
            t0 = r.frontier[-1][1]
            if t0 - tmin > self.slack:
                continue
            for task in candidates:
                c = self._score(task, r, t0, base) + self.lag_weight * (t0 - tmin)
                if best is None or c < best[0]:
                    best = (c, r, task)
        return best[1], best[2]

    def select_task(self, robot: Robot) -> Task | None:
        candidates = self._candidates(robot)
        carried = self.entities[robot.carry] if robot.carry is not None else None
        if candidates:
            t0 = robot.frontier[-1][1]
            # lowest score wins (rank_tour: the tour that would actually be planned; slower)
            base: dict = {}
            task = min(candidates, key=lambda task: self._score(task, robot, t0, base))
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
        E, m = self.entities, self.map
        while avail:
            best, best_cost = None, None
            for pid, a in avail.items():
                s = E[pid].pos
                if cur is None:
                    d = m.from_top[s] if start is None else m.dist_to_pallet(s, start)
                else:
                    d = m.between[E[cur].pos][s]
                d = max(d, E[pid].locked_until + 1 - t)
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
            travel += m.from_top[E[cur].pos]
        return stops, travel, sum(need.values())

    # ------------------------------------------------------------------ aisle-aware S-shaped sweep

    def tour(self, need: Counter, start: Coords, t0: int, robot: Robot):
        """The tour actually planned for an order. `tour_mode` greedy: `route` as is. sweep: `route`'s pallets,
        then swap each SKU to whichever duplicate pallet shortens an S-shaped sweep, visited in sweep order.
        best: whichever of the two is shorter. Returns (stops, travel estimate, unmet), like `route`."""
        stops, travel, unmet = self.route(need, start, t0, robot=robot)
        if self.tour_mode == "greedy" or unmet:
            return stops, travel, unmet
        carried = [s for s in stops if s[0].carrier is not None]
        static = [s for s in stops if s[0].carrier is None]
        t1 = t0 + sum(q for _, q in carried)
        qty = {p.id: q for p, q in static}
        cost = lambda order: self._tour_cost(order, start, t1, qty)
        # only SKUs served from a single pallet are re-chosen; any pallet of the SKU with enough stock will do
        count = Counter(p.sku for p, _ in static)
        pick = {p.sku: p.id for p, _ in static if count[p.sku] == 1}
        fixed = [p.id for p, _ in static if count[p.sku] > 1]
        alts = {sku: [q.id for q in self.by_sku[sku] if q.carrier is None and q.planned_count >= qty[pid]]
                for sku, pid in pick.items()}
        order = self._sweep(fixed + list(pick.values()), cost)
        best = cost(order)
        improved = True
        while improved:
            improved = False
            for sku, options in alts.items():
                for alt in options:
                    old = pick[sku]
                    if alt == old:
                        continue
                    pick[sku] = alt
                    qty[alt] = qty[old]
                    trial = self._sweep(fixed + list(pick.values()), cost)
                    c = cost(trial)
                    if c < best:
                        best, order, improved = c, trial, True
                    else:
                        pick[sku] = old
        if self.tour_mode == "best" and travel <= best:
            return stops, travel, unmet
        return carried + [(self.entities[pid], qty[pid]) for pid in order], best, 0

    def _tour_cost(self, order: list[int], start: Coords | None, t: int, qty: dict[int, int]) -> int:
        """Travel for visiting `order` (pallet ids) then reaching the fulfilment row, counting waits for pallets
        still out being replenished, as `route` does."""
        if not order:
            return 0
        m, E, travel, cur = self.map, self.entities, 0, None
        for pid in order:
            s = E[pid].pos
            if cur is None:
                d = m.from_top[s] if start is None else m.dist_to_pallet(s, start)
            else:
                d = m.between[cur][s]
            d = max(d, E[pid].locked_until + 1 - t)
            travel += d
            t += d + qty[pid]
            cur = s
        return travel + m.from_top[cur]

    def _sweep(self, pids: list[int], cost) -> list[int]:
        """S-shaped sweep: each pick face (the column of access cells beside a pallet column) is a lane, walked
        top-to-bottom or bottom-to-top alternately, so consecutive lanes are joined at the band's end. The upper
        and lower bands are swept separately; the best of either band first and either horizontal direction wins."""
        m = self.map
        bands = ([p for p in pids if self.entities[p].pos[1] < HEIGHT // 2],
                 [p for p in pids if self.entities[p].pos[1] >= HEIGHT // 2])
        best, best_cost = None, None
        for first, second in (bands, bands[::-1]):
            n = len({m.side[self.entities[p].pos][0] for p in first})
            for lr1 in (True, False):
                for lr2 in (True, False):
                    order = self._serpentine(first, lr1, True) + self._serpentine(second, lr2, n % 2 == 0)
                    c = cost(order)
                    if best_cost is None or c < best_cost:
                        best, best_cost = order, c
        return best

    def _serpentine(self, pids: list[int], left_to_right: bool, first_down: bool) -> list[int]:
        side = self.map.side
        lanes: dict[int, list[int]] = {}
        for p in pids:
            lanes.setdefault(side[self.entities[p].pos][0], []).append(p)
        out = []
        for i, x in enumerate(sorted(lanes, reverse=not left_to_right)):
            lane = sorted(lanes[x], key=lambda p: self.entities[p].pos[1])
            out += lane if (i % 2 == 0) == first_down else lane[::-1]
        return out

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
        m, side = self.map, self.map.side[p.pos]
        return m.from_top[p.pos] + 2 * m.to_bottom[p.pos] + int(m.to_parking[side]) + END_WEIGHT * int(m.to_fulfill[side])

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
        inner = sum(m.between[a.pos][b.pos] for a, b in zip(seq, seq[1:])) + m.to_bottom[seq[-1].pos]
        end = m.side[seq[0].pos]
        return m.dist_to_pallet(seq[0].pos, start) + 2 * inner + int(m.to_parking[end]) + END_WEIGHT * int(m.to_fulfill[end])

    def replenish_task(self, robot: Robot, seed: Pallet | None) -> Task | None:
        """Build a trip around `seed` (or around just the carried pallet), adding extras while they score well."""
        seq = [seed] if seed is not None else []
        if not seq and robot.carry is None:
            return None
        free = set(SIDES) - {self.map.side_off[p.pos] for p in seq}  # the seed takes one side slot
        if robot.carry is None or self.trip_slot == "above":
            # extras only use the side slots (above needs an end-of-column pallet docked from below)
            return TaskType.REPLENISH, (tuple(self._add_extras(robot, seq, free)[0]), tuple(seq), ABOVE)
        # the carried pallet takes a side slot too: whichever leaves the better extra
        options = []
        for off in sorted(free):
            extended, score = self._add_extras(robot, seq, free - {off})
            options.append((score, extended, off))
        score, extended, off = max(options, key=lambda o: o[0])
        return TaskType.REPLENISH, (tuple(extended), tuple(seq), off)

    def _add_extras(self, robot: Robot, seq: list[Pallet], slots: set[Coords]) -> tuple[list[Pallet], float]:
        """Greedily insert extra pallets into the free `slots` while they score well and pass the caps.
        Returns the new docking order and the total score of what was added."""
        slots, total = set(slots), 0.0
        t0 = robot.frontier[-1][1]
        stock = self.stock()
        base = self.trip_cost(robot.pos, seq)
        while slots:
            best = None
            for q in self.pallets:
                if (q in seq or q.carrier is not None or q.locked_until >= self.min_frontier or q.last_pick >= t0
                        or self.map.side_off[q.pos] not in slots or q.planned_count > self.max_fill * q.max_count):
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
            score, seq, q = best
            total += score
            slots.discard(self.map.side_off[q.pos])
            base = self.trip_cost(robot.pos, seq)
            stock[q.sku] += q.max_count - q.planned_count  # a second pallet of this SKU only scores what's still short
        return seq, total

    # ------------------------------------------------------------------ presort: move every pallet to its slot first

    def target_layout(self) -> dict[int, Coords]:
        """The `packed` layout: SKUs ranked by how many orders use them, their pallets filling the slot rows top
        down, middle columns of a row first. A pallet keeps its side of a column pair (it is docked from the aisle
        and keeps that offset). Cells robots start on are left empty. Within each row the pallets bound for it are
        then matched to its slots so as few as possible move, and those that move travel as little as possible."""
        m = self.map
        if self.rank_by == "visits":
            ranked = sorted(self.by_sku, key=lambda s: (-self.visits[s], -self.demand[s]))
        else:  # items
            ranked = sorted(self.by_sku, key=lambda s: (-self.demand[s], -self.visits[s]))
        mid = sum(x for x, _ in m.by_row[SLOT_ROWS[0]]) / len(m.by_row[SLOT_ROWS[0]])
        robots = {r.pos for r in self.robots}
        free: dict[Coords, list[Coords]] = {}
        for y in SLOT_ROWS:
            key = (lambda s: abs(s[0] - mid)) if self.fill_order == "middle" else (lambda s: s[0])
            for s in sorted(m.by_row[y], key=key):
                if s not in robots:
                    free.setdefault(m.side_off[s], []).append(s)
        layout = {p.id: free[m.side_off[p.pos]].pop(0) for sku in ranked for p in self.by_sku[sku]}
        groups: dict[tuple, list[int]] = {}
        for pid, s in layout.items():
            groups.setdefault((s[1], m.side_off[s]), []).append(pid)
        for pids in groups.values():
            slots = [layout[pid] for pid in pids]
            cost = [[0 if self.entities[pid].pos == s else 1000 + m.between[self.entities[pid].pos][s] for s in slots]
                    for pid in pids]
            rows, cols = linear_sum_assignment(cost)
            for i, j in zip(rows, cols):
                layout[pids[i]] = slots[j]
        return layout

    def relocate_all(self, verbose: bool = True) -> None:
        """Move every pallet to its `target_layout` slot before any order is planned. The robot with the earliest
        frontier takes the next trip (`relocation_task`). A move is ready once nothing (still) stands in its target
        slot. If every remaining move waits on another (a cycle), one of them is re-targeted to a free slot."""
        target = self.target_layout()
        pending = {pid: s for pid, s in target.items() if self.entities[pid].pos != s}
        if verbose:
            print(f"  presort: {len(pending)} of {len(target)} pallets to move")
        robots = self.robots
        while pending:
            occupied = {p.pos for p in self.pallets}
            ready = [(self.entities[pid], s) for pid, s in pending.items() if s not in occupied]
            if not ready:
                self._break_cycle(pending, occupied)
                continue
            robot = min(robots, key=lambda r: r.frontier[-1][1])
            if not self.execute(robot, (TaskType.RELOCATE, self.relocation_task(robot, ready))):
                inv, t = robot.frontier[-1]
                robot.frontier.append((inv, t + IDLE_STEP))
                continue
            for pid in [pid for pid, s in pending.items() if self.entities[pid].pos == s]:
                del pending[pid]
        self.presort_end = max(r.frontier[-1][1] for r in robots)
        if self.barrier:
            for r in robots:
                r.frontier.append(((), self.presort_end))
        if verbose:
            print(f"  presort done by t={self.presort_end}: {self.relocations} pallets in "
                  f"{self.relocation_trips} trips")

    def _break_cycle(self, pending: dict[int, Coords], occupied: set[Coords]) -> None:
        m = self.map
        robots = {r.pos for r in self.robots}
        spare = [s for s in m.slots if s not in occupied and s not in pending.values() and s not in robots]
        pid, s = min(((pid, s) for pid in pending for s in spare if m.side_off[s] == m.side_off[pending[pid]]),
                     key=lambda o: abs(o[1][1] - pending[o[0]][1]) + abs(o[1][0] - pending[o[0]][0]))
        pending[pid] = s

    def relocation_task(self, robot: Robot, ready: list[tuple[Pallet, Coords]]) -> list:
        """Options for `robot`'s next presort trip, best first. Seed: the ready move with the shortest trip. If
        pairing, also consider each ready move on the other side of a column pair as a second pallet (it rides in
        the robot's other side slot), in either docking and either undocking order. The best pair is offered first
        if it beats doing the two moves one after the other by `pair_margin`; the seed alone is the fallback."""
        m = self.map

        def cost(docks, dests):
            c, cur = m.dist_to_pallet(docks[0].pos, robot.pos), docks[0].pos
            for q in docks[1:]:
                c, cur = c + m.between[cur][q.pos], q.pos
            for _, s in dests:
                c, cur = c + m.between[cur][s], s
            return c, int(m.to_parking[m.side[cur]])

        def single(p, s):
            walk, park = cost([p], [(p, s)])
            return walk + park, walk

        p, s = min(ready, key=lambda o: single(*o)[0])
        options = [((p,), ((p, s),))]
        if not self.pair:
            return options
        _, seed_walk = single(p, s)
        best = None
        for q, qs in ready:
            if m.side_off[q.pos] == m.side_off[p.pos]:
                continue
            # the two moves done back to back: seed, then from its slot fetch q and put it in place
            apart = seed_walk + m.between[s][q.pos] + m.between[q.pos][qs] + int(m.to_parking[m.side[qs]])
            for docks in ((p, q), (q, p)):
                for dests in (((p, s), (q, qs)), ((q, qs), (p, s))):
                    walk, park = cost(docks, dests)
                    if walk + park <= apart - self.pair_margin and (best is None or walk + park < best[0]):
                        best = (walk + park, docks, dests)
        if best is not None:
            options.insert(0, best[1:])
        return options

    # ------------------------------------------------------------------ re-slotting replenished pallets

    def orders_served(self, p: Pallet) -> float:
        """Orders a full `p` can serve before it runs dry, at the SKU's average quantity per remaining order."""
        visits = self.visits[p.sku]
        if visits <= 0:
            return 0.0
        return min(visits, p.max_count / max(1.0, self.demand[p.sku] / visits))

    def choose_slot(self, p: Pallet, pos: Coords, t: int, taken: set[Coords]) -> Coords:
        """Where to put `p` back after its refill, the robot standing at `pos` at timestep t.

        Walk down the slot rows from the SKU's target row. On each row take the open slot (on p's side of a column
        pair, nothing reserved in it from t on) nearest the robot. A slot within `max_shift` columns of p's
        current slot is taken outright. A farther one must pay for its detour: fill_weight * (steps it is closer to
        the fulfilment row) * (orders p will serve) - (extra steps to put it there and park) must be positive.
        Otherwise try the next row down. Reaching p's own row (or running out of rows) puts it back where it was.
        """
        home = p.pos
        if self.reslot == "off":
            return home
        m, rt = self.map, self.reservations
        dist = self._heuristic_cell(pos)  # BFS from the robot (distances are symmetric)

        def cost(s: Coords) -> float:
            side = m.side[s]
            return int(dist[side]) + int(m.to_parking[side]) + END_WEIGHT * int(m.to_fulfill[side])

        rows = SLOT_ROWS[SLOT_ROWS.index(self.target_row[p.sku]):]
        for y in rows:
            if y == home[1]:
                return home
            open_ = [s for s in m.by_row[y] if s not in taken and m.side_off[s] == m.side_off[home]
                     and rt.free_for(*s, t, None, ())]
            if not open_:
                continue
            s = min(open_, key=lambda s: dist[m.side[s]])
            if abs(m.side[s][0] - m.side[home][0]) <= self.max_shift:
                return s
            saved = self.fill_weight * (m.from_top[home] - m.from_top[s]) * self.orders_served(p)
            if saved - (cost(s) - cost(home)) > 0:
                return s
        return home

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
        elif task[0] is TaskType.RELOCATE:
            attempts = [lambda o=o: self._plan_replenish(robot, list(o[0]), t0, refill=False, dests=list(o[1]))
                        for o in task[1]]
        else:
            seq, fallback, carry_off = task[1]
            seqs = [seq] if seq == fallback else [seq, fallback]
            attempts = [lambda s=s: self._plan_replenish(robot, list(s), t0, carry_off) for s in seqs]
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
        rid, side = robot.id, self.map.side[pallet.pos]
        rob = [(rid, (0, 0))]
        path, ta = self._walk(rob, robot.pos, t0, lambda c: c == side, lambda ta: ta + 1, self._heuristic_cell(side))
        self._moves(path, t0, actions, reservations, rob)
        actions.append((ta, Action.DOCK, pallet.pos))
        reservations.append((rid, side, ta, ta + 1))
        foot = rob + [(pallet.id, self.map.side_off[pallet.pos])]
        foot, pos, t = self._flip(foot, side, ta + 1, BELOW, actions, reservations, final=True)
        frontiers.append(((), t))

        def commit():
            self.reservations.unpark(*pallet.pos, ta + 1)
            pallet.carrier = rid
            robot.carry = pallet.id

        return reservations, actions, frontiers, pos, commit

    def _plan_order(self, robot: Robot, need: Counter, t0: int):
        stops, _, unmet = self.tour(need, robot.pos, t0, robot)
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
                access = set(self.map.access[p.pos])
                # occupy the access cell from arrival until the pallet is available, then for `qty` picks
                hold = lambda ta, p=p, qty=qty: max(ta, p.locked_until + 1) + qty
                path, ta = self._walk(foot, pos, t, access.__contains__, hold, self.map.to_pallet[p.pos])
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

    def _plan_replenish(self, robot: Robot, seq: list[Pallet], t0: int, carry_off: Coords = ABOVE,
                        refill: bool = True, dests: list[tuple[Pallet, Coords]] | None = None):
        """Trip to the replenishment row: flip the carried pallet to `carry_off` (above or to one side), dock `seq` in order (each from
        its side access cell, into the slot that leaves it at its home offset), refill, then undock them in reverse
        order. Reversing makes every undock footprint identical to the matching dock footprint, which was valid.
        Finally flip the carried pallet back below and park.

        Presort relocation reuses this: refill=False skips the replenishment row, and `dests` gives the
        (pallet, slot) undock order instead of reversed docking order and `choose_slot`."""
        actions, reservations, frontiers = [], [], []
        rid = robot.id
        foot = self._foot(robot)
        pos, t = robot.pos, t0

        # 1. if the first pallet still has picks reserved far ahead, wait here (parked, safe) rather than squatting
        #    in the aisle, which both blocks other robots and makes the space-time search explode.
        if seq:
            t = max(t0, seq[0].last_pick + 1 - self.map.dist_to_pallet(seq[0].pos, pos))
            self._hold(reservations, foot, pos, t0, t)
        if robot.carry is not None:
            foot, pos, t = self._flip(foot, pos, t, carry_off, actions, reservations)

        # 2. collect each pallet; dock only after the last pick already reserved on it has happened
        docked = []
        for q in seq:
            side = self.map.side[q.pos]
            dock_at = lambda ta, q=q: max(ta, q.last_pick + 1)
            path, ta = self._walk(foot, pos, t, lambda c, s=side: c == s, lambda ta, d=dock_at: d(ta) + 1,
                                  self._heuristic_cell(side))
            self._moves(path, t, actions, reservations, foot)
            td = dock_at(ta)
            self._hold(reservations, foot, side, ta, td + 1)
            actions.append((td, Action.DOCK, q.pos))
            foot = foot + [(q.id, self.map.side_off[q.pos])]
            docked.append((q, td))
            pos, t = side, td + 1

        # 3. drag everything to the replenishment row. The refill fires at the end of the timestep in which the
        #    robot moves onto row 39, so it can turn straight round on the next timestep.
        if refill:
            path, ta = self._walk(foot, pos, t, lambda c: c[1] == REPLENISH_ROW, lambda ta: ta, self.map.to_replenish)
            self._moves(path, t, actions, reservations, foot)
            pos, t = path[-1], ta
        dock_time = {q.id: td for q, td in docked}
        if dests is None:
            dests = [(q, None) for q, _ in reversed(docked)]

        # 4. put each pallet in a slot (`choose_slot`: possibly not the one it came from), last docked first.
        #    A new slot is parked in the reservation table while the rest of the trip is planned, so the robot's
        #    later legs route round it; that is undone afterwards and redone on commit.
        returned, tentative = [], []
        try:
            for q, slot in dests:
                td = dock_time[q.id]
                if slot is None:
                    slot = self.choose_slot(q, pos, t, {s for _, _, _, s in returned})
                side = self.map.side[slot]
                path, ta = self._walk(foot, pos, t, lambda c, s=side: c == s, lambda ta: ta + 1,
                                      self._heuristic_cell(side))
                self._moves(path, t, actions, reservations, foot)
                actions.append((ta, Action.UNDOCK, slot))
                self._hold(reservations, foot, side, ta, ta + 1)
                reservations.append((q.id, slot, ta, None))
                if slot != q.pos:
                    self.reservations.park(*slot, ta, q.id)
                    tentative.append((slot, ta))
                foot = [f for f in foot if f[0] != q.id]
                returned.append((q, td, ta, slot))
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
        finally:
            for slot, ta in tentative:
                self.reservations.unpark(*slot, ta)

        def commit():
            # each home cell is free while its pallet is out; the reservations above re-park it there on return.
            # Other robots may not pick from it until it is back (locked_until), by which time it is full again.
            moved = False
            for q, td, tu, slot in returned:
                self.reservations.unpark(*q.pos, td + 1)
                moved |= slot != q.pos
                self.moved += refill and slot != q.pos
                q.pos = slot
                if refill:
                    q.planned_count = q.max_count
                q.locked_until = tu
            if moved and self.live_map:
                self.map.relayout({p.pos for p in self.pallets if p.carrier is None})
                self._cell_bfs.clear()
                self._spot_bfs.clear()
            if not refill:
                self.relocation_trips += 1
                self.relocations += len(returned)
                return
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
