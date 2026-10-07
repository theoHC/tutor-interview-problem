"""Queue-based task allocation.

Loop: take the robot whose frontier (the timestep at which its last planned task finishes) is smallest, give it a
task, plan that task with cooperative A* against everything already reserved, commit, repeat.

Task choice for that robot:
  * ORDER     - among the first `window` orders in the queue that current/incoming stock can satisfy, take the one
                whose estimated travel (greedy nearest-pallet tour from the robot, ending on the fulfilment row) is
                shortest.
  * REPLENISH - only if *no* order in the queue can be satisfied by the stock on the map. The pallet chosen is the
                one that, once refilled, most reduces the estimated movement cost of the orders at the head of the
                queue (unsatisfiable units are charged a large penalty, so unblocking orders dominates).
"""

from collections import Counter, deque
from enum import Enum, auto

from coopastar import NoPath, plan
from navigation import FULFILL_ROW, REPLENISH_ROW, ReservationTable, StaticMap, bfs
from objects import Action, Coords, Pallet, Robot

UNMET_PENALTY = 1000  # estimated cost per item an order cannot currently get (used when ranking replenishments)
IDLE_STEP = 10  # how far a robot's frontier advances when it has nothing it can do right now


class TaskType(Enum):
    ORDER = auto()
    REPLENISH = auto()


# (ORDER, tuple of SKUs) or (REPLENISH, pallet)
Task = tuple[TaskType, tuple[int, ...] | Pallet]

# A tentative plan is committed only once every stage of it has been found:
# (entity id, cell, first layer, last layer or None for "forever"), and (robot timestep, action, coords)
Reservation = tuple[int, Coords, int, int | None]


class Manager:
    def __init__(self, path: str, window: int = 10):
        self.entities: list[Robot | Pallet] = []  # index == id
        self.tasks: deque[Task] = deque()
        self.window = window
        self._load(path)
        self.reservations = ReservationTable(self.entities)
        self.map = StaticMap(self.pallets)
        self.by_sku: dict[int, list[Pallet]] = {}
        for p in self.pallets:
            self.by_sku.setdefault(p.sku, []).append(p)
        self._cell_bfs: dict[Coords, object] = {}
        self.replenishments = 0

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

    # ------------------------------------------------------------------ main loop

    def solve(self, verbose: bool = True) -> int:
        robots = self.robots
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
                done += 1
                if verbose and done % 50 == 0:
                    print(f"  {done} orders planned, frontier t={robot.frontier[-1][1]}, "
                          f"replenishments={self.replenishments}")
        return self.makespan

    @property
    def makespan(self) -> int:
        return max((t for r in self.robots for t, _, _ in r.actions), default=-1) + 1

    # ------------------------------------------------------------------ task selection

    def stock(self) -> Counter:
        """Items per SKU that are (or will be, once in-flight replenishments land) unreserved on the map."""
        s = Counter()
        for p in self.pallets:
            s[p.sku] += p.planned_count
        return s

    def select_task(self, robot: Robot) -> Task | None:
        stock = self.stock()
        candidates = []
        for task in self.tasks:
            need = Counter(task[1])
            if all(stock[sku] >= n for sku, n in need.items()):
                candidates.append(task)
                if len(candidates) == self.window:
                    break
        if candidates:
            t0 = robot.frontier[-1][1]
            # shortest estimated path wins
            return min(candidates, key=lambda task: self.route(Counter(task[1]), robot.pos, t0)[1])
        pallet = self.choose_replenishment()
        return None if pallet is None else (TaskType.REPLENISH, pallet)

    def route(self, need: Counter, start: Coords | None, t0: int = 0, override: dict | None = None):
        """Greedy nearest-pallet tour that collects `need`, finishing on the fulfilment row.

        start=None means "starting from the fulfilment row". `override` maps pallet id -> pretend available count.
        Returns (stops [(pallet, qty)], travel estimate, unmet item count). Waiting for a pallet that is still out
        being replenished is counted as travel time.
        """
        need = dict(need)
        avail = {}
        for sku in need:
            for p in self.by_sku.get(sku, ()):
                a = override.get(p.id, p.planned_count) if override else p.planned_count
                if a > 0:
                    avail[p.id] = a
        stops, travel, cur, t = [], 0, None, t0
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

    def choose_replenishment(self) -> Pallet | None:
        """Pick the pallet whose refill most reduces the estimated movement cost of the orders at the queue head."""
        head = [Counter(task[1]) for task, _ in zip(self.tasks, range(self.window))]
        stock = self.stock()
        short = {sku for need in head for sku, n in need.items() if stock[sku] < n}
        wanted = short or {sku for need in head for sku in need}
        candidates = [
            p for sku in wanted for p in self.by_sku[sku]
            if p.planned_count < p.max_count and p.locked_until < self.min_frontier
        ]
        if not candidates:
            return None

        def cost(override):
            total = 0
            for need in head:
                _, travel, unmet = self.route(need, None, override=override)
                total += travel + UNMET_PENALTY * unmet
            return total

        return min(candidates, key=lambda p: cost({p.id: p.max_count}))

    @property
    def min_frontier(self) -> int:
        return min(r.frontier[-1][1] for r in self.robots)

    # ------------------------------------------------------------------ planning

    def execute(self, robot: Robot, task: Task) -> bool:
        """Plan `task` for `robot` from its frontier; commit reservations and actions only if every stage succeeds."""
        rt = self.reservations
        inv, t0 = robot.frontier[-1]
        rt.unpark(*robot.pos, t0)
        try:
            if task[0] is TaskType.ORDER:
                result = self._plan_order(robot, Counter(task[1]), t0)
            else:
                result = self._plan_replenish(robot, task[1], t0)
        except NoPath:
            rt.park(*robot.pos, t0, robot.id)
            return False
        reservations, actions, frontiers, end_pos, commit = result
        commit()  # task-specific bookkeeping (stock, locks, freeing a carried pallet's home cell)
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

    def _walk(self, rid, pos, t, goal, hold, h, group=None, offsets=((0, 0),)):
        """One A* leg (robot alone unless `group`/`offsets` describe docked pallets). Returns (path, arrival time)."""
        path = plan(self.reservations, pos, t, group or (rid,), offsets, goal, hold, lambda c: h.item(c))
        return path, t + len(path) - 1

    @staticmethod
    def _moves(path, t, actions, reservations, ids_offsets):
        """Emit a `move` per changed cell and reserve every footprint cell for every layer of the path."""
        for i in range(1, len(path)):
            if path[i] != path[i - 1]:
                actions.append((t + i - 1, Action.MOVE, path[i]))
        for i, (x, y) in enumerate(path):
            for eid, (dx, dy) in ids_offsets:
                reservations.append((eid, (x + dx, y + dy), t + i, t + i))

    def _plan_order(self, robot: Robot, need: Counter, t0: int):
        stops, _, unmet = self.route(need, robot.pos, t0)
        assert unmet == 0
        actions, reservations, frontiers = [], [], []
        pos, t, inv = robot.pos, t0, []
        for p, qty in stops:
            access = set(self.map.access[p.id])
            # occupy the access cell from arrival until the pallet is available, then for `qty` picks
            hold = lambda ta, p=p, qty=qty: max(ta, p.locked_until + 1) + qty
            path, ta = self._walk(robot.id, pos, t, access.__contains__, hold, self.map.to_pallet[p.id])
            self._moves(path, t, actions, reservations, [(robot.id, (0, 0))])
            pos, start = path[-1], max(ta, p.locked_until + 1)
            reservations.append((robot.id, pos, ta, start + qty))
            for k in range(qty):
                actions.append((start + k, Action.PICK, p.pos))
            t = start + qty
            inv += [p.sku] * qty
            frontiers.append((tuple(sorted(inv)), t))
        # head for the fulfilment row and stay there (parked) until the next task is planned
        path, ta = self._walk(robot.id, pos, t, lambda c: c[1] == FULFILL_ROW, lambda ta: None, self.map.to_fulfill)
        self._moves(path, t, actions, reservations, [(robot.id, (0, 0))])
        reservations.append((robot.id, path[-1], ta, None))
        actions.append((ta, Action.FULFILL, path[-1]))
        frontiers.append(((), ta + 1))

        def commit():
            # stock is only deducted once the whole plan is known to be feasible
            for p, qty in stops:
                p.planned_count -= qty
                p.last_pick = max(p.last_pick, max(t for t, a, c in actions if a is Action.PICK and c == p.pos))

        return reservations, actions, frontiers, path[-1], commit

    def _plan_replenish(self, robot: Robot, pallet: Pallet, t0: int):
        actions, reservations, frontiers = [], [], []
        rid, pid, home = robot.id, pallet.id, pallet.pos
        rob = [(rid, (0, 0))]

        # 1. go to the pallet; dock only after the last pick already reserved on it has happened. If that pick is
        #    far off, wait in the (safe, parked) current cell rather than squatting in the aisle for ages, which
        #    both blocks other robots and makes the space-time search explode.
        dock_at = lambda ta: max(ta, pallet.last_pick + 1)
        t_leave = max(t0, pallet.last_pick + 1 - self.map.dist_to_pallet(pid, robot.pos))
        reservations.append((rid, robot.pos, t0, t_leave))
        # A docked pallet keeps its offset, so docking from directly north would leave the pallet hanging below the
        # robot and the robot could never stand on the replenishment row. Every pallet has a side access cell.
        dock_cells = {c for c in self.map.access[pid] if c != (home[0], home[1] - 1)}
        path, ta = self._walk(rid, robot.pos, t_leave, dock_cells.__contains__,
                              lambda ta: dock_at(ta) + 1, self.map.to_pallet[pid])
        self._moves(path, t_leave, actions, reservations, rob)
        dock_cell, td = path[-1], dock_at(ta)
        reservations.append((rid, dock_cell, ta, td + 1))
        actions.append((td, Action.DOCK, home))
        off = (home[0] - dock_cell[0], home[1] - dock_cell[1])
        both = [(rid, (0, 0)), (pid, off)]

        # 2. drag it to the replenishment row and sit there for one extra timestep so the refill certainly fires
        t = td + 1
        path, ta = self._walk(rid, dock_cell, t, lambda c: c[1] == REPLENISH_ROW, lambda ta: ta + 1,
                              self.map.to_replenish, (rid, pid), ((0, 0), off))
        self._moves(path, t, actions, reservations, both)
        t = ta + 1
        cell = path[-1]
        for eid, (dx, dy) in both:
            reservations.append((eid, (cell[0] + dx, cell[1] + dy), ta, t))

        # 3. bring it back to its home slot and undock
        path, ta = self._walk(rid, cell, t, lambda c: c == dock_cell, lambda ta: ta + 1,
                              self._heuristic_cell(dock_cell), (rid, pid), ((0, 0), off))
        self._moves(path, t, actions, reservations, both)
        tu = ta
        actions.append((tu, Action.UNDOCK, home))
        reservations.append((rid, dock_cell, tu, tu + 1))
        reservations.append((pid, home, tu, None))
        frontiers.append(((), tu + 1))

        # 4. clear the aisle: park somewhere open so this robot never blocks another robot's access cell
        t = tu + 1
        path, ta = self._walk(rid, dock_cell, t, lambda c: self.map.parking[c], lambda ta: None, self.map.to_parking)
        self._moves(path, t, actions, reservations, rob)
        reservations.append((rid, path[-1], ta, None))
        frontiers.append(((), ta))

        def commit():
            # the home cell is free while the pallet is out; the reservations above re-park it there on return.
            # Other robots may not pick from it until it is back (locked_until), by which time it is full again.
            self.reservations.unpark(*home, td + 1)
            pallet.planned_count = pallet.max_count
            pallet.locked_until = tu
            self.replenishments += 1

        return reservations, actions, frontiers, path[-1], commit

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
