# Solution Overview

**Result:** all 1,000 orders fulfilled in **58,940 timesteps** (defaults: presort with 2-pallet trips, window 20). Checked with `validate.py`.

```
python3 solve.py [BIG_ORDER.txt] [solution.txt]               # 58,940, ~50 s
python3 validate.py [BIG_ORDER.txt] [solution.txt]            # independent rule check
python3 solve.py ... --no-presort --window 10 --rank-tour     # previous best without presort: 62,398, ~4 min
python3 solve.py ... --no-presort --window 10 --reslot packed # re-slot pallets on refill instead (64,275; see below)
```

## Approach

A basic queue-based allocator with cooperative (space-time) A* path planning. First, every pallet is moved into a
layout sorted by SKU popularity (see *Presort* below). Then:

1. **Pick the robot.** Choose the robot whose *frontier* (the timestep its last planned task ends) is smallest.
2. **Pick the task.**
   - **Order:** scan the order queue for the first `window` (20) orders that the stock on the map can fully satisfy. Assign the one with the shortest estimated path from this robot. The estimate is a greedy nearest-pallet tour that ends on the fulfilment row.
   - **Replenish:** only when *no* order in the whole queue can be satisfied. Every non-full pallet of a SKU that is short for the head-of-queue orders is a candidate. The candidate chosen is the one whose refill gives the lowest estimated movement cost for those orders. Each item still unavailable after the refill adds a large penalty (1000) to the cost. In effect, the refill that unblocks the most demand wins, and travel time breaks ties.
3. **Plan it.** Plan each leg of the task with space-time A* against a reservation table that holds every trajectory already committed. The legs are walk to a pallet, then hold there while picking (or docking), and so on. The plan is committed only if every leg is found. Otherwise the robot idles for 10 timesteps and is retried.
4. Repeat until the queue is empty, then write all actions sorted by `(timestep, robot)`.

### Order tours: greedy vs aisle-aware S-shaped sweep (`--tour`)

The pallets stand in six 2-wide columns, in two bands (y=7–16 and y=23–32). Each pallet is picked from a *face*: the
column of access cells beside its pallet column (x = 9, 12, 16, 19, …, 47). The sweep treats each face as a lane.
Within each band, it walks the faces left-to-right or right-to-left, alternating top-to-bottom and bottom-to-top, so
consecutive lanes join at the band's end. It sweeps one band and then the other, and keeps the best of the 8
band-order and direction combinations. The sweep also picks *which* duplicate pallet each SKU uses. Starting from the
greedy tour's pallets, it swaps a SKU to another pallet with enough stock whenever that shortens the sweep, and
repeats until no swap helps. SKUs split across several pallets keep the greedy allocation.

| `--tour` (order ranking by greedy estimate) | Makespan | Moves |
|---|---|---|
| `greedy` (previous behaviour) | 63,841 | 242,149 |
| `sweep` | 64,829 | 243,809 |
| `best`: the shorter of greedy and sweep per order (default) | 63,019 | 236,580 |
| `best` + `--rank-tour` (candidates ranked by that tour too) | **62,398** | 235,183 |

On a static check (full pallets, starting anywhere on row 0, averaged over all 1,000 orders), the sweep with pallet
choice needs 192 moves per order against the greedy tour's 207. Without the pallet choice it needs 218, so the
pallet choice is where the sweep's gain comes from. In a live solve, though, the sweep is only about 2% shorter on
average, and it is longer on 37% of orders. Two things erode it:

- Depleted pallets leave fewer duplicates to swap to.
- The robot starts at a fixed x, usually where it last fulfilled. A sweep must begin at one end, but nearest-neighbour adapts to the start position.

Taking the shorter tour per order is what pays off. The sweep's local search costs about 20 ms per call, so by default
it runs only for the order being planned. `--rank-tour` also runs it for each of the `window` candidates, which takes
about 5× as long.

### Presort: moving every pallet to the sorted layout first (on by default; `--no-presort` turns it off)

Before any order is planned, every pallet is moved to its slot in the `packed` layout (described under re-slotting
below):

- **Target layout (`target_layout`).**
  - SKUs are ranked by how many orders use them, ties broken by items.
  - Their pallets fill the slot rows top-down: y=3–16, then y=20–32. Within a row, the middle columns fill first.
  - Each pallet stays on its side of a column pair, because it keeps its docking offset. Cells where robots start are left empty.
  - Within each row, the pallets bound for it are matched to its slots (Hungarian, `scipy.optimize`) so the fewest move and the rest travel least.
  - Result: 233 of 240 pallets move. The 240 pallets fill rows down to y=26.
- **Ordering.** A move is *ready* once nothing still stands in its target slot. Its occupant must already have been
  scheduled to leave, and A* waits for that departure through the reservation table. If every remaining move waits
  on another (a cycle; 3 exist), one of them is re-targeted to the nearest spare slot on its row and side.
- **Trips (`relocation_task`).** These reuse the replenishment trip planner, `_plan_replenish(refill=False, dests=...)`:
  dock, carry, undock and park, but no visit to row 39.
  - The robot with the earliest frontier takes the ready move with the shortest estimated trip.
  - Like batched replenishment, it may take a second pallet in its other side slot. The candidate is any ready move whose pallet is on the opposite side of a column pair.
  - The best pair, over both docking and both undocking orders, is used if it is estimated at least `--pair-margin` (0) steps shorter than doing the two moves one after the other.
  - If the pair can't be planned, the robot falls back to the single move.
- Distance tables are recomputed after every trip, as with re-slotting.
- Replenishment afterwards returns each pallet to its (new) slot.

| Variant | Presort done by | Makespan |
|---|---|---|
| No presort (window 10, same tours) | | 63,019 |
| Presort, 1 pallet per trip (233 trips), window 10 | t=2,382 | 60,411 |
| Presort, up to 2 pallets per trip (121 trips), window 10 | t=1,765 | 59,802 |
| *Pallets* start *in the packed layout (window 10)* | *0* | *58,717* |
| Pairs + window 20 (**default**) | t=1,765 | **58,940** |
| Pairs + window 5 / 15 / 25 / 30 | | 59,782 / 59,719 / 59,278 / 59,362 |
| Pairs + window 20 + strict barrier (no order until every move ends) | | 59,126 |
| Pairs + window 20 + rank SKUs by items | | 59,604 |
| Pairs + window 20 + rows filled left-to-right | | 59,278 |
| Pairs + window 20, pair margin −10 | t=1,758 | 59,206 |
| Pairs, window 10, pair margin 10 / 25 | t=1,797 / t=1,893 | 59,831 / 60,235 |
| Pairs + window 20 + `--rank-tour` (~10 min) | | 59,156 |
| Pairs + window 10, greedy tours only | | 60,557 |

Pairing almost halves the trips and saves about 600 timesteps of presort. The presort costs about 1,100 timesteps
of makespan against starting in the sorted layout (59,802 vs 58,717). That is less than its own duration, because
robots start picking from the half-built layout while the last moves land. A strict barrier (`--barrier`) made
no reliable difference. `--rank-tour` no longer helps once the layout is sorted.

### Re-slotting pallets on replenishment (`--reslot`, off by default)

Instead of returning a refilled pallet to the slot it came from, it can be put in a slot whose row is set by its SKU.

- **Slots.** Every cell of the six column pairs from y=3 to 16 (top block) and y=20 to 32 (bottom block). That leaves a
  3-row gap at y=17–19. A pallet docked from the aisle keeps its offset, so it can only go into a column on the same side
  of a pair (left or right) as the one it came from.
- **Target row.** SKUs are ranked by how many orders use them (ties broken by items). `packed`: walking down the rows,
  each SKU gets the row its pallets would fill if the blocks were filled in rank order (12 slots per row), so 240
  pallets reach y=25. `linear`: ranks are spread evenly from y=3 to y=32.
- **Choice (`choose_slot`).** Walk down the rows from the target row. On each row, take the open slot nearest the
  robot. Open means on the right side, and nothing reserved in that cell from now on. If it is within `--max-shift` (7)
  columns of the pallet's old slot, take it. Otherwise it must pay for its detour:
  `fill_weight × (steps closer to the fulfilment row) × (orders the full pallet serves) − (extra steps to put it there
  and park) > 0`. A slot that fails goes to the next row down. Reaching the pallet's own row puts it back where it was.
- **Live distances.** Every map table is now keyed by slot cell. `StaticMap.relayout` recomputes them (scipy
  `csgraph`, about 75 ms) whenever a commit moves a pallet. Stale tables cost about 1,200 timesteps (65,488 vs 64,287).

| Variant (default greedy ranking unless noted) | Makespan | Moves | Re-slotted |
|---|---|---|---|
| `--reslot off` (pallets return home) | **63,019** | 236,580 | 0 |
| `packed`, fill weight 4 | 64,275 | 244,659 | 68 |
| `packed`, fill weight 0.25–2 | 64,287 | 244,641 | 68 |
| `packed`, max shift 14 / 0 / 100 | 64,358 / 64,809 / 64,644 | | 65–70 |
| `linear`, max shift 7 / 0 / 14 | 64,534 / 64,662 / 64,671 | | 70–73 |
| `packed`, fill weight 4, `--rank-tour` | 64,233 (vs 62,398 off) | | 74 |
| *Upper bound: pallets **start** in the packed layout, no re-slotting* | *58,717* | | |

**Why it loses.** The end-state layout is good. On a static check (full stock, greedy tour from row 0, averaged
over all orders), the fully packed layout averages 190.8 moves per order, against 212.7 for the starting layout.
Starting the solve in it scores 58,717. But refills move only about 70 pallets over the whole run, and the
half-migrated layout is *worse* than the start: 220.8 after 200 orders, 223.8 after 400, still 211.4 at the end.
A tour still sweeps down the aisles for most SKUs, and a pallet in the middle of an aisle it passes anyway costs
almost nothing. A lone pallet lifted to y=3 is an up-and-back detour from the top of the block on every tour that
doesn't enter or leave through that aisle. Moving one SKU-0 pallet from (39,11) to (39,3) adds 2.4 moves to the
average tour. The heuristic counts each visit as its own trip from row 0, so it accepts almost every move: orders
served are 27–150 per refill, which is why fill weight barely matters. Sorting by popularity within the original
rows (7–16, 23–32) doesn't help either (216.6). The gain comes from filling rows 3–6 with high runners.

### Order ranking and central robot assignment (`--select`, `--assign`; defaults unchanged)

Two variants of recommendation 5 were tried. Neither beat the default (58,940), so the defaults stay `raw` and `frontier`.

- **`--select`** ranks the `window` candidates. `raw` is the estimated travel from the robot (default). `marginal` is that travel minus the order's estimated cost from the fulfilment row, so it measures only what the robot's position adds. `per-item` is travel divided by items.
- **`--assign central`** chooses the (robot, order) pair together. Every robot within `--slack` steps of the earliest frontier is scored against every candidate: `score + lag_weight × (its frontier − earliest frontier)`. If no order is feasible, the earliest robot replenishes as before.

| Variant | Makespan |
|---|---|
| Default (`raw`, earliest-frontier robot) | **58,940** |
| `--select marginal` / `per-item` | 60,137 / 59,984 |
| Central, slack 10, lag 1 | 58,942 |
| Central, slack 20, lag 1 / 0.5 / 2 | 59,106 / 59,168 / 59,396 |
| Central, slack 20, lag 1, window 30 | 59,067 |
| Central, slack 5, lag 1 | 59,414 |
| Central, slack 10, lag 1, window 10 | 60,031 |
| Central, slack 10, lag 2 | 59,313 |
| Central, slack 60 / 200, lag 1 | 59,548 |
| Central, slack 60, lag 1, `marginal` | 59,630 |

Marginal and per-item ranking lose about 1,000–1,200. Central assignment is at best level with the default (slack 10: +2, within noise), and a wider slack is worse. Robots already finish within about 300 timesteps of each other, so there is little imbalance for joint assignment to fix. Letting a later-frontier robot plan first also fits worse into the prioritised reservation table.

### What each task does

- **Order:** visit pallets in greedy nearest-first order and pick the required quantity at each. Then go to the fulfilment row, fulfil, and park there.
- **Replenish:**
  1. Wait at the parking spot until just before the last pick already reserved on the pallet.
  2. Walk to a side access cell and dock.
  3. Drag the pallet to row 39. The refill fires at the end of the arrival timestep, so the robot turns straight round.
  4. Drag it back to its home slot (or, with `--reslot`, the slot `choose_slot` picks) and undock.
  5. Park in an open cell away from any pallet, so the robot never blocks an access cell.

### Stock bookkeeping (no over-picking)

- `Pallet.planned_count` is the stock left after every pick reserved so far. A plan deducts its picks when it commits.
- Replenishment docks only after the pallet's `last_pick`. While the pallet is out, `locked_until` blocks new picks from it.
- When replenishment is planned, `planned_count` resets to `max_count`, because every later pick happens after the refill.
- Stock from in-flight refills counts when deciding whether an order is feasible. A robot may therefore wait beside a pallet until it returns full, instead of starting another refill.

## Code map

| File | Role |
|---|---|
| [objects.py](objects.py) | `Robot` and `Pallet` dataclasses (`planned_count`, `locked_until`, `last_pick`) and frontier types |
| [navigation.py](navigation.py) | `ReservationTable` (`[x, y, t]` occupancy, plus a permanent layer for resting pallets and parked robots) and `StaticMap` (distance tables keyed by slot cell, used for A* heuristics and route estimates; `relayout` recomputes them with scipy when pallets change slot) |
| [coopastar.py](coopastar.py) | Space-time A*. It supports a multi-cell footprint (robot plus docked pallet) and "arrive, then hold for N steps" goals |
| [manager.py](manager.py) | Presort (target layout, relocation trips), queue allocation, route and replenishment heuristics, task planning, solution writer |
| [solve.py](solve.py) | Entry point |
| [validate.py](validate.py) | Simulator of the README rules. Stricter than required where the README is ambiguous |

## Assumptions to confirm on the testbench

These are chosen to be safe under any reading of the rules. Each costs some time if the real simulator is more lenient.

- **No following:** a robot never enters a cell another entity is leaving in the same timestep, in either direction. We don't depend on the order the simulator resolves moves in.
- **Action coordinates:** `fulfill` is written with the robot's own coordinates. `pick`, `dock` and `undock` use the pallet's coordinates.
- **Dock side:** robots never dock from directly north of a pallet. A docked pallet keeps its offset, so a pallet hanging below the robot would stop the robot from ever reaching row 39.

## Where the time goes

Measured on the current best `solution.txt` (58,940, defaults):

| Robot-timesteps (5 robots × 58,940 = 295k) | Count | Share |
|---|---|---|
| Moves | 218,930 | 74.3% |
| … of which presort moves (before t=1,765) | 8,220 | 2.8% |
| Picks (fixed by the problem) | 64,506 | 21.9% |
| Fulfil, dock, undock (233 presort docks and undocks included) | 1,936 | 0.7% |
| No action (waiting, including the tail after a robot's last task) | 9,328 | 3.2% |
| Dragging docked pallets, dock → undock (overlaps with moves) | 16,797 | 5.7% |

Robots finish between t=58,637 and t=58,939. The presort occupies about 8,800 robot-timesteps (5 × 1,765) and
saves about 23,000 order and replenishment moves (window 10: 236,580 without presort vs 213,401 after it).

On average an order visits about 37 distinct SKUs and takes about 211 moves of travel against about 65 picks. Robots finish within about 300 timesteps of each other, so load balance is not the problem. **Travel is.** The selection window only helps at the margin: window 1 gives 69,613, 10 gives 66,596, 30 gives 66,333 and 100 gives 66,742 (measured before the row-39 wait was removed). Dropping the 213 one-step waits changed the makespan by +147. The greedy allocator is sensitive to small timing shifts, so differences of a few hundred timesteps are noise rather than signal.

## Recommendations (ordered by expected impact)

1. **Carry the high runners.** A robot can dock up to 4 pallets and pick from them with no travel. Demand is Zipf-shaped: SKU 0 alone is 7,824 of 64,506 items (12%), and the top few SKUs appear in almost every order. Each robot could carry 2–3 high-runner pallets for the whole run and refill them by passing along row 39. This removes the most frequent stops from every tour. This needs footprint-aware A*, which `coopastar.plan` already supports through `offsets`.
2. **Optimise the target layout itself.** Presort is implemented and it is now the largest single gain. The
   layout is still the simple rank-by-row `packed` one. A local search over pallet swaps, scored by the static
   estimate (average tour over the orders, full stock), could find a better one. For example, it could spread a SKU's
   duplicates across aisles, or put the SKUs that are ordered together in the same aisle. Presort cost barely depends on
   which layout is targeted.
   Docking end-of-column pallets from above or below would also let a presort trip carry 3–4 pallets.
3. **Better tours.** The S-shaped sweep is implemented (see above), and the per-order better of sweep and greedy saves
   about 1,400 timesteps. Next steps:
   - Run 2-opt/or-opt over the static distance tables on whichever tour wins.
   - Let the pallet-swap local search also re-choose pallets for the greedy ordering.
   - Make the sweep start-aware, for example by beginning at the lane nearest the robot.

   Live tours still average about 216 moves. Speeding up `tour` (numba, or caching per order and stock state) would
   make `--rank-tour` cheap enough to keep on by default.
4. **Proactive and batched replenishment.** Today a refill only starts when *nothing* is feasible, so robots often stall on depletion and then refill one pallet per trip. Instead:
   - Trigger refills when a pallet falls below a threshold.
   - Dock several depleted pallets in one trip.
   - Include the replenisher's own travel in the pallet score.
   - Fold a refill into an order tour when the robot passes near row 39.
5. **Smarter order selection.** Tried and did not help (see *Order ranking and central robot assignment*): marginal and per-item ranking are 1,000+ worse, and joint robot-order assignment ties the default at best. A different angle would be looking ahead (choosing orders that leave stock and positions good for the next ones).
6. **Relax the no-following rule once confirmed.** Allow follow-the-leader moves if the testbench accepts them. This is a small win: tens to hundreds of timesteps.
7. **Planner robustness and speed.** Prioritised planning never revises a committed plan. Windowed replanning or conflict-based search would recover the waits that come from planning order. The Python A* is fast enough now (about 16 s per solve), but ideas 1–3 add search. Porting the inner loop to numba, or caching heuristics, will keep iteration quick.
