# Solution Overview

**Result:** all 1,000 orders fulfilled in **62,398 timesteps** (`--rank-tour`). Checked with `validate.py`.

```
python3 solve.py [BIG_ORDER.txt] [solution.txt] --rank-tour   # 62,398, ~4 min
python3 solve.py [BIG_ORDER.txt] [solution.txt]               # 63,019, ~50 s
python3 validate.py [BIG_ORDER.txt] [solution.txt]            # independent rule check
python3 solve.py ... --reslot packed                           # re-slot pallets on refill (64,275; see below)
```

## Approach

A basic queue-based allocator with cooperative (space-time) A* path planning:

1. **Pick the robot.** Choose the robot whose *frontier* (the timestep its last planned task ends) is smallest.
2. **Pick the task.**
   - **Order:** scan the order queue for the first `window` (10) orders that the stock on the map can fully satisfy. Assign the one with the shortest estimated path from this robot. The estimate is a greedy nearest-pallet tour that ends on the fulfilment row.
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
| [manager.py](manager.py) | Queue allocation, route and replenishment heuristics, task planning, solution writer |
| [solve.py](solve.py) | Entry point |
| [validate.py](validate.py) | Simulator of the README rules. Stricter than required where the README is ambiguous |

## Assumptions to confirm on the testbench

These are chosen to be safe under any reading of the rules. Each costs some time if the real simulator is more lenient.

- **No following:** a robot never enters a cell another entity is leaving in the same timestep, in either direction. We don't depend on the order the simulator resolves moves in.
- **Action coordinates:** `fulfill` is written with the robot's own coordinates. `pick`, `dock` and `undock` use the pallet's coordinates.
- **Dock side:** robots never dock from directly north of a pallet. A docked pallet keeps its offset, so a pallet hanging below the robot would stop the robot from ever reaching row 39.

## Where the time goes

Measured on the current best `solution.txt` (62,398, `--rank-tour`, no re-slotting):

| Robot-timesteps (5 robots × 62,398 = 312k) | Count | Share |
|---|---|---|
| Moves | 235,183 | 75.4% |
| Picks (fixed by the problem) | 64,506 | 20.7% |
| Fulfil, dock, undock | 1,470 | 0.5% |
| No action (waiting, including the tail after a robot's last task) | 10,831 | 3.5% |
| Dragging docked pallets, dock → undock (overlaps with moves) | 11,193 | 3.6% |

Robots finish between t=62,064 and t=62,397.

On average an order visits about 37 distinct SKUs and takes about 235 moves of travel against about 65 picks. Robots finish within about 335 timesteps of each other, so load balance is not the problem. **Travel is.** The selection window only helps at the margin: window 1 gives 69,613, 10 gives 66,596, 30 gives 66,333 and 100 gives 66,742 (measured before the row-39 wait was removed). Dropping the 213 one-step waits changed the makespan by +147. The greedy allocator is sensitive to small timing shifts, so differences of a few hundred timesteps are noise rather than signal.

## Recommendations (ordered by expected impact)

1. **Carry the high runners.** A robot can dock up to 4 pallets and pick from them with no travel. Demand is Zipf-shaped: SKU 0 alone is 7,824 of 64,506 items (12%), and the top few SKUs appear in almost every order. Each robot could carry 2–3 high-runner pallets for the whole run and refill them by passing along row 39. This removes the most frequent stops from every tour. This needs footprint-aware A*, which `coopastar.plan` already supports through `offsets`.
2. **Re-layout the floor up front.** Re-slotting on refill is implemented (`--reslot`) but loses about 1,250
   timesteps, because the half-migrated layout is worse than the start (see above). The packed end state is worth
   about 4,300 timesteps (58,717 if pallets start there), so migrating there *early* in dedicated passes should pay
   off. That means about 200 pallet moves, roughly 2,000 makespan if spread over 5 robots, while the robots are
   otherwise idle at the start or interleaved with the first orders. Then keep re-slotting on refill to hold the
   layout. A layout-aware slot test (the change in estimated tour cost over upcoming orders, rather than vertical
   gain × orders) would stop refills from making the transient layout worse.
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
5. **Smarter order selection.** Score orders by travel *per item* or by marginal travel given the current position, rather than raw path length. The raw measure favours small orders now and leaves expensive ones for the end. Selecting orders for all robots jointly would also beat the earliest-frontier greedy.
6. **Relax the no-following rule once confirmed.** Allow follow-the-leader moves if the testbench accepts them. This is a small win: tens to hundreds of timesteps.
7. **Planner robustness and speed.** Prioritised planning never revises a committed plan. Windowed replanning or conflict-based search would recover the waits that come from planning order. The Python A* is fast enough now (about 16 s per solve), but ideas 1–3 add search. Porting the inner loop to numba, or caching heuristics, will keep iteration quick.
