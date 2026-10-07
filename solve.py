"""Usage: python solve.py [worklist] [solution] [--window N] [--carry] [--alpha A] [--max-detour D] [--max-fill F]
                   [--future-trip K] [--min-items-per-step N] [--trip-slot above|side]
                   [--tour greedy|sweep|best] [--rank-tour]
                   [--reslot off|packed|linear] [--max-shift N] [--fill-weight W]
                   [--no-presort] [--no-pair] [--pair-margin M] [--barrier]"""

import argparse
import time

from manager import (ALPHA, FILL_WEIGHT, FUTURE_TRIP, MAX_DETOUR, MAX_FILL, MAX_SHIFT, MIN_ITEMS_PER_STEP,
                     PAIR_MARGIN, Manager)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("worklist", nargs="?", default="BIG_ORDER.txt")
    ap.add_argument("solution", nargs="?", default="solution.txt")
    ap.add_argument("--window", type=int, default=20, help="how many feasible queue-head orders to compare")
    ap.add_argument("--carry", action="store_true", help="give each robot a top-SKU pallet to carry all run")
    ap.add_argument("--alpha", type=float, default=ALPHA, help="steps of detour worth refilling 100%% of a SKU's demand")
    ap.add_argument("--max-detour", type=float, default=MAX_DETOUR, help="max estimated steps added per extra pallet")
    ap.add_argument("--max-fill", type=float, default=MAX_FILL, help="ignore extra pallets fuller than this fraction")
    ap.add_argument("--future-trip", type=float, default=FUTURE_TRIP,
                    help="extra's detour must be <= this * expected cost of the dedicated trip it saves (inf = off)")
    ap.add_argument("--min-items-per-step", type=float, default=MIN_ITEMS_PER_STEP,
                    help="extra must newly cover at least this many items per detour step (0 = off)")
    ap.add_argument("--trip-slot", choices=("above", "side"), default="above",
                    help="where the carried pallet rides on replenishment trips (with --carry)")
    ap.add_argument("--tour", choices=("greedy", "sweep", "best"), default="best",
                    help="order tour: greedy nearest-pallet, aisle-aware S-shaped sweep, or the shorter of the two")
    ap.add_argument("--rank-tour", action="store_true",
                    help="rank candidate orders by the --tour estimate instead of the greedy one (~5x slower)")
    ap.add_argument("--reslot", choices=("off", "packed", "linear"), default="off",
                    help="where a replenished pallet goes back: its old slot, or a row set by SKU popularity "
                         "(packed: rows filled in rank order; linear: ranks spread evenly over all rows)")
    ap.add_argument("--max-shift", type=float, default=MAX_SHIFT,
                    help="columns a re-slotted pallet may move sideways before its detour must pay for itself")
    ap.add_argument("--fill-weight", type=float, default=FILL_WEIGHT,
                    help="steps saved per order served, per step a new slot is nearer the fulfilment row")
    ap.add_argument("--no-presort", action="store_true",
                    help="skip moving every pallet to the packed layout (high runners nearest row 0) before any order")
    ap.add_argument("--barrier", action="store_true",
                    help="presort: no robot starts an order until every pallet move has finished")
    ap.add_argument("--no-pair", action="store_true", help="presort: move one pallet per trip")
    ap.add_argument("--pair-margin", type=float, default=PAIR_MARGIN,
                    help="presort: steps a 2-pallet trip must save over two separate moves")
    ap.add_argument("--select", choices=("raw", "marginal", "per-item"), default="raw",
                    help="order ranking: raw travel, travel minus its cost from the fulfilment row, or travel per item")
    ap.add_argument("--assign", choices=("frontier", "central"), default="frontier",
                    help="frontier: earliest robot picks its order; central: pick the best (robot, order) pair")
    ap.add_argument("--slack", type=int, default=0, help="central: robots this many steps past the earliest compete")
    ap.add_argument("--lag-weight", type=float, default=1.0, help="central: score penalty per step of robot lag")
    args = ap.parse_args()

    start = time.time()
    m = Manager(args.worklist, window=args.window, carry=args.carry, alpha=args.alpha,
                max_detour=args.max_detour, max_fill=args.max_fill,
                future_trip=args.future_trip, min_items_per_step=args.min_items_per_step,
                trip_slot=args.trip_slot, tour_mode=args.tour, rank_tour=args.rank_tour,
                reslot=args.reslot, max_shift=args.max_shift, fill_weight=args.fill_weight,
                presort=not args.no_presort, pair=not args.no_pair, pair_margin=args.pair_margin,
                barrier=args.barrier, select=args.select, assign=args.assign, slack=args.slack,
                lag_weight=args.lag_weight)
    makespan = m.solve()
    m.write_solution(args.solution)
    if m.presort:
        print(f"presort: {m.relocations} pallets moved in {m.relocation_trips} trips, done by t={m.presort_end}")
    print(f"makespan {makespan} timesteps, {m.trips} trips, {m.replenishments} replenishments ({m.moved} re-slotted), "
          f"wrote {args.solution} in {time.time() - start:.1f}s")


if __name__ == "__main__":
    main()
