"""Usage: python solve.py [worklist] [solution] [--window N] [--carry] [--alpha A] [--max-detour D] [--max-fill F]
                   [--future-trip K] [--min-items-per-step N] [--trip-slot above|side]"""

import argparse
import time

from manager import ALPHA, FUTURE_TRIP, MAX_DETOUR, MAX_FILL, MIN_ITEMS_PER_STEP, Manager


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("worklist", nargs="?", default="BIG_ORDER.txt")
    ap.add_argument("solution", nargs="?", default="solution.txt")
    ap.add_argument("--window", type=int, default=10, help="how many feasible queue-head orders to compare")
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
    args = ap.parse_args()

    start = time.time()
    m = Manager(args.worklist, window=args.window, carry=args.carry, alpha=args.alpha,
                max_detour=args.max_detour, max_fill=args.max_fill,
                future_trip=args.future_trip, min_items_per_step=args.min_items_per_step,
                trip_slot=args.trip_slot)
    makespan = m.solve()
    m.write_solution(args.solution)
    print(f"makespan {makespan} timesteps, {m.trips} trips, {m.replenishments} replenishments, "
          f"wrote {args.solution} in {time.time() - start:.1f}s")


if __name__ == "__main__":
    main()
