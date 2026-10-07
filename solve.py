"""Usage: python solve.py [worklist] [solution] [--window N] [--no-carry] [--alpha A] [--max-detour D] [--max-fill F]"""

import argparse
import time

from manager import ALPHA, MAX_DETOUR, MAX_FILL, Manager


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("worklist", nargs="?", default="BIG_ORDER.txt")
    ap.add_argument("solution", nargs="?", default="solution.txt")
    ap.add_argument("--window", type=int, default=10, help="how many feasible queue-head orders to compare")
    ap.add_argument("--no-carry", action="store_true", help="don't give each robot a permanently carried pallet")
    ap.add_argument("--alpha", type=float, default=ALPHA, help="steps of detour worth refilling 100%% of a SKU's demand")
    ap.add_argument("--max-detour", type=float, default=MAX_DETOUR, help="max estimated steps added per extra pallet")
    ap.add_argument("--max-fill", type=float, default=MAX_FILL, help="ignore extra pallets fuller than this fraction")
    args = ap.parse_args()

    start = time.time()
    m = Manager(args.worklist, window=args.window, carry=not args.no_carry, alpha=args.alpha,
                max_detour=args.max_detour, max_fill=args.max_fill)
    makespan = m.solve()
    m.write_solution(args.solution)
    print(f"makespan {makespan} timesteps, {m.trips} trips, {m.replenishments} replenishments, "
          f"wrote {args.solution} in {time.time() - start:.1f}s")


if __name__ == "__main__":
    main()
