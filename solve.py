"""Usage: python solve.py [worklist] [solution] [--window N]"""

import argparse
import time

from manager import Manager


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("worklist", nargs="?", default="BIG_ORDER.txt")
    ap.add_argument("solution", nargs="?", default="solution.txt")
    ap.add_argument("--window", type=int, default=10, help="how many feasible queue-head orders to compare")
    args = ap.parse_args()

    start = time.time()
    m = Manager(args.worklist, window=args.window)
    makespan = m.solve()
    m.write_solution(args.solution)
    print(f"makespan {makespan} timesteps, {m.replenishments} replenishments, "
          f"wrote {args.solution} in {time.time() - start:.1f}s")


if __name__ == "__main__":
    main()
