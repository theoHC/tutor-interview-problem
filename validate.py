"""Independent simulator for solution files, following the README rules.

Usage: python validate.py [worklist] [solution]

Deliberately strict where the README is ambiguous: a move's target must be empty both before and after the
timestep (apart from the mover's own docked pallets), so the result does not depend on intra-timestep ordering.
"""

import sys
from collections import Counter, defaultdict

WIDTH, HEIGHT = 60, 40


def load(path):
    lines = iter(line.split() for line in open(path).read().splitlines())
    robots = [tuple(map(int, next(lines))) for _ in range(int(next(lines)[0]))]
    cap = [int(next(lines)[0]) for _ in range(int(next(lines)[0]))]
    pallets = [tuple(map(int, next(lines))) for _ in range(int(next(lines)[0]))]
    orders = [tuple(map(int, next(lines))) for _ in range(int(next(lines)[0]))]
    return robots, cap, pallets, orders


def main(world="BIG_ORDER.txt", solution="solution.txt"):
    robots, cap, pallets, orders = load(world)
    rpos = list(robots)
    ppos = [(x, y) for x, y, _ in pallets]
    psku = [s for _, _, s in pallets]
    pcount = [cap[s] for s in psku]
    docked = [set() for _ in robots]  # robot -> pallet indices
    owner = [None] * len(pallets)
    storage = [Counter() for _ in robots]
    open_orders = Counter(tuple(sorted(o)) for o in orders)
    fulfilled = 0

    by_t = defaultdict(dict)
    last_t = -1
    for n, line in enumerate(open(solution)):
        t, r, a, x, y = line.split()
        t, r, x, y = int(t), int(r), int(x), int(y)
        assert t >= last_t, f"line {n}: timesteps not increasing"
        assert r not in by_t[t], f"line {n}: duplicate (t, robot)"
        last_t = t
        by_t[t][r] = (a, x, y)

    def occupants():
        occ = {}
        for i, c in enumerate(rpos):
            occ[c] = ("r", i)
        for i, c in enumerate(ppos):
            assert c not in occ, f"collision at {c}"
            occ[c] = ("p", i)
        return occ

    def adj(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1

    for t in range(last_t + 1):
        acts = by_t.get(t, {})
        before = occupants()
        picks = Counter()
        moves = {}
        for r, (a, x, y) in acts.items():
            c = (x, y)
            if a == "move":
                assert adj(rpos[r], c), f"t={t} r={r}: move not adjacent"
                d = (x - rpos[r][0], y - rpos[r][1])
                group = {("r", r)} | {("p", p) for p in docked[r]}
                for kind, i in group:
                    src = rpos[i] if kind == "r" else ppos[i]
                    dst = (src[0] + d[0], src[1] + d[1])
                    assert 0 <= dst[0] < WIDTH and 0 <= dst[1] < HEIGHT, f"t={t} r={r}: out of bounds"
                    assert before.get(dst) in (None, *group), f"t={t} r={r}: target {dst} occupied by {before.get(dst)}"
                    moves[(kind, i)] = dst
            elif a == "pick":
                p = ppos.index(c) if c in ppos else None
                assert p is not None and adj(rpos[r], c), f"t={t} r={r}: no adjacent pallet at {c}"
                picks[p] += 1
                storage[r][psku[p]] += 1
            elif a == "dock":
                p = ppos.index(c)
                assert adj(rpos[r], c) and owner[p] is None and len(docked[r]) < 4, f"t={t} r={r}: bad dock"
                owner[p] = r
                docked[r].add(p)
            elif a == "undock":
                p = ppos.index(c)
                assert owner[p] == r, f"t={t} r={r}: bad undock"
                owner[p] = None
                docked[r].discard(p)
            elif a == "fulfill":
                assert rpos[r][1] == 0, f"t={t} r={r}: fulfill off row 0"
                key = tuple(sorted(storage[r].elements()))
                assert open_orders[key] > 0, f"t={t} r={r}: storage matches no open order"
                open_orders[key] -= 1
                storage[r] = Counter()
                fulfilled += 1
            else:
                raise AssertionError(f"t={t}: unknown action {a}")
        for p, k in picks.items():
            assert pcount[p] >= k, f"t={t}: pallet {p} at {ppos[p]} over-picked ({pcount[p]} < {k})"
            pcount[p] -= k
        for (kind, i), dst in moves.items():
            if kind == "r":
                rpos[i] = dst
            else:
                ppos[i] = dst
        after = occupants()  # asserts no two entities share a cell
        assert len(after) == len(rpos) + len(ppos)
        for r in range(len(robots)):
            if rpos[r][1] == HEIGHT - 1:
                for p in docked[r]:
                    pcount[p] = cap[psku[p]]

    assert fulfilled == len(orders), f"only {fulfilled}/{len(orders)} orders fulfilled"
    print(f"VALID: {fulfilled} orders fulfilled, score = {last_t + 1} timesteps")


if __name__ == "__main__":
    main(*sys.argv[1:])
