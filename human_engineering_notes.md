After working out the basic architecture largely by hand, Claud Opus 5.5 was left to work out the details of a naive v1 implementation, which used greedy path length to score orders, completed them in order, and scheduled refills per-pallet when no orders were possible. First score of 66,596 timesteps, which is honestly much better than I imagined the naive solve would be. My intuition was that a naive solve would be >100k, and that the ammount of time I had to think about optimization would get me to 2-3x the leading time. Nice :).

The relatively sparse map definitely makes this problem way easier.

Firstly, Claude missed that replenishment is automatic. Having it remove a 1-tick wait when replenishing didn't actually help (made things a few hundred ticks worse, yay naive planning), but should hopefully be part of later improvements.

First optimization is going to be having every robot carry 1 pallet of a different most-requested sku all the time, and to add some smarter replenishment heuristics (since I want to make sure that a carried pallet doesn't screw with replenishment time).

Wow, that helped way less than expected - best tuned improvement took the solve down to 65,605. I think changing around the floor plan to get pallets closer to the top as part of refils is going to matter way more. Gonna try a few more Claude suggested improvements to carrying, then switch to thinking about floor plan changes.

Adding some smarter filtering on replenishments seems to matter much more than 