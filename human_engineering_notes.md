After working out the basic architecture largely by hand, Claud Opus 5.5 was left to work out the details of a naive v1 implementation, which used greedy path length to score orders, completed them in order, and scheduled refills per-pallet when no orders were possible. First score of 66,596 timesteps, which is honestly much better than I imagined the naive solve would be. My intuition was that a naive solve would be >100k, and that the ammount of time I had to think about optimization would get me to 2-3x the leading time. Nice :).

The relatively sparse map definitely makes this problem way easier.

Firstly, Claude missed that replenishment is automatic. Having it remove a 1-tick wait when replenishing didn't actually help (made things a few hundred ticks worse, yay naive planning), but should hopefully be part of later improvements.

First optimization is going to be having every robot carry 1 pallet of a different most-requested sku all the time, and to add some smarter replenishment heuristics (since I want to make sure that a carried pallet doesn't screw with replenishment time).

Wow, that helped way less than expected - best tuned improvement took the solve down to 65,605. I think changing around the floor plan to get pallets closer to the top as part of refils is going to matter way more. Gonna try a few more Claude suggested improvements to carrying, then switch to thinking about floor plan changes.

Adding some smarter filtering on replenishments seems to matter much more than carrying, and in fact at least in my current setup carrying seems to make things worse due to how it affects my replenishment runs. Getting the sense that carrying matters much less than I expected, and that floor plan is really where the gains are for me at my current level of available time for this problem.

Wow. Carrying is actually nontrivial to pull off, and not the help I imagined it'd be. Smarter replenishment got me to 63k. Let's rethink this.

I'd bet a more serious rethink of my planning approach (vs. the naive space time reservation table w/o replanning) would give me the biggest potential gains, but while I'm willing to let Claude do a lot of detail trobuleshooting for a project like this, I do insist on understanding my planning algorithms, and don't really have time to do the sort of deeper dive that I think I'd want to do/that would get me the best results. I really feel like there's some space here for employing optimal control concepts to do smarter global searches. If I didn't have a very hard final project for my masters degree right now, I'd definitely have spent more time trying to come up with an optimal control version of this problem. Even if it turned out to be the wrong way, I feel like it'd make me a smarter engineer. Very sad that we don't live in "infinite time for interesting problems" universe.

Hey, looks like smarter sweeping (explicitly handling the aisles, and only when it's better than ordering pallets greedily) gets us some tasty tasty gains at the expenese of crazy extra runtime, but now down to like 62,398. That's like. Real. But not much.

Ok actually that's not really tasty. Nice sure, but like. I should be expecting more. Gah. My planerrrrrr.

Ok, let's try a floor plan improvement - allowing the columns to be marched towards the fulfilment zone whenever they're taken down for refilling. So high runners should end up very close to fulfilment and less common ones further down. If this one doesn't take a big bite out of my performance, I'll call it on this problem.

Oh god, naive re-organizing of the pallets doesn't seem to help either. OK, this is an AI-encouraged problem, let's see if that helps. I can see the LLM chain-of-thought review after some incident report now "best approach depends on if reviewer cares more about final score or process integrity...". Anyhow, let's just let it go wild on planner improvements next, and if they turn out to work I'll do some reading on them.

Yeah the constant thread on these is that the cost of the optimization kills the gains - turns out that the fully re-organized warehouse is like 10% better than the default layout, but the partially reorganized warehouse is actually worse. Goddamn does this suck. Local attacks on a global problem just be like this.

Hmmm... potentially, starting sorting could save me about 2000 moves. Let's give it a shot.

HAHA WE'RE SUB-60K (58940)!!!!!! Nice. Just moving everything up towards the top and sorting by pick frequency really helped. It's not quite as much as I'd hoped, but the net result is that principled optimizations saved me about 10% from my start. Thinking about heuristics for doubling up pallet dragging earlier helped here, I think, since "can I efficiently snag a second pallet while I'm passing by" is a very useful question in pre-sorting.

Now, let's see if there's any juice in smarter allocation of tasks (e.g. sort them by marginal extra movement needed) and in assigning tasks centrally.

...aaaaaand nope. I bet for a smaller number of larger tasks, centralized assignment would help, but since each robot does a large number of small tasks, my naive approach turns out to be the best. Let's try some Claude-suggested changes to the planner (windowed and conflict based replanning)