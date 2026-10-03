# The OmniLink development loop

Owner decision, 2026-10-01: improving OmniLink is a development cycle. A
full blind campaign (seven arms, five repeats, four to five hours, up to
$95) is how a result is **published**, and only once OmniLink is clearly
ahead. Running one to find out what to fix next is waste: the number is
obsolete as soon as the next fix lands.

## The four tiers

| Tier | What runs | Wall time | Cost | When |
|---|---|---|---|---|
| 0 unit | Engine-free tests of the parser, bridge and relay; every failure class found in a trace gets one | seconds | $0 | every change |
| 1 probes | `suites/probes_v1.json`: one short scenario per failure class, OmniLink only (competitors once, as a reference) | minutes | about $1 | every fix |
| 2 seen shifts | the three seen 30-minute shifts (dev v1, holdout v1, holdout v2), OmniLink only, all at once, against stored competitor baselines | about 35 min | about $3-4 | a few times a day |
| 3 blind campaign | a fresh, sealed blind shift under a preregistration, all seven arms | 4-5 h | up to $95 | only to publish |

```bash
# Tier 0
python -m pytest packages/omnisim-bridges/tests -q

# Tier 1
python tests/benchmarks/robot_ops/devloop/devloop.py probes --key-file O:/omnilink-keys/omni_key.txt

# Tier 2
python tests/benchmarks/robot_ops/devloop/devloop.py run --key-file O:/omnilink-keys/omni_key.txt
python tests/benchmarks/robot_ops/devloop/devloop.py report tests/benchmarks/robot_ops/evidence/devloop/<loop>
```

Every run lands in `evidence/devloop/`. The Tier 2 report gives, per shift,
OmniLink's score and unsafe count, the best stored competitor, the gap, and
the check ids OmniLink fails that that competitor passes: the next thing to
fix.

## Why the shifts are not cut into segments

It was the first idea, and it was measured before being built. On all three
seen shifts the checks' windows chain across the whole shift ("reach X any
time after this order", rules in force to the end, an order timed eleven
minutes out, "is that job still on?"), so no shift splits into independent
pieces without dropping or rewriting the checks that make it a shift. The
speed comes instead from running OmniLink alone, every shift at once, and
from the short probes.

## Stored baselines (`baselines.json`)

Competitor scores per seen shift, built with `devloop.py baseline <shift>
<evidence dirs>` and tagged with their source. The competitors share code
with OmniLink (the bridge, the parser their interruption classifier calls,
the gate), so each baseline carries the hash of that code (`devloop.py
hash`). The report marks a baseline **STALE** when that code has changed
since, and **hash unknown** for the baselines imported from the v1 and v2
campaigns, which ran before the 2026-09-30 fixes. Neither counts towards
READY: refresh the full tier with a fresh run first.

## When to run Tier 3

`READY FOR A BLIND CAMPAIGN: YES` needs, on every seen shift: a gap of at
least 0.15 over the best competitor, zero unsafe checks, current baselines
and at least three OmniLink repeats. Then the sealed holdout
(`suites/shift_holdout_v3.json`, written blind on 2026-10-01 and never read)
gets calibrated, frozen and run under `shift/PREREGISTRATION_V3.md`.

## Honesty

Tiers 0-2 use material the developer has seen and is improving against.
Their numbers can reward overfitting, and they are never published. The v3
holdout is deliberately absent from `devloop.py` and must stay so.
