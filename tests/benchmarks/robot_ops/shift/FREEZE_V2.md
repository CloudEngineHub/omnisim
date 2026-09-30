> **DRAFT, NOT FROZEN.** The `{{...}}` fields are filled in, and this line
> removed, only after a clean calibration round on a quiet machine
> (`shift-v2-calibration-04` was void: see its NOTE.md).

# FREEZE — shift v2

Frozen {{DATE}}, before any agent ran the v2 shift, under
[PREREGISTRATION_V2.md](PREREGISTRATION_V2.md). Nothing below changes until
the campaign ends.

| Item | Value |
|---|---|
| Code commit (product, bridges, runner, adapters, grader, judge, analysis) | `{{COMMIT}}` |
| Product code | unchanged since shift v1's result (`1c070b71b`) except the v2 fairness changes, which make the competitors stronger: `64452e809` (`GET /main_task` and the full tier's interrupt rule) and `be93bbf4f` (a runner default for a bare `park` fixture) |
| Holdout | `suites/shift_holdout_v2.json`, SHA-256 `{{SHA}}`, written blind by a fresh Opus context under `AUTHOR_BRIEF_V2.md` |
| Calibration | `evidence/shift-v2-calibration-02` and `{{CAL}}`: `oracle` passes every check (judged included) in every run; `oracle_bad` fails every check except those listed below |
| Dropped checks | `{{DROPS}}` |
| Arms | `omnilink`, `plain_basic`, `langgraph_basic`, `lobster_basic`, `plain_full`, `langgraph_full`, `lobster_full` |
| Repeats | 5: five waves, one after another, one process of 7 each (`--parallel 7 --repeat-base r`), then one re-run wave of every ERROR episode |
| Model | `g1-engine` / `gemini-3.5-flash` for every arm (Vertex AI, location `global`) |
| Judge | `g1-engine` / `gemini-2.5-pro`, prompt `JUDGE_SYSTEM` in `omnisim/ops_bench/judge.py` |
| Rates | 1.50 / 0.15 / 9.00 USD per million tokens (input / cached / output including thinking) |
| Spend cap | $95 across the waves, re-runs and judge |
| Competitor limits | whole conversation (cap 600); 1,000 requests per episode; stack recorded per run (`competitor_stack`: LangGraph 1.2.12, Lobster `71bd814`, Node 22.19) |
| Health gate | 5 probes before each wave; all must answer, median at most 8 s; wait up to 2 h |
| Machine | `9722d23d12a3`, AMD64 Family 25 Model 80, 16 cores, RTX 3060 Laptop, Windows 11 |
| Decision rule | exact one-sided Mann-Whitney p ≤ 0.05, unsafe at most the other's, no episode left ERROR; claims A and B against every arm of their tier |
| Command | `bash tests/benchmarks/robot_ops/shift/run_campaign_v2.sh <OmniKey file>` with `DROP={{DROPS}}` |

## Disclosures

- **What the developer saw of the v2 holdout before the freeze:** its
  calibration outputs. That is check ids and names, pass counts, and the
  robot's measured positions and tool results in calibration runs. No
  messages, rubrics, names or stations. The author's reports held counts and
  checksums only.
- **The first calibration run found a runner bug,** not a script defect. A
  bare `park` fixture had no default prop, and every run died. It was fixed
  in `be93bbf4f`, which is in the frozen commit.
- **Every author and the judge are AI models,** and the benchmark, grader,
  adapters and OmniLink come from one party. See PREREGISTRATION_V2.md,
  "Known limits".
