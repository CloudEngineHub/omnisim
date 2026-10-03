"""The OmniLink development loop: measure, fix, measure again -- fast.

A blind campaign (7 arms x 5 repeats, 4-5 h) is how a result is PUBLISHED.
It is the wrong tool for finding out what to fix next. This loop runs only
OmniLink, on shifts that have already been seen, all at once, and compares
it with STORED competitor baselines. The competitors do not change while
OmniLink is being improved, so measuring them again every loop is waste.

    # one-time (or after shared code changed): store competitor baselines
    python tests/benchmarks/robot_ops/devloop/devloop.py baseline holdout_v2 \\
        tests/benchmarks/robot_ops/evidence/shift-holdout-v2-wave*  ...-rerun

    # every loop: OmniLink on every seen shift at once, then the comparison
    python tests/benchmarks/robot_ops/devloop/devloop.py run --key-file O:/omnilink-keys/omni_key.txt

    # re-print a finished loop
    python tests/benchmarks/robot_ops/devloop/devloop.py report <loop dir>

⚠️ DEVELOPMENT SIGNAL ONLY. Every shift here has been seen, and OmniLink is
being improved against them, so a gap here can be overfitting. A number is
publishable only from a fresh blind holdout under a preregistration
(shift/PREREGISTRATION_V*.md). The sealed v3 holdout is deliberately NOT in
SHIFTS and must never be added.

⚠️ STALE BASELINES. The competitors share code with OmniLink (the robot
bridge, the parser their interruption classifier calls, the gate). Each
baseline stores the hash of that shared code; the report says when it has
changed since, and `baseline` should then be re-run from a fresh competitor
run before trusting the gap.
"""
from __future__ import annotations

import argparse
import datetime as _dt
import hashlib
import json
import os
import statistics
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROBOT_OPS = HERE.parent
ROOT = ROBOT_OPS.parents[2]
sys.path.insert(0, str(ROBOT_OPS))
sys.path.insert(0, str(ROOT))
import analyze_shift as A  # noqa: E402

BASELINES = HERE / "baselines.json"
# The interpreter the campaigns use: the benchmark venv carries the competitor
# frameworks (LangGraph). The system Python does not, and a langgraph arm run
# from it errors in ten seconds (probes-20261001-030800-first).
_VENV = ROOT / "tests" / "benchmarks" / "harness_comparison" / ".venv" / "Scripts" / "python.exe"
PY = str(_VENV) if _VENV.exists() else sys.executable
LOOPS = ROBOT_OPS / "evidence" / "devloop"

# Seen shifts only. Drops are each shift's own preregistered drops.
SHIFTS = {
    "dev_v1": {"suite": "tests/benchmarks/robot_ops/suites/shift_dev_v1.json", "drop": []},
    "holdout_v1": {"suite": "tests/benchmarks/robot_ops/suites/shift_holdout_v1.json", "drop": ["5", "10"]},
    "holdout_v2": {"suite": "tests/benchmarks/robot_ops/suites/shift_holdout_v2.json", "drop": ["3", "8"]},
}
COMPETITORS = ("plain_basic", "langgraph_basic", "lobster_basic",
               "plain_full", "langgraph_full", "lobster_full")

# Code every arm runs through, not only OmniLink. A change here can move the
# competitors' scores too, so a baseline measured before it is stale.
SHARED_CODE = (
    "packages/omnisim-bridges/src/omnisim_bridges/interpret.py",
    "packages/omnisim-bridges/src/omnisim_bridges/route.py",
    "packages/omnisim-bridges/src/omnisim_bridges/gate.py",
    "packages/omnisim-bridges/src/omnisim_bridges/intents.py",
    "packages/omnisim-bridges/src/omnisim_bridges/navigation.py",
    "projects/samples/demos/controllers/omnilink_mobile_bridge/omnilink_mobile_bridge.py",
    "omnisim/ops_bench/competitors.py",
    "omnisim/ops_bench/session.py",
    "omnisim/ops_bench/suite.py",
)

# The gap at which a blind campaign is worth its 4-5 hours (owner, 2026-10-01).
READY_GAP, READY_REPEATS = 0.15, 3


def _env(model_delay: float = 0.0) -> dict:
    """The child environment. model_delay > 0 sets the relay's test hatch
    OMNILINK_TEST_MODEL_DELAY_S, adding that many seconds to every OmniLink
    model round: a slow provider, on demand (2026-10-01). It slows OmniLink's
    relay only -- the competitors' adapters do not read it -- so a slow-model
    run is an OmniLink robustness probe, not a comparison."""
    env = dict(os.environ)
    if model_delay and model_delay > 0:
        env["OMNILINK_TEST_MODEL_DELAY_S"] = str(model_delay)
    else:
        env.pop("OMNILINK_TEST_MODEL_DELAY_S", None)
    return env


def shared_hash() -> str:
    h = hashlib.sha256()
    for rel in SHARED_CODE:
        p = ROOT / rel
        h.update(rel.encode())
        h.update(p.read_bytes() if p.exists() else b"<missing>")
    return h.hexdigest()[:16]


def _rows(shift: str, dirs, keep_errors: bool = False):
    """Scored rows. An ERROR row (a model-provider failure somewhere in the
    shift) is dropped from baselines and from READY, but a dev loop still
    wants to SEE it: one provider hiccup at minute 9 should not erase what
    the other 29 minutes measured (loop1, holdout_v1)."""
    A.DROPPED = set(SHIFTS[shift]["drop"])
    rows = [r for r in A.load([str(d) for d in dirs]) if keep_errors or r.get("outcome") != "ERROR"]
    return rows


def _summ(rows):
    return {"scores": [round(A.score(r), 4) for r in rows],
            "unsafe": [A.unsafe_count(r) for r in rows],
            "failed": [sorted({x.split(":")[0] for x in r.get("reasons") or []
                               if x.split(":")[0].isdigit() and x.split(":")[0] not in A.DROPPED},
                              key=int) for r in rows]}


# ── baseline ──────────────────────────────────────────────────────────────

def cmd_baseline(args):
    data = json.loads(BASELINES.read_text(encoding="utf-8")) if BASELINES.exists() else {}
    rows = _rows(args.shift, args.dirs)
    shift = data.setdefault(args.shift, {})
    heads = set()
    for d in args.dirs:
        m = Path(d) / "manifest.json"
        if m.exists():
            heads.add(json.loads(m.read_text(encoding="utf-8")).get("git_head", "")[:9])
    for arm in COMPETITORS:
        mine = [r for r in rows if r["arm"] == arm]
        if not mine:
            continue
        shift[arm] = {**_summ(mine),
                      "source": sorted(str(Path(d).as_posix()) for d in args.dirs),
                      "code_commit": sorted(heads),
                      "shared_hash": args.shared_hash or None,
                      "note": args.note or ""}
    BASELINES.write_text(json.dumps(data, indent=1) + "\n", encoding="utf-8")
    for arm, b in shift.items():
        print(f"{args.shift:11} {arm:16} n={len(b['scores'])} mean={statistics.mean(b['scores']):.3f} "
              f"unsafe={sum(b['unsafe'])}")


# ── run ───────────────────────────────────────────────────────────────────

def cmd_run(args):
    shifts = list(SHIFTS) if args.shifts == "all" else args.shifts.split(",")
    stamp = _dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    loop = LOOPS / f"{stamp}{('-' + args.label) if args.label else ''}"
    loop.mkdir(parents=True, exist_ok=True)
    head = subprocess.run(["git", "rev-parse", "--short=9", "HEAD"], cwd=ROOT,
                          capture_output=True, text=True).stdout.strip()
    dirty = subprocess.run(["git", "status", "--porcelain", "--", "packages/omnisim-bridges/src",
                            "projects/samples/demos/controllers/omnilink_mobile_bridge", "omnisim"],
                           cwd=ROOT, capture_output=True, text=True).stdout.strip()
    meta = {"started": stamp, "git_head": head, "dirty_product": dirty.splitlines(),
            "shared_hash": shared_hash(), "shifts": shifts, "repeats": args.repeats,
            "label": args.label, "model_delay_s": args.model_delay}
    (loop / "loop.json").write_text(json.dumps(meta, indent=1), encoding="utf-8")
    procs = []
    for shift in shifts:
        for rep in range(args.repeats):
            out = loop / f"{shift}-r{rep}"
            cmd = [PY, "-m", "omnisim", "ops-bench", "run", "--suite", SHIFTS[shift]["suite"],
                   "--arms", "omnilink", "--repeats", "1", "--repeat-base", str(rep),
                   "--engine", args.engine, "--model", args.model, "--rates", args.rates,
                   "--max-requests", "1000", "--cap-usd", str(args.cap_usd),
                   "--key-file", args.key_file, "--out", str(out)]
            log = open(loop / f"{shift}-r{rep}.log", "w", encoding="utf-8")
            procs.append((shift, out, subprocess.Popen(cmd, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                                                       env=_env(args.model_delay))))
    print(f"loop {loop.name}: {len(procs)} OmniLink episode(s) running at once "
          f"({', '.join(shifts)}; head {head}{', DIRTY product' if dirty else ''})", flush=True)
    for shift, out, p in procs:
        p.wait()
        print(f"  {out.name}: exit {p.returncode}", flush=True)
    for shift, out, _ in procs:
        if (out / "rows.jsonl").exists() and _has_judged(shift):
            subprocess.run([PY, "-m", "omnisim", "ops-bench", "judge", str(out),
                            "--suite", SHIFTS[shift]["suite"], "--engine", args.engine,
                            "--model", args.judge_model, "--key-file", args.key_file], cwd=ROOT)
    report(loop)


def cmd_confirm(args):
    """The step before a blind campaign: on each seen shift, OmniLink
    `--omnilink-repeats` times AND the full-tier competitors, in one wave per
    shift. The competitors' rows become their baselines, stamped with the
    CURRENT shared-code hash -- so the READY verdict compares OmniLink with
    competitors running today's shared code, not last week's."""
    shifts = list(SHIFTS) if args.shifts == "all" else args.shifts.split(",")
    stamp = _dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    loop = LOOPS / f"{stamp}-confirm{('-' + args.label) if args.label else ''}"
    loop.mkdir(parents=True, exist_ok=True)
    head = subprocess.run(["git", "rev-parse", "--short=9", "HEAD"], cwd=ROOT,
                          capture_output=True, text=True).stdout.strip()
    dirty = subprocess.run(["git", "status", "--porcelain", "--", "packages/omnisim-bridges/src",
                            "projects/samples/demos/controllers/omnilink_mobile_bridge", "omnisim"],
                           cwd=ROOT, capture_output=True, text=True).stdout.strip()
    meta = {"started": stamp, "git_head": head, "dirty_product": dirty.splitlines(),
            "shared_hash": shared_hash(), "shifts": shifts, "repeats": args.omnilink_repeats,
            "label": "confirm", "competitors": args.competitors.split(",")}
    (loop / "loop.json").write_text(json.dumps(meta, indent=1), encoding="utf-8")
    # Two waves per shift (2026-10-01): OmniLink's repeats, then the
    # competitors. The first confirm round ran all six at once with no health
    # gate; the provider slowed (median 11-16 s a request, p90 55 s) and the
    # round measured the provider, not the agents. Each wave now waits for a
    # healthy provider (median <= --health-median-s over 5 probes).
    waves = [("omnilink", ["omnilink"] * args.omnilink_repeats),
             ("competitors", args.competitors.split(","))]
    for shift in shifts:
        out = loop / f"{shift}-r0"
        for wave, arms in waves:
            if not arms or arms == [""]:
                continue
            wout = out / wave
            cmd = [PY, "-m", "omnisim", "ops-bench", "run", "--suite", SHIFTS[shift]["suite"],
                   "--arms", *arms, "--parallel", str(len(arms)), "--repeats", "1", "--repeat-base", "0",
                   "--engine", args.engine, "--model", args.model, "--rates", args.rates,
                   "--max-requests", "1000", "--cap-usd", str(args.cap_usd),
                   "--health-median-s", str(args.health_median_s), "--health-max-wait-h", "2",
                   "--key-file", args.key_file, "--out", str(wout)]
            print(f"confirm {loop.name}: {shift} / {wave}, {len(arms)} episodes at once "
                  f"({', '.join(arms)})", flush=True)
            with open(loop / f"{shift}-r0-{wave}.log", "w", encoding="utf-8") as log:
                subprocess.run(cmd, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, env=_env(0))
            if (wout / "rows.jsonl").exists() and _has_judged(shift):
                subprocess.run([PY, "-m", "omnisim", "ops-bench", "judge", str(wout),
                                "--suite", SHIFTS[shift]["suite"], "--engine", args.engine,
                                "--model", args.judge_model, "--key-file", args.key_file], cwd=ROOT)
        comp = out / "competitors"
        if (comp / "rows.jsonl").exists():
            cmd_baseline(argparse.Namespace(shift=shift, dirs=[str(comp)], shared_hash=meta["shared_hash"],
                                            note=f"confirm round {loop.name} (head {head}"
                                                 f"{', dirty product' if dirty else ''})"))
    report(loop)


PROBES = "tests/benchmarks/robot_ops/suites/probes_v1.json"


def cmd_probes(args):
    """Tier 1: the short probe pack (one scenario per known failure class),
    every probe's arms at once. Minutes, not hours."""
    stamp = _dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    out = LOOPS / f"probes-{stamp}{('-' + args.label) if args.label else ''}"
    arms = args.arms.split(",")
    cmd = [PY, "-m", "omnisim", "ops-bench", "run", "--suite", PROBES,
           "--arms", *arms, "--parallel", str(len(arms)), "--engine", args.engine,
           "--model", args.model, "--rates", args.rates, "--max-requests", "200",
           "--cap-usd", str(args.cap_usd), "--key-file", args.key_file, "--out", str(out)]
    if args.only:
        cmd += ["--only", *args.only.split(",")]
    print(f"probes {out.name}: arms {', '.join(arms)}"
          + (f", model slowed by {args.model_delay:g} s a round" if args.model_delay else ""), flush=True)
    subprocess.run(cmd, cwd=ROOT, stdout=subprocess.DEVNULL if args.quiet else None,
                   env=_env(args.model_delay))
    probes_report(out)


def probes_report(out: Path):
    rows = [json.loads(l) for l in (out / "rows.jsonl").read_text(encoding="utf-8").splitlines() if l.strip()]
    arms = sorted({r["arm"] for r in rows}, key=lambda a: (a != "omnilink", a))
    tasks = sorted({r["task"] for r in rows})
    print()
    print(f"{'probe':34}" + "".join(f"{a[:14]:>16}" for a in arms))
    for t in tasks:
        cells = []
        for a in arms:
            r = next((x for x in rows if x["task"] == t and x["arm"] == a), None)
            if r is None:
                cells.append("-")
                continue
            failed = sorted({x.split(":")[0] for x in r.get("reasons") or [] if x.split(":")[0].isdigit()}, key=int)
            word = r["outcome"] + ("!" if r.get("unsafe") else "")
            cells.append(word if not failed else f"{word} {','.join(failed)}")
        print(f"{t[:34]:34}" + "".join(f"{c[:15]:>16}" for c in cells))
    print("(FAIL n,m = failed check indices; ! = unsafe)")


def _has_judged(shift):
    t = json.loads((ROOT / SHIFTS[shift]["suite"]).read_text(encoding="utf-8"))["tasks"][0]
    return any(c["type"] == "judged" for c in t["checks"])


# ── report ────────────────────────────────────────────────────────────────

def report(loop: Path):
    meta = json.loads((loop / "loop.json").read_text(encoding="utf-8"))
    base = json.loads(BASELINES.read_text(encoding="utf-8")) if BASELINES.exists() else {}
    out = {"loop": loop.name, "git_head": meta["git_head"], "shifts": {}}
    lines = [f"\nDEV LOOP {loop.name}  (head {meta['git_head']}"
             f"{', product DIRTY' if meta.get('dirty_product') else ''})  -- development signal only\n",
             f"{'shift':11} {'OmniLink':>9} {'unsafe':>6} | {'best competitor':>16} {'mean':>6} {'unsafe':>6} | {'gap':>6}  baseline"]
    ready = True
    for shift in meta["shifts"]:
        dirs = sorted(loop.glob(f"{shift}-r*/")) + sorted(loop.glob(f"{shift}-r*/omnilink/"))
        all_rows = [r for r in _rows(shift, [d for d in dirs if (d / 'rows.jsonl').exists()], keep_errors=True)
                    if r["arm"] == "omnilink"]
        rows = [r for r in all_rows if r.get("outcome") != "ERROR"]
        errored = [r for r in all_rows if r.get("outcome") == "ERROR"]
        o = _summ(rows)
        if not rows and errored:
            # Only errored episodes: show them, marked, but they never count to READY.
            o = _summ(errored)
            o["provider_error"] = True
        b = base.get(shift, {})
        best = max(b.items(), key=lambda kv: statistics.mean(kv[1]["scores"])) if b else None
        om = statistics.mean(o["scores"]) if o["scores"] else None
        stale = bool(best) and best[1].get("shared_hash") not in (None, meta["shared_hash"])
        unknown = bool(best) and best[1].get("shared_hash") is None
        rec = {"omnilink": o, "best": None, "errored_episodes": len(errored)}
        if best:
            bm = statistics.mean(best[1]["scores"])
            gap = (om - bm) if om is not None else None
            rec["best"] = {"arm": best[0], "mean": round(bm, 4), "unsafe": best[1]["unsafe"],
                           "stale": stale, "hash_unknown": unknown}
            rec["gap"] = None if gap is None else round(gap, 4)
            # Checks OmniLink failed that the best competitor passed in most runs.
            b_fail = best[1]["failed"]
            maj = {k for k in {x for f in b_fail for x in f}
                   if sum(k in f for f in b_fail) > len(b_fail) / 2}
            o_fail = {x for f in o["failed"] for x in f}
            rec["omnilink_behind_on"] = sorted(o_fail - maj, key=int)
            rec["omnilink_ahead_on"] = sorted(maj - o_fail, key=int)
            tag = "STALE (shared code changed)" if stale else ("hash unknown" if unknown else "current")
            lines.append(f"{shift:11} {om if om is not None else float('nan'):9.3f} {sum(o['unsafe']):6d} | "
                         f"{best[0]:>16} {bm:6.3f} {sum(best[1]['unsafe']):6d} | "
                         f"{(gap if gap is not None else float('nan')):+6.3f}  {tag}, n={len(best[1]['scores'])}")
            lines.append(f"{'':11} behind on checks {rec['omnilink_behind_on'] or '-'}; "
                         f"ahead on {rec['omnilink_ahead_on'] or '-'}"
                         + (f"  [{len(errored)} episode(s) hit a provider error"
                            + ("; shown, not counted]" if o.get("provider_error") else "; excluded]")
                            if errored else ""))
            if (gap is None or gap < READY_GAP or sum(o["unsafe"]) > 0 or stale or unknown
                    or o.get("provider_error")):
                ready = False
        else:
            lines.append(f"{shift:11} {om if om is not None else float('nan'):9.3f} {sum(o['unsafe']):6d} | "
                         f"{'(no baseline)':>16}")
            ready = False
        out["shifts"][shift] = rec
    n = min((0 if out["shifts"][s]["omnilink"].get("provider_error") else
             len(out["shifts"][s]["omnilink"]["scores"]) for s in meta["shifts"]), default=0)
    if n < READY_REPEATS:
        ready = False
    out["ready_for_blind_campaign"] = ready
    lines.append(f"\nREADY FOR A BLIND CAMPAIGN: {'YES' if ready else 'no'} "
                 f"(needs gap >= {READY_GAP} on every shift, 0 unsafe, current baselines, "
                 f"{READY_REPEATS}+ OmniLink repeats; this loop has {n})")
    (loop / "report.json").write_text(json.dumps(out, indent=1), encoding="utf-8")
    print("\n".join(lines))
    return out


def cmd_explain(args):
    """Why did OmniLink fail check N? The check, what the grader measured,
    and what OmniLink said and did in that check's window."""
    loop = Path(args.loop)
    suite = json.loads((ROOT / SHIFTS[args.shift]["suite"]).read_text(encoding="utf-8"))["tasks"][0]
    steps = {s["id"]: s for s in suite["steps"]}
    order = [s["id"] for s in suite["steps"]]
    for d in sorted(loop.glob(f"{args.shift}-r*/")):
        rows = [r for r in _rows(args.shift, [d], keep_errors=True) if r["arm"] == "omnilink"]
        for r in rows:
            for n in args.checks:
                c = suite["checks"][int(n)]
                key = next((k for k in r["checks"] if k.split(":")[0] == str(n)), None)
                why = [x for x in r.get("reasons") or [] if x.split(":")[0] == str(n)]
                print()
                print(f"=== {d.name} check {n}: {json.dumps(c)[:300]}")
                print(f"    {'FAILED ' + str(why) if why else 'passed'}; measured: {json.dumps(r['checks'].get(key))[:300]}")
                lo = order.index(c["from"]) if c.get("from") in order else 0
                hi = order.index(c["to"]) if c.get("to") in order else (lo + 3 if c.get("from") in order else len(order) - 1)
                if c.get("step") in order:
                    lo = hi = order.index(c["step"])
                for sid in order[max(0, lo - 1):hi + 1]:
                    v = r["replies"].get(sid)
                    st = steps[sid]
                    if not v:
                        print(f"    [{sid}] {'fixture ' + json.dumps(st.get('fixture')) if 'fixture' in st else '(not fired / no reply)'}")
                        continue
                    raw = v.get("raw") or {}
                    acts = "; ".join(f"{a.get('tool')}={a.get('result')}:{str(a.get('summary'))[:60]}"
                                     for a in (raw.get("actions") or [])[:4])
                    print(f"    [{sid}] t={v['t_sent']:.0f}->{(v.get('t_done') or 0):.0f} via={raw.get('via', '-')}")
                    print(f"        said: {v['prompt'][:160]}")
                    print(f"        reply: {(v.get('text') or '')[:220]}")
                    if acts:
                        print(f"        acts: {acts}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    b = sub.add_parser("baseline", help="store competitor scores for a seen shift from evidence dirs")
    b.add_argument("shift", choices=list(SHIFTS))
    b.add_argument("dirs", nargs="+")
    b.add_argument("--shared-hash", default="", help="hash of the shared code those runs used, if known")
    b.add_argument("--note", default="")
    r = sub.add_parser("run", help="OmniLink on every seen shift at once, then compare")
    r.add_argument("--shifts", default="all")
    r.add_argument("--repeats", type=int, default=1)
    r.add_argument("--key-file", required=True)
    r.add_argument("--engine", default="g1-engine")
    r.add_argument("--model", default="gemini-3.5-flash")
    r.add_argument("--judge-model", default="gemini-2.5-pro")
    r.add_argument("--rates", default="1.5,0.15,9.0")
    r.add_argument("--cap-usd", type=float, default=6.0, help="per episode process")
    r.add_argument("--label", default="")
    r.add_argument("--model-delay", type=float, default=0.0,
                   help="slow every OmniLink model round by this many seconds (robustness probe)")
    c = sub.add_parser("confirm", help="OmniLink x N plus fresh full-tier baselines on every seen shift")
    c.add_argument("--shifts", default="all")
    c.add_argument("--omnilink-repeats", type=int, default=3)
    c.add_argument("--competitors", default="plain_full,langgraph_full,lobster_full")
    c.add_argument("--key-file", required=True)
    c.add_argument("--engine", default="g1-engine")
    c.add_argument("--model", default="gemini-3.5-flash")
    c.add_argument("--judge-model", default="gemini-2.5-pro")
    c.add_argument("--rates", default="1.5,0.15,9.0")
    c.add_argument("--cap-usd", type=float, default=30.0, help="per shift wave")
    c.add_argument("--health-median-s", type=float, default=8.0,
                   help="wait before each wave until the provider's median request time is at most this")
    c.add_argument("--label", default="")
    q = sub.add_parser("probes", help="Tier 1: the short probe pack, in minutes")
    q.add_argument("--arms", default="omnilink")
    q.add_argument("--only", default="")
    q.add_argument("--key-file", required=True)
    q.add_argument("--engine", default="g1-engine")
    q.add_argument("--model", default="gemini-3.5-flash")
    q.add_argument("--rates", default="1.5,0.15,9.0")
    q.add_argument("--cap-usd", type=float, default=3.0)
    q.add_argument("--label", default="")
    q.add_argument("--quiet", action="store_true")
    q.add_argument("--model-delay", type=float, default=0.0,
                   help="slow every OmniLink model round by this many seconds (robustness probe)")
    pr = sub.add_parser("probes-report", help="re-print a finished probe run")
    pr.add_argument("dir")
    p = sub.add_parser("report", help="re-print a finished loop")
    p.add_argument("loop")
    p_hash = sub.add_parser("hash", help="print the current shared-code hash")
    e = sub.add_parser("explain", help="why OmniLink failed given checks in a loop")
    e.add_argument("loop")
    e.add_argument("shift", choices=list(SHIFTS))
    e.add_argument("checks", nargs="+")
    a = ap.parse_args(argv)
    if a.cmd == "baseline":
        cmd_baseline(a)
    elif a.cmd == "run":
        cmd_run(a)
    elif a.cmd == "confirm":
        cmd_confirm(a)
    elif a.cmd == "probes":
        cmd_probes(a)
    elif a.cmd == "probes-report":
        probes_report(Path(a.dir))
    elif a.cmd == "explain":
        cmd_explain(a)
    elif a.cmd == "report":
        report(Path(a.loop))
    else:
        print(shared_hash())


if __name__ == "__main__":
    main()
