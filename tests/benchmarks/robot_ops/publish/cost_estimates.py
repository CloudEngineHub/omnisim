# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Recompute a cost addendum from the two immutable public evidence archives.

python -I -S cost_estimates.py <folder containing evidence.zip and codex-evidence.zip>
python -I -S cost_estimates.py <folder> --check

The graph ranks the original seven API-backed systems' three scored shifts,
including failed checks. Infrastructure errors and the superseded Codex wave
are separate overhead, never erased. Codex dollars are an unranked pricing
scenario on recorded usage, not a signed-in account charge. No network or keys needed.
"""
import hashlib
import json
from pathlib import Path
import sys
import zipfile

RATES = {"input": 2.0, "cached_input": 0.10, "cache_write": 2.50, "output": 10.0}


def sha(data):
    return hashlib.sha256(data).hexdigest()


def estimate(tokens):
    uncached = tokens["inputTokens"] - tokens["cachedInputTokens"] - tokens["cacheWriteInputTokens"]
    if uncached < 0 or tokens["reasoningOutputTokens"] > tokens["outputTokens"]:
        raise ValueError("Inconsistent token categories")
    if tokens["totalTokens"] != tokens["inputTokens"] + tokens["outputTokens"]:
        raise ValueError("Inconsistent total tokens")
    # Cached/write tokens are subsets of input; reasoning is a subset of output.
    return (uncached * RATES["input"] + tokens["cachedInputTokens"] * RATES["cached_input"]
            + tokens["cacheWriteInputTokens"] * RATES["cache_write"]
            + tokens["outputTokens"] * RATES["output"]) / 1_000_000


def calculate(folder):
    folder = Path(folder)
    archives = {name: (folder / name).read_bytes() for name in ("evidence.zip", "codex-evidence.zip")}
    original = json.loads((folder / "results.json").read_text(encoding="utf-8"))
    codex = json.loads((folder / "codex-results.json").read_text(encoding="utf-8"))
    if original["suite_sha256"] != codex["suite_sha256"] or original["repeats"] != 3 or codex["repeats"] != 3:
        raise ValueError("Unexpected suite or repeat count")
    if codex["model"] != "gpt-6.1-sol" or codex["usd_estimated"] is not None:
        raise ValueError("Unexpected Codex model or billing record")
    arms = {}
    with zipfile.ZipFile(folder / "evidence.zip") as z:
        if json.loads(z.read("results.json")) != original:
            raise ValueError("Original results differ from archived evidence")
        rows = [json.loads(line) for name in z.namelist() if name.endswith("/rows.jsonl")
                for line in z.read(name).decode().splitlines() if line]
        for key, published in original["arms"].items():
            attempts = [r for r in rows if r["arm"] == key]
            scored = [r for r in attempts if r["outcome"] != "ERROR"]
            if len(scored) != 3:
                raise ValueError(f"Expected three scored shifts: {key}")
            total = sum(r["cost"]["usd_estimated"] for r in attempts)
            if abs(total - published["usd_all_attempts"]) > 0.0001:
                raise ValueError(f"Published campaign cost mismatch: {key}")
            cost = sum(r["cost"]["usd_estimated"] for r in scored)
            arms[key] = {"model": "gemini-3.5-flash", "scored_shifts": 3,
                         "scored_usd": cost, "usd_per_scored_shift": cost / 3,
                         "overhead_usd": total - cost, "all_attempts_usd": total,
                         "comparable_cost": True,
                         "attempts": len(attempts), "dollar_basis": "Recorded usage at original provider list rates"}
    codex_attempts = []
    with zipfile.ZipFile(folder / "codex-evidence.zip") as z:
        if json.loads(z.read("codex-results.json")) != codex:
            raise ValueError("Codex results differ from archived evidence")
        for name in sorted(z.namelist()):
            if not name.endswith("/rows.jsonl"):
                continue
            r = json.loads(z.read(name))
            run = name.split("/")[-2]
            event_name = name.rsplit("/", 1)[0] + "/episodes/" + r["episode_dir"] + "/codex-events.jsonl"
            events = [json.loads(l) for l in z.read(event_name).decode().splitlines() if l]
            usage = [e["params"]["tokenUsage"] for e in events if e.get("method") == "thread/tokenUsage/updated"]
            total = usage[-1]["total"]
            if total != r["codex_usage"]["tokens"]:
                raise ValueError(f"Native final cumulative usage mismatch: {run}")
            max_input = max(u["last"]["inputTokens"] for u in usage)
            if max_input > 272_000:
                raise ValueError("Long-context pricing requires per-request accounting")
            primary = run in codex["run_dirs"]
            done = [e["params"]["turn"] for e in events if e.get("method") == "turn/completed"]
            turns_with_usage = {e["params"].get("turnId") for e in events
                                if e.get("method") == "thread/tokenUsage/updated"}
            without_usage = [t for t in done if t["id"] not in turns_with_usage]
            if primary:
                ep = next(e for e in codex["episodes"] if e["repeat"] == r["repeat"])
                if ep["usage"]["tokens"] != total or r["outcome"] == "ERROR":
                    raise ValueError("Primary published usage mismatch")
            codex_attempts.append({"run": run, "primary": primary, "outcome": r["outcome"],
                                   "tokens": total, "max_observed_request_input_tokens": max_input,
                                   "elapsed_s": r["elapsed_s"],
                                   "turns_without_individual_usage": len(without_usage),
                                   "interrupted_turns_without_individual_usage": sum(t.get("status") == "interrupted" for t in without_usage),
                                   "standard_api_equivalent_usd": estimate(total)})
    if len(codex_attempts) != 6 or sum(e["primary"] for e in codex_attempts) != 3:
        raise ValueError("Expected both complete Codex waves")
    primary = sum(e["standard_api_equivalent_usd"] for e in codex_attempts if e["primary"])
    overhead = sum(e["standard_api_equivalent_usd"] for e in codex_attempts if not e["primary"])
    arms["codex_full"] = {"model": "gpt-6.1-sol", "scored_shifts": 3, "scored_usd": primary,
                           "usd_per_scored_shift": primary / 3, "overhead_usd": overhead,
                           "all_attempts_usd": primary + overhead, "attempts": 6,
                           "comparable_cost": False,
                           "dollar_basis": "Conditional recorded-token Standard API pricing scenario; excluded from cost ranking; actual signed-in cost unavailable",
                           "billed_usd": None}
    return {"benchmark": "Warehouse shift v1 — cost addendum", "date": "2026-09-30",
            "suite_sha256": original["suite_sha256"],
            "metric": "Mean estimated model USD for the three primary scored shifts; includes failed checks; overhead reported separately",
            "excluded_costs": ["OmniLink subscription", "Codex subscription or credits", "hosting", "local compute", "tax"],
            "codex_estimate_conditions": {"model": "gpt-6.1-sol", "reasoning_effort": "high",
                "service_tier_assumption": "Standard API, no Fast/Ultrafast or regional premium",
                "rates_usd_per_million": RATES, "pricing_checked": "2026-09-30",
                "long_context_threshold": 272000, "observed_cache_write_tokens": 0,
                "accounting": "Latest native cumulative totals per thread, not a sum of repeated usage notifications. Cached and cache-write counts are subsets of input; reasoning is included once in output. One primary interrupted turn had no individual usage event; its billed usage is unknown. Native counters are best-effort, not an invoice. Zero recorded native cache writes do not establish API write charges. Historical billing tier was not preserved; current config cannot establish past billing. These API rates do not estimate included subscription usage. Development and this coding chat's work are excluded.",
                "sources": ["https://developers.openai.com/api/docs/models/gpt-6.1-sol",
                            "https://developers.openai.com/api/docs/guides/prompt-caching",
                            "https://developers.openai.com/api/docs/guides/agents-api/observability",
                            "https://learn.chatgpt.com/docs/pricing"]},
            "original_rates_usd_per_million": {"input": 1.5, "cached_input": 0.15, "output_including_thinking": 9.0},
            "source_archive_sha256": {name: sha(data) for name, data in archives.items()},
            "arms": arms, "codex_attempts": codex_attempts,
            "limits": "Retrospective cost analysis. Only the original seven metered API configurations are ranked. Codex's actual cost is unavailable; its conditional recorded-token pricing scenario does not establish a cost tie or advantage. Same workload, different models for Codex and original seven; later Codex build and concurrency differ. Superseded Codex wave and original error episodes are retained as overhead. Development pilots and this coding chat's work are excluded for all systems."}


if __name__ == "__main__":
    folder = Path(sys.argv[1])
    result = calculate(folder)
    path = folder / "cost-estimates.json"
    if "--check" in sys.argv:
        if json.loads(path.read_text(encoding="utf-8")) != result:
            raise SystemExit("Cost addendum differs from archived evidence")
        print("VERIFIED: costs, all 34 attempts, Codex tokens, cache categories and pricing threshold")
    else:
        path.write_bytes((json.dumps(result, indent=2) + "\n").encode())
    for key, value in sorted(result["arms"].items(), key=lambda pair: pair[1]["usd_per_scored_shift"]):
        print(f"{key}: ${value['usd_per_scored_shift']:.4f}/scored shift; ${value['all_attempts_usd']:.4f} all attempts")
