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
"""A crew channel: robots in one world talking to each other in words.

Each robot keeps its own OmniLink agent. `message_robot` delivers a sentence
to a crewmate's own /prompt door, prefixed with the sender's name exactly as
an operator's would be ("Picker: your cart is loaded"), so the crewmate's
parser, model and safety gate treat it like any other speaker. Nothing a
crewmate says bypasses the gate, and nothing it asks for is obeyed blindly.

Declared per world with one environment variable, the same for every robot:

    OMNILINK_CREW="Picker=8765,Tug=8766,Packer=8767"

A bridge finds itself in that map by its own HTTP port. With no map, or a
port that is not in it, no crew tool is registered. OMNILINK_CREW_OPERATOR
names the human the crew reports to ("Sam"): a message to them is logged,
not delivered to any robot.

Delivery is fire-and-forget on a daemon thread: the sender's turn never
waits on the crewmate's, so two robots messaging each other cannot
deadlock. The crewmate's reply to the delivery is logged but NOT handed back
to the sender; to answer, a crewmate calls message_robot itself. Every
message and reply is appended to OMNILINK_CREW_LOG (JSONL) when set.

A per-sender cap (CREW_RATE_PER_MIN, simulator minutes) stops two
agents thanking each other forever.
"""
from __future__ import annotations

import json
import os
import threading
import time
import urllib.request
from typing import Any, Callable, Dict, List, Optional

CREW_RATE_PER_MIN = 8
DELIVERY_TIMEOUT_S = 600.0   # a crewmate's turn can be a whole tow
_log_lock = threading.Lock()


def crew_map(spec: Optional[str] = None) -> Dict[str, int]:
    """"Picker=8765,Tug=8766" -> {"Picker": 8765, "Tug": 8766}. Malformed entries are skipped."""
    out: Dict[str, int] = {}
    for part in (spec if spec is not None else os.environ.get("OMNILINK_CREW", "")).split(","):
        name, _, port = part.partition("=")
        name = name.strip()
        try:
            p = int(port.strip())
        except ValueError:
            continue
        if name and p > 0:
            out[name] = p
    return out


def _log(record: Dict[str, Any]) -> None:
    path = os.environ.get("OMNILINK_CREW_LOG", "")
    if not path:
        return
    line = json.dumps(record, ensure_ascii=False)
    with _log_lock:
        with open(path, "a", encoding="utf-8") as f:
            f.write(line + "\n")


def _post_prompt(port: int, text: str) -> Dict[str, Any]:
    # "crew": the receiving bridge sends it to its model, never answers it from the parser.
    body = json.dumps({"text": text, "crew": True, "timeout_s": DELIVERY_TIMEOUT_S - 20}).encode("utf-8")
    req = urllib.request.Request(f"http://127.0.0.1:{port}/prompt", data=body,
                                 headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=DELIVERY_TIMEOUT_S) as r:
        return json.loads(r.read().decode("utf-8") or "{}")


class Crew:
    def __init__(self, me: str, peers: Dict[str, int], sim_time: Callable[[], float],
                 post: Callable[[int, str], Dict[str, Any]] = _post_prompt, operator: str = "") -> None:
        self.me, self.peers, self.sim_time, self.post = me, dict(peers), sim_time, post
        self.operator = operator.strip()
        self._sent: List[float] = []
        self._lock = threading.Lock()

    def _resolve(self, to: str) -> Optional[str]:
        key = str(to or "").strip().lower()
        if self.operator and key == self.operator.lower():
            return self.operator
        for name in self.peers:
            if name.lower() == key:
                return name
        hits = [n for n in self.peers if key and (key in n.lower() or n.lower() in key)]
        return hits[0] if len(hits) == 1 else None

    def message(self, to: str, message: str) -> Dict[str, Any]:
        text = " ".join(str(message or "").split())
        name = self._resolve(to)
        if name is None:
            return {"sent": False, "error": "unknown_crewmate", "crew": sorted(self.peers),
                    "say": f"There is no crewmate called {to!r}. Crew: {', '.join(sorted(self.peers))}."}
        if not text:
            return {"sent": False, "error": "empty_message"}
        now = float(self.sim_time() or 0.0)
        with self._lock:
            self._sent = [t for t in self._sent if now - t < 60.0]
            if len(self._sent) >= CREW_RATE_PER_MIN:
                return {"sent": False, "error": "crew_channel_rate_limited",
                        "say": "I've sent a lot of messages this minute; I'll hold off unless something changes."}
            self._sent.append(now)
        record = {"kind": "message", "sim_s": round(now, 3), "wall": round(time.time(), 3),
                  "from": self.me, "to": name, "text": text}
        _log(record)
        if name == self.operator:
            return {"sent": True, "to": name, "note": f"{name} has it."}
        threading.Thread(target=self._deliver, args=(name, text, record), daemon=True,
                         name=f"crew-{self.me}-to-{name}").start()
        return {"sent": True, "to": name,
                "note": (f"Delivered to {name}'s own inbox. Its reply does not come back here; "
                         f"if {name} needs to answer, it will message you.")}

    def _deliver(self, name: str, text: str, record: Dict[str, Any]) -> None:
        reply: Dict[str, Any] = {}
        err = ""
        try:
            reply = self.post(self.peers[name], f"{self.me}: {text}")
        except TimeoutError:
            # The crewmate is still working its turn: the message WAS delivered,
            # only its reply did not come back within the budget. Not an error.
            reply = {"reply_timeout": True}
        except Exception as e:  # the crewmate is down or refused the connection
            err = f"{type(e).__name__}: {e}"
        _log({"kind": "reply", "sim_s": round(float(self.sim_time() or 0.0), 3), "wall": round(time.time(), 3),
              "from": name, "to": self.me, "in_reply_to": record["text"],
              "text": str(reply.get("response") or ""), "via": reply.get("via"),
              "actions": reply.get("actions"), "error": err or reply.get("error") or "",
              "reply_timeout": bool(reply.get("reply_timeout"))})


def crew_tools(bridge: Any, http_port: int, spec: Optional[str] = None) -> List[Any]:
    """The crew tool for this bridge, or [] when this world declares no crew it belongs to."""
    try:
        from omnisim_bridges.tool import Tool
    except Exception:
        return []
    peers = crew_map(spec)
    me = next((n for n, p in peers.items() if p == int(http_port)), None)
    if me is None:
        return []
    others = {n: p for n, p in peers.items() if n != me}
    if not others:
        return []
    operator = os.environ.get("OMNILINK_CREW_OPERATOR", "").strip()
    crew = Crew(me, others, lambda: getattr(bridge, "sim_time", 0.0), operator=operator)
    bridge.crew = crew
    roster = ", ".join(sorted(others))
    lead = (f" {operator} is the human shift lead the crew works for: message {operator} to report "
            "that a job they gave is done, or that it cannot be done and why.") if operator else ""
    return [Tool(
        name="message_robot",
        description=(
            f"Talk to another robot on your crew. You are {me}; your crewmates are {roster}. "
            "Each crewmate is a robot with its own mind, body and tools: it hears your sentence "
            f"exactly as '{me}: <message>', decides for itself, and its own safety rules still apply. "
            "Use it to ask a crewmate for something, to tell it something it needs (where you are, "
            "what you loaded, that a job is done), or to answer a crewmate who asked you. "
            "Say concrete things: names, positions in metres, what you actually measured. "
            "When you want a crewmate to DO something, ask it directly in your own words ('please load "
            "the cart now'): its safety gate lets it move on a request, never on a sentence that only "
            "reports something ('the cart is at x ...'), and it refuses wording that relays what someone "
            "else said or reported. "
            "It returns at once; the crewmate's reply does NOT come back to you. If it needs to "
            "answer, it will message you, and you will hear that as a new message. "
            "Do not message just to say thanks or okay, and do not repeat a message you already sent." + lead),
        parameters={"type": "object",
                    "properties": {"to": {"type": "string", "description": f"One of: {roster}{', ' + operator if operator else ''}."},
                                   "message": {"type": "string", "description": "One or two plain sentences."}},
                    "required": ["to", "message"]},
        dispatch=lambda args: crew.message(str(args.get("to") or ""), str(args.get("message") or "")),
        physical=False,
    )]
