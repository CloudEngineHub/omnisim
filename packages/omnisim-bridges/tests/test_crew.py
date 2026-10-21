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
"""The crew channel (omnisim_bridges.crew): robots messaging each other."""
from __future__ import annotations

import json
import threading

from omnisim_bridges import crew


class _Bridge:
    sim_time = 12.0


def test_the_roster_parses_and_skips_malformed_entries():
    assert crew.crew_map("Picker=8765, Tug=8766,bad,Packer=x,=9,Arm=0") == {"Picker": 8765, "Tug": 8766}


def test_a_bridge_finds_itself_by_port_and_lists_only_the_others():
    tools = crew.crew_tools(_Bridge(), 8766, spec="Picker=8765,Tug=8766,Packer=8767")
    assert [t.name for t in tools] == ["message_robot"]
    assert tools[0].physical is False
    assert "You are Tug" in tools[0].description and "Packer, Picker" in tools[0].description


def test_no_crew_no_tool():
    assert crew.crew_tools(_Bridge(), 8766, spec="") == []
    assert crew.crew_tools(_Bridge(), 9999, spec="Picker=8765,Tug=8766") == []
    assert crew.crew_tools(_Bridge(), 8765, spec="Picker=8765") == []


def _crew(sent):
    done = threading.Event()

    def post(port, text):
        sent.append((port, text))
        done.set()
        return {"response": "on it", "via": "model"}
    c = crew.Crew("Picker", {"Tug": 8766, "Packer": 8767}, lambda: 3.0, post=post, operator="Sam")
    return c, done


def test_a_message_is_delivered_as_the_senders_own_words(tmp_path, monkeypatch):
    monkeypatch.setenv("OMNILINK_CREW_LOG", str(tmp_path / "crew.jsonl"))
    sent = []
    c, done = _crew(sent)
    out = c.message("tug", "  Please bring   the cart. ")
    assert out["sent"] and out["to"] == "Tug"
    assert done.wait(5)
    assert sent == [(8766, "Picker: Please bring the cart.")]
    rows = [json.loads(x) for x in (tmp_path / "crew.jsonl").read_text(encoding="utf-8").splitlines()]
    assert rows[0]["kind"] == "message" and rows[0]["from"] == "Picker" and rows[0]["sim_s"] == 3.0


def test_the_operator_inbox_logs_and_delivers_nothing(tmp_path, monkeypatch):
    monkeypatch.setenv("OMNILINK_CREW_LOG", str(tmp_path / "crew.jsonl"))
    sent = []
    c, _ = _crew(sent)
    assert c.message("Sam", "Order 42 is packed.")["sent"]
    assert sent == []
    assert json.loads((tmp_path / "crew.jsonl").read_text(encoding="utf-8"))["to"] == "Sam"


def test_unknown_crewmate_and_empty_message_are_refused():
    c, _ = _crew([])
    assert c.message("forklift", "hi")["error"] == "unknown_crewmate"
    assert c.message("Tug", "   ")["error"] == "empty_message"


def test_the_rate_cap_stops_a_thank_you_loop():
    c, _ = _crew([])
    results = [c.message("Packer", f"thanks {i}") for i in range(crew.CREW_RATE_PER_MIN + 2)]
    assert [r["sent"] for r in results].count(True) == crew.CREW_RATE_PER_MIN
    assert results[-1]["error"] == "crew_channel_rate_limited"


def test_a_delivery_is_marked_crew_so_the_receiver_skips_its_parser(monkeypatch):
    seen = {}

    class _Resp:
        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

        def read(self):
            return b'{"response": "ok"}'

    def fake_urlopen(req, timeout=None):
        seen["url"], seen["body"] = req.full_url, json.loads(req.data.decode())
        return _Resp()
    monkeypatch.setattr(crew.urllib.request, "urlopen", fake_urlopen)
    assert crew._post_prompt(8767, "Picker: hello") == {"response": "ok"}
    assert seen["url"] == "http://127.0.0.1:8767/prompt"
    assert seen["body"]["crew"] is True and seen["body"]["text"] == "Picker: hello"
