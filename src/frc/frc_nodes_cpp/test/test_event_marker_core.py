import subprocess
import textwrap
from pathlib import Path


def test_event_marker_core_detects_edges_and_stuck(tmp_path):
    repo = Path(__file__).resolve().parents[4]
    include_dir = repo / "src/frc/frc_nodes_cpp/include"
    source = tmp_path / "event_marker_core_contract.cpp"
    binary = tmp_path / "event_marker_core_contract"

    source.write_text(textwrap.dedent(
        """
        #include <cassert>
        #include <string>
        #include <vector>
        #include "frc_nodes_cpp/event_marker_core.hpp"

        int main() {
          frc_nodes_cpp::EventMarkerConfig cfg;
          cfg.stuck_duration_s = 3.0;
          cfg.cooldown_s = 8.0;
          frc_nodes_cpp::EventMarkerCore core(cfg);

          auto first = core.onChassis(0, 0, 1.0);
          assert(first.empty());

          auto takeover = core.onChassis(1, 0, 2.0);
          assert(takeover.size() == 1);
          assert(takeover[0].type == "takeover");
          assert(takeover[0].severity == "gold");
          assert(takeover[0].note == "ctrl_mode 0->1");

          auto no_repeat = core.onChassis(1, 0, 3.0);
          assert(no_repeat.empty());

          auto manual = core.onChassis(1, 1, 4.0);
          assert(manual.size() == 1);
          assert(manual[0].type == "manual");
          assert(manual[0].severity == "gold");
          assert(manual[0].note == "ps2 SELECT");

          auto held_select = core.onChassis(1, 1, 4.1);
          assert(held_select.empty());

          core.onCmd(0.3, 0.0, 10.0);
          core.onOdomVelocity(0.0, 0.0);
          assert(!core.onTick(10.0).has_value());
          assert(!core.onTick(12.9).has_value());
          const auto stuck = core.onTick(13.1);
          assert(stuck.has_value());
          assert(stuck->type == "stuck");
          assert(stuck->severity == "silver");
          assert(stuck->note == "v=0.000 cmd=0.30");

          assert(!core.onTick(13.2).has_value());
          return 0;
        }
        """
    ))

    subprocess.run(
        ["g++", "-std=c++17", "-I", str(include_dir), str(source), "-o", str(binary)],
        check=True,
    )
    subprocess.run([str(binary)], check=True)
