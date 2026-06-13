import subprocess
import textwrap
from pathlib import Path


def test_risk_core_renders_anchor_gaussian_and_keeps_max(tmp_path):
    repo = Path(__file__).resolve().parents[4]
    include_dir = repo / "src/frc/frc_nodes_cpp/include"
    source = tmp_path / "risk_pipeline_core_contract.cpp"
    binary = tmp_path / "risk_pipeline_core_contract"

    source.write_text(textwrap.dedent(
        """
        #include <cassert>
        #include <cmath>
        #include "frc_nodes_cpp/risk_pipeline_core.hpp"

        int main() {
          frc_nodes_cpp::AnchorStateLite confirmed;
          confirmed.state = "confirmed";
          confirmed.severity = 0.8;
          confirmed.p_usable = 0.5;
          assert(std::fabs(frc_nodes_cpp::injectionWeight(confirmed) - 0.4) < 1e-6);

          frc_nodes_cpp::AnchorStateLite stale = confirmed;
          stale.state = "stale";
          assert(std::fabs(frc_nodes_cpp::injectionWeight(stale) - 0.2) < 1e-6);

          frc_nodes_cpp::AnchorStateLite retired = confirmed;
          retired.state = "retired";
          assert(frc_nodes_cpp::injectionWeight(retired) == 0.0);

          frc_nodes_cpp::RiskFrame frame(5, 1.0, 0.0, 0.0);
          frc_nodes_cpp::renderGaussian(
            frame, 2.5, 2.5, 0.8F, 1.0F, 0.7F,
            frc_nodes_cpp::kSourceMapAnchor);
          const auto center = frame.index(2, 2);
          assert(frame.risk[center] > 0.7F);
          assert(std::fabs(frame.confidence[center] - 0.7F) < 1e-6);
          assert(frame.source[center] == frc_nodes_cpp::kSourceMapAnchor);

          frame.risk[center] = 0.95F;
          frame.confidence[center] = 0.3F;
          frame.source[center] = 4;
          frc_nodes_cpp::renderGaussian(
            frame, 2.5, 2.5, 0.2F, 1.0F, 0.9F,
            frc_nodes_cpp::kSourceMapAnchor);
          assert(std::fabs(frame.risk[center] - 0.95F) < 1e-6);
          assert(std::fabs(frame.confidence[center] - 0.3F) < 1e-6);
          assert(frame.source[center] == 4);
          return 0;
        }
        """
    ))

    subprocess.run(
        ["g++", "-std=c++17", "-I", str(include_dir), str(source), "-o", str(binary)],
        check=True,
    )
    subprocess.run([str(binary)], check=True)
