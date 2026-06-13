import subprocess
import textwrap
from pathlib import Path


def test_health_core_applies_stale_and_degeneracy_rules(tmp_path):
    repo = Path(__file__).resolve().parents[4]
    include_dir = repo / "src/frc/frc_nodes_cpp/include"
    source = tmp_path / "health_core_contract.cpp"
    binary = tmp_path / "health_core_contract"

    source.write_text(textwrap.dedent(
        """
        #include <cassert>
        #include <cmath>
        #include "frc_nodes_cpp/health_aggregator_core.hpp"

        int main() {
          frc_nodes_cpp::HealthAggregatorCore core(1.0, 75.0);
          frc_nodes_cpp::HealthInputs inputs;

          inputs.degeneracy = {9.7, {80.0F, 12.0F, 0.0F}};
          inputs.correction = {9.6, {1.0F, 0.42F}};
          inputs.fix_status = {9.5, 2};
          inputs.velocity = {9.4, {0.3F, -0.1F}};

          const auto fresh = core.build(inputs, 10.0);
          assert(std::fabs(fresh.lio_min_eig - 80.0F) < 1e-5F);
          assert(!fresh.lio_degenerate);
          assert(fresh.pgo_correcting);
          assert(std::fabs(fresh.pgo_last_jump - 0.42F) < 1e-5F);
          assert(fresh.rtk_status == 2);
          assert(std::fabs(fresh.v - 0.3F) < 1e-5F);
          assert(std::fabs(fresh.w + 0.1F) < 1e-5F);

          inputs.degeneracy = {10.0, {70.0F, 20.0F, 0.0F}};
          const auto low_eig = core.build(inputs, 10.1);
          assert(low_eig.lio_degenerate);

          inputs.degeneracy = {8.0, {70.0F, 20.0F, 1.0F}};
          inputs.fix_status = {8.0, 2};
          const auto stale = core.build(inputs, 10.1);
          assert(std::fabs(stale.lio_min_eig) < 1e-5F);
          assert(!stale.lio_degenerate);
          assert(stale.rtk_status == -1);
          return 0;
        }
        """
    ))

    subprocess.run(
        ["g++", "-std=c++17", "-I", str(include_dir), str(source), "-o", str(binary)],
        check=True,
    )
    subprocess.run([str(binary)], check=True)
