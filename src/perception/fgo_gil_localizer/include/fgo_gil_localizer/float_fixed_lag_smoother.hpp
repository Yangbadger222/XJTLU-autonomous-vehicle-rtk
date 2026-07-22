#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <vector>

#include <Eigen/Core>

#include "fgo_gil_localizer/ecef_imu_preintegrator.hpp"
#include "fgo_gil_localizer/gnss_double_difference.hpp"
#include "fgo_gil_localizer/integer_ambiguity_resolver.hpp"
#include "fgo_gil_localizer/lidar_factors.hpp"

namespace fgo_gil_localizer
{

using StateId = std::uint64_t;

struct StateFactorNoise
{
  double position_m = 1.0;
  double rotation_rad = 0.1;
  double velocity_m_s = 1.0;
  double accelerometer_bias_m_s2 = 0.1;
  double gyroscope_bias_rad_s = 0.01;
};

struct ImuGraphFactorConfig
{
  EcefImuConfig integration;
  StateFactorNoise noise{
    0.10, 0.02, 0.10, 0.01, 0.001};
};

struct LidarGraphFactorConfig
{
  double line_sigma_m = 0.05;
  double plane_sigma_m = 0.05;
  double huber_delta_sigma = 2.5;
  std::size_t maximum_line_factors_per_keyframe = 48;
  std::size_t maximum_plane_factors_per_keyframe = 96;
  bool estimate_map_alignment = false;
  double map_alignment_translation_prior_sigma_m = 0.05;
  double map_alignment_rotation_prior_sigma_rad = 0.005;
};

struct GnssGraphFactorConfig
{
  double code_huber_delta_sigma = 2.5;
  double carrier_huber_delta_sigma = 2.5;
};

struct ReceiverSolutionGraphFactorConfig
{
  double position_huber_delta_sigma = 2.5;
};

struct ReceiverPositionMeasurement
{
  Vec3 antenna_position_ecef_m;
  Eigen::Matrix3d covariance_ecef_m2 = Eigen::Matrix3d::Identity();
  Vec3 antenna_in_body_m;
};

struct FloatSmootherConfig
{
  double duration_s = 10.0;
  std::size_t maximum_states = 20;
  std::size_t maximum_iterations = 6;
  double initial_damping = 1.0e-6;
  double convergence_delta_norm = 1.0e-6;
};

struct FloatSmootherDiagnostics
{
  std::size_t states = 0;
  std::size_t ambiguities = 0;
  std::size_t factors = 0;
  std::size_t state_prior_factors = 0;
  std::size_t imu_factors = 0;
  std::size_t lidar_line_factors = 0;
  std::size_t lidar_plane_factors = 0;
  std::size_t gnss_code_factors = 0;
  std::size_t gnss_carrier_factors = 0;
  std::size_t receiver_position_factors = 0;
  std::uint64_t optimization_calls = 0;
  std::uint64_t optimization_rollbacks = 0;
  std::uint64_t marginalizations = 0;
  std::uint64_t gnss_outages = 0;
  std::uint64_t rejected_factors = 0;
  std::size_t last_iterations = 0;
  std::size_t last_residual_rows = 0;
  double last_cost = 0.0;
  double last_delta_norm = 0.0;
  double last_condition_estimate = 0.0;
  double window_span_s = 0.0;
  double map_alignment_translation_correction_m = 0.0;
  double map_alignment_rotation_correction_rad = 0.0;
  bool map_alignment_estimated = false;
  bool last_solve_succeeded = false;
};

struct CarrierResidualDiagnostics
{
  SignalGroup group;
  std::size_t factors = 0;
  std::size_t fix_eligible_ambiguities = 0;
  std::size_t minimum_arc_observations = 0;
  std::size_t maximum_arc_observations = 0;
  double raw_rms_m = 0.0;
  double raw_max_m = 0.0;
  double normalized_rms = 0.0;
  double normalized_max = 0.0;
  double sigma_mean_m = 0.0;
  double sigma_min_m = 0.0;
  double sigma_max_m = 0.0;
};

struct CodeResidualDiagnostics
{
  SignalGroup group;
  std::size_t factors = 0;
  double raw_rms_m = 0.0;
  double raw_max_m = 0.0;
  double normalized_rms = 0.0;
  double normalized_max = 0.0;
  double sigma_mean_m = 0.0;
  double sigma_min_m = 0.0;
  double sigma_max_m = 0.0;
};

enum class FixedBackSubstitutionRejection : std::uint8_t
{
  None,
  NotEvaluated,
  InvalidInput,
  MissingAmbiguity,
  CovarianceUnavailable,
  CorrectionLimit,
  CostIncrease,
};

const char * toString(FixedBackSubstitutionRejection reason) noexcept;

struct FixedBackSubstitutionConfig
{
  double maximum_position_correction_m = 0.50;
  double maximum_rotation_correction_rad = 0.10;
  double maximum_velocity_correction_m_s = 1.0;
  double maximum_cost_increase = 5.0;
};

struct FixedBackSubstitutionResult
{
  bool accepted = false;
  FixedBackSubstitutionRejection rejection =
    FixedBackSubstitutionRejection::NotEvaluated;
  StateId state_id = 0;
  EcefState latest_state;
  double maximum_position_correction_m = 0.0;
  double maximum_rotation_correction_rad = 0.0;
  double maximum_velocity_correction_m_s = 0.0;
  double cost_before = 0.0;
  double cost_after = 0.0;
};

class FloatFixedLagSmoother
{
public:
  FloatFixedLagSmoother(
    FloatSmootherConfig config = {},
    LidarGraphFactorConfig lidar_config = {},
    GnssGraphFactorConfig gnss_config = {},
    ReceiverSolutionGraphFactorConfig receiver_solution_config = {});

  bool addState(StateId id, const EcefState & initial_state);
  bool setLidarMapAlignment(
    const RigidPose & ecef_lidar_world,
    std::optional<RigidPose> prior_anchor = std::nullopt);
  bool addStatePrior(StateId id, const EcefState & mean, const StateFactorNoise & noise);
  bool addImuFactor(
    StateId from,
    StateId to,
    const std::vector<ImuSample> & samples,
    const ImuGraphFactorConfig & config = {});
  bool addLidarFactors(
    StateId state,
    const std::vector<PointToLineFactor> & line_factors,
    const std::vector<PointToPlaneFactor> & plane_factors,
    const RigidPose & body_lidar = {});
  bool addGnssFactors(
    StateId state,
    const std::vector<DoubleDifferenceMeasurement> & measurements);
  bool addReceiverPositionFactor(
    StateId state,
    const ReceiverPositionMeasurement & measurement);

  bool optimize();
  void recordGnssOutage() noexcept {++diagnostics_.gnss_outages;}

  const EcefState * state(StateId id) const noexcept;
  const RigidPose & lidarMapAlignment() const noexcept {return lidar_map_alignment_;}
  std::optional<double> ambiguity(const DdAmbiguityKey & key) const;
  std::vector<CodeResidualDiagnostics> codeResidualDiagnostics() const;
  std::vector<CarrierResidualDiagnostics> carrierResidualDiagnostics(
    std::size_t minimum_observation_epochs) const;
  std::optional<FloatAmbiguityEstimate> floatAmbiguityEstimate();
  FixedBackSubstitutionResult previewFixedAmbiguities(
    const std::vector<DdAmbiguityKey> & keys,
    const Eigen::VectorXd & fixed_values_m,
    const FixedBackSubstitutionConfig & config = {});
  std::size_t stateCount() const noexcept {return states_.size();}
  std::size_t ambiguityCount() const noexcept {return ambiguities_.size();}
  std::size_t factorCount() const noexcept;
  const FloatSmootherDiagnostics & diagnostics() const noexcept {return diagnostics_;}

private:
  struct StatePriorFactor
  {
    StateId state = 0;
    EcefState mean;
    StateFactorNoise noise;
  };

  struct ImuFactor
  {
    StateId from = 0;
    StateId to = 0;
    std::vector<ImuSample> samples;
    ImuGraphFactorConfig config;
  };

  struct LidarFactorBatch
  {
    StateId state = 0;
    std::vector<PointToLineFactor> lines;
    std::vector<PointToPlaneFactor> planes;
    RigidPose body_lidar;
  };

  struct GnssFactorBatch
  {
    StateId state = 0;
    std::vector<DoubleDifferenceMeasurement> measurements;
  };

  struct ReceiverPositionFactor
  {
    StateId state = 0;
    ReceiverPositionMeasurement measurement;
  };

  struct MarginalVariable
  {
    enum class Kind : std::uint8_t {State, MapAlignment, Ambiguity};
    Kind kind = Kind::State;
    StateId state = 0;
    DdAmbiguityKey ambiguity;
    int dimension = 0;

    bool operator==(const MarginalVariable & other) const noexcept
    {
      if (kind != other.kind) {
        return false;
      }
      if (kind == Kind::State) {
        return state == other.state;
      }
      return kind == Kind::MapAlignment || ambiguity == other.ambiguity;
    }
  };

  struct VariableLayout
  {
    std::map<StateId, int> state_offsets;
    std::optional<int> map_alignment_offset;
    std::map<DdAmbiguityKey, int> ambiguity_offsets;
    std::vector<MarginalVariable> variables;
    int dimension = 0;
  };

  struct LinearSystem
  {
    Eigen::MatrixXd hessian;
    Eigen::VectorXd gradient;
    double cost = 0.0;
    std::size_t rows = 0;
  };

  struct DenseMarginalPrior
  {
    std::vector<MarginalVariable> variables;
    std::map<StateId, EcefState> state_anchors;
    std::optional<RigidPose> map_alignment_anchor;
    std::map<DdAmbiguityKey, double> ambiguity_anchors;
    Eigen::MatrixXd hessian;
    Eigen::VectorXd gradient;
  };

  VariableLayout createLayout() const;
  LinearSystem buildLinearSystem(
    const VariableLayout & layout,
    std::optional<StateId> marginalize_state = std::nullopt);
  std::optional<Eigen::MatrixXd> linearizedCovariance(const LinearSystem & system) const;
  bool prepareFixLinearization();
  void invalidateFixLinearization() noexcept;
  std::optional<double> transformedAmbiguityInitialization(
    const DdAmbiguityKey & key) const;
  bool applyDelta(const VariableLayout & layout, const Eigen::VectorXd & delta);
  bool marginalizeOldestIfNeeded();
  bool marginalizeOldest();
  void refreshDiagnostics();

  FloatSmootherConfig config_;
  LidarGraphFactorConfig lidar_config_;
  GnssGraphFactorConfig gnss_config_;
  ReceiverSolutionGraphFactorConfig receiver_solution_config_;
  std::map<StateId, EcefState> states_;
  std::vector<StateId> state_order_;
  std::map<DdAmbiguityKey, double> ambiguities_;
  std::map<DdAmbiguityKey, StateId> ambiguity_last_state_;
  std::map<DdAmbiguityKey, std::size_t> ambiguity_observation_counts_;
  std::vector<StatePriorFactor> state_priors_;
  std::vector<ImuFactor> imu_factors_;
  std::vector<LidarFactorBatch> lidar_factors_;
  std::vector<GnssFactorBatch> gnss_factors_;
  std::vector<ReceiverPositionFactor> receiver_position_factors_;
  RigidPose lidar_map_alignment_;
  RigidPose lidar_map_alignment_anchor_;
  std::optional<DenseMarginalPrior> marginal_prior_;
  std::optional<VariableLayout> fix_layout_cache_;
  std::optional<LinearSystem> fix_system_cache_;
  std::optional<Eigen::MatrixXd> fix_covariance_cache_;
  FloatSmootherDiagnostics diagnostics_;
};

}  // namespace fgo_gil_localizer
