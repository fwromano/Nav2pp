#ifndef NAV2_CHAUFFEUR_CRITIC__CHAUFFEUR_CRITIC_HPP_
#define NAV2_CHAUFFEUR_CRITIC__CHAUFFEUR_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::ChauffeurCritic
 * @brief MPPI critic that prefers passenger-comfortable Ackermann trajectories.
 *
 * This is deliberately a soft objective. Safety, obstacle, path, and goal
 * critics remain responsible for feasibility and mission progress.
 */
class ChauffeurCritic : public CriticFunction
{
public:
  void initialize() override;
  void score(CriticData & data) override;

protected:
  unsigned int power_{1u};
  float weight_{1.0f};

  float longitudinal_jerk_weight_{1.0f};
  float lateral_accel_weight_{0.35f};
  float lateral_jerk_weight_{1.0f};
  float steering_effort_weight_{0.10f};
  float steering_rate_weight_{0.80f};
  float steering_accel_weight_{0.35f};

  float longitudinal_jerk_deadband_{0.75f};
  float lateral_accel_deadband_{1.50f};
  float lateral_jerk_deadband_{1.00f};
  float steering_effort_deadband_{0.75f};
  float steering_rate_deadband_{0.80f};
  float steering_accel_deadband_{2.00f};
  float min_steering_speed_{0.25f};
  float max_normalized_steering_{1.50f};

  bool apply_steering_terms_to_non_ackermann_{false};
};

}  // namespace mppi::critics

#endif  // NAV2_CHAUFFEUR_CRITIC__CHAUFFEUR_CRITIC_HPP_
