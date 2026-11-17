#pragma once
// Isolated SIL research objective. These are soft prediction-consistency
// proxies, not collision certificates, execution commitments or a replacement
// for the publication-time full-body safety checks.
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace coupling_objective {
using Vec2 = Eigen::Vector2d;

struct Sample {
  Vec2 pos = Vec2::Zero(), vel = Vec2::Zero(), acc = Vec2::Zero();
  // Samples are at a FIXED normalized coordinate u of native piece (traj,
  // piece). The caller freezes this sample set before optimization starts.
  int traj = 0, piece = 0, gear = 1;
  double u = 0., T = 1.;
};

struct Target {
  Vec2 pos = Vec2::Zero();
  double yaw = 0., speed = 0., time = 0., weight = 1.;
  // Body yaw and signed speed have the same convention as the MPPI checked
  // rollout. A stopped point must inherit its gear from that frozen rollout.
  int gear = 1;
};

struct FrozenTargets {
  // All times use ONE source-rollout-relative clock. Neither endpoint may be
  // enlarged to cover a requested delivery time. No extrapolation is done here.
  double checked_begin = 0., checked_end = 0.;
  std::vector<Target> execution;
  bool has_delivery = false;
  Target delivery;
};

struct Settings {
  bool execution_enabled = true, delivery_enabled = true;
  double execution_weight = 1., delivery_weight = 1.;
  double position_scale = .20, heading_weight = .20;
  double velocity_weight = .20, velocity_scale = .20;
  double temperature = .05, tube_radius = 1.;
  double velocity_epsilon = .03, heading_speed_threshold = .03;
};

struct Result {
  double cost = 0., execution_cost = 0., delivery_cost = 0.;
  std::vector<Vec2> grad_p, grad_v;
  std::size_t execution_targets = 0, delivery_targets = 0;
};

namespace detail {
inline void require(bool condition, const char* description) {
  if (!condition) throw std::invalid_argument(description);
}
inline bool finite(double x) { return std::isfinite(x); }
inline void checkSettings(const Settings& c) {
  for (double value : {c.execution_weight, c.delivery_weight, c.heading_weight,
       c.velocity_weight, c.heading_speed_threshold, c.tube_radius})
    require(finite(value) && value >= 0., "negative/non-finite coupling setting");
  for (double value : {c.position_scale, c.velocity_scale, c.temperature,
       c.velocity_epsilon})
    require(finite(value) && value > 0., "non-positive/non-finite coupling scale");
  for (double scale : {c.position_scale, c.velocity_scale, c.velocity_epsilon})
    require(finite(scale * scale) && scale * scale > 0.,
            "coupling scale square exceeds numerical range");
  require(finite(c.tube_radius * c.tube_radius),
          "coupling tube radius square exceeds numerical range");
}
inline void checkSamples(const std::vector<Sample>& samples) {
  for (const auto& s : samples) {
    require(s.pos.allFinite() && s.vel.allFinite() && s.acc.allFinite(),
            "non-finite native coupling sample");
    require(s.traj >= 0 && s.piece >= 0 && (s.gear == 1 || s.gear == -1),
            "invalid native sample metadata");
    require(finite(s.u) && s.u >= 0. && s.u <= 1. && finite(s.T) && s.T > 0.,
            "invalid native sample coordinate/duration");
  }
}
inline void checkTarget(const Target& t, const FrozenTargets& frozen) {
  require(t.pos.allFinite() && finite(t.yaw) && finite(t.speed) && finite(t.time),
          "non-finite frozen coupling target");
  require((t.gear == 1 || t.gear == -1) && finite(t.weight) && t.weight > 0.,
          "invalid frozen target gear/weight");
  require(t.speed * t.gear >= -1e-12, "frozen target signed speed/gear mismatch");
  require(t.time >= frozen.checked_begin && t.time <= frozen.checked_end,
          "coupling target exceeds checked rollout interval");
}

struct Match {
  double cost = 0.;
  std::vector<std::size_t> indices;
  std::vector<double> probabilities;
  std::vector<Vec2> grad_p, grad_v;
};

inline Match match(const std::vector<Sample>& samples, const Target& target,
                   const Settings& c, bool delivery) {
  Match out;
  std::vector<double> distances;
  const Vec2 heading(std::cos(target.yaw), std::sin(target.yaw));
  const Vec2 target_velocity = target.speed * heading;
  const double position_inv = 1. / (c.position_scale * c.position_scale);
  const double velocity_inv = 1. / (c.velocity_scale * c.velocity_scale);
  // This switch depends ONLY on the frozen input, never on optimized speed.
  const double heading_weight = std::abs(target.speed) > c.heading_speed_threshold
      ? c.heading_weight : 0.;
  double minimum = std::numeric_limits<double>::infinity();
  for (std::size_t j = 0; j < samples.size(); ++j) {
    const auto& s = samples[j];
    if (s.gear != target.gear) continue;
    const Vec2 delta = s.pos - target.pos;
    const double denom = std::hypot(std::hypot(s.vel.x(), s.vel.y()), c.velocity_epsilon);
    if (!finite(denom)) throw std::overflow_error("coupling velocity norm exceeds numerical range");
    const double dot = s.vel.dot(heading);
    const double orientation = 1. - s.gear * dot / denom;
    double distance = position_inv * delta.squaredNorm() + heading_weight * orientation;
    Vec2 gp = 2. * position_inv * delta;
    Vec2 gv = -heading_weight * s.gear *
        (heading / denom - (dot / denom) * (s.vel / denom) / denom);
    if (delivery && c.velocity_weight > 0.) {
      const Vec2 error = s.vel - target_velocity;
      distance += c.velocity_weight * velocity_inv * error.squaredNorm();
      gv += 2. * c.velocity_weight * velocity_inv * error;
    }
    // A finite input which is too large to represent is rejected, rather than
    // silently returning infinity or corrupting the native L-BFGS gradient.
    if (!finite(distance) || !gp.allFinite() || !gv.allFinite())
      throw std::overflow_error("coupling pair exceeds numerical range");
    require(distance >= -1e-12, "negative coupling pair distance");
    distance = std::max(0., distance);
    out.indices.push_back(j);
    out.grad_p.push_back(gp);
    out.grad_v.push_back(gv);
    distances.push_back(distance);
    minimum = std::min(minimum, distance);
  }
  require(!out.indices.empty(), "no same-gear native samples for coupling target");
  double partition = 0.;
  out.probabilities.reserve(distances.size());
  for (double distance : distances) {
    const double probability = std::exp(-(distance - minimum) / c.temperature);
    out.probabilities.push_back(probability);
    partition += probability;
  }
  // Normalization makes this soft-min nonnegative for nonnegative distances.
  // Shifting by the minimum prevents both underflow of all terms and overflow.
  out.cost = minimum - c.temperature *
      std::log(partition / static_cast<double>(distances.size()));
  if (!finite(out.cost)) throw std::overflow_error("non-finite coupling soft match");
  out.cost = std::max(0., out.cost);
  for (double& probability : out.probabilities) probability /= partition;
  return out;
}

inline void accumulate(Result& out, const Match& match, double multiplier) {
  for (std::size_t i = 0; i < match.indices.size(); ++i) {
    const std::size_t j = match.indices[i];
    const double weight = multiplier * match.probabilities[i];
    out.grad_p[j] += weight * match.grad_p[i];
    out.grad_v[j] += weight * match.grad_v[i];
  }
}
}  // namespace detail

inline Result evaluate(const std::vector<Sample>& samples,
                       const FrozenTargets& targets, const Settings& c = Settings()) {
  detail::checkSettings(c);
  detail::checkSamples(samples);
  Result out;
  out.grad_p.assign(samples.size(), Vec2::Zero());
  out.grad_v.assign(samples.size(), Vec2::Zero());
  const bool execution = c.execution_enabled && c.execution_weight > 0.;
  const bool delivery = c.delivery_enabled && c.delivery_weight > 0.;
  // Exact zero-weight identity, including no requirement for a prediction when
  // neither objective will consume it. No safety or planning state is changed.
  if (!execution && !delivery) return out;
  detail::require(!samples.empty(), "coupling objective has no native samples");
  detail::require(detail::finite(targets.checked_begin) && detail::finite(targets.checked_end) &&
      targets.checked_begin >= 0. && targets.checked_end >= targets.checked_begin,
      "invalid checked rollout interval");
  if (execution) {
    detail::require(!targets.execution.empty(), "execution coupling has no frozen targets");
    double total_weight = 0., previous = -1.;
    for (const auto& t : targets.execution) {
      detail::checkTarget(t, targets);
      detail::require(t.time > previous, "execution target times must increase strictly");
      previous = t.time;
      total_weight += t.weight;
    }
    detail::require(detail::finite(total_weight), "execution target weights overflow");
    for (const auto& t : targets.execution) {
      const auto m = detail::match(samples, t, c, false);
      const double error = std::max(0., m.cost - c.tube_radius * c.tube_radius);
      const double weight = c.execution_weight * t.weight / total_weight;
      out.execution_cost += weight * error * error;
      detail::accumulate(out, m, 2. * weight * error);
      ++out.execution_targets;
    }
  }
  if (delivery) {
    detail::require(targets.has_delivery, "delivery coupling has no frozen delivery target");
    detail::checkTarget(targets.delivery, targets);
    const auto m = detail::match(samples, targets.delivery, c, true);
    const double weight = c.delivery_weight * targets.delivery.weight;
    out.delivery_cost = weight * m.cost;
    detail::accumulate(out, m, weight);
    out.delivery_targets = 1;
  }
  out.cost = out.execution_cost + out.delivery_cost;
  if (!detail::finite(out.cost)) throw std::overflow_error("coupling objective exceeds numerical range");
  for (std::size_t i = 0; i < samples.size(); ++i)
    if (!out.grad_p[i].allFinite() || !out.grad_v[i].allFinite())
      throw std::overflow_error("coupling gradient exceeds numerical range");
  return out;
}
}  // namespace coupling_objective
