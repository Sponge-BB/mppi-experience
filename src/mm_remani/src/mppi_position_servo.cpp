// Numerical MPPI integration only; no ROS, hardware or external dependencies.
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace {
int validate_arm(
    double* states, std::size_t batch, const double* targets,
    const double* qmin, const double* qmax, const double* vmax,
    const double* amax, double dt) {
  if (!states || !qmin || !qmax || !vmax || !amax ||
      !std::isfinite(dt) || dt <= 0.) return 1;
  const double count_real = std::ceil(dt / .01);
  if (count_real >= static_cast<double>(std::numeric_limits<std::size_t>::max())) return 1;
  for (std::size_t j = 0; j < 6; ++j)
    if (!std::isfinite(qmin[j]) || !std::isfinite(qmax[j]) ||
        !std::isfinite(vmax[j]) || !std::isfinite(amax[j]) ||
        qmin[j] >= qmax[j] || vmax[j] <= 0. || amax[j] <= 0.) return 2;
  for (std::size_t i = 0; i < batch; ++i)
    for (std::size_t j = 0; j < 6; ++j)
      if (!std::isfinite(states[17*i+5+j]) || !std::isfinite(states[17*i+11+j]) ||
          (targets && !std::isfinite(targets[6*i+j]))) return 2;
  return 0;
}

void advance_arm(double* states, std::size_t batch, const double* targets,
    const double* qmin, const double* qmax, const double* vmax,
    const double* amax, double dt) {
  const std::size_t count = std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(dt/.01)));
  const double step = dt / count;
  for (std::size_t k = 0; k < count; ++k) {
    for (std::size_t i = 0; i < batch; ++i) {
      double* z = states + 17*i;
      for (std::size_t j = 0; j < 6; ++j) {
        const double q = z[5+j], velocity = z[11+j];
        double desired = targets ? (targets[6*i+j]-q)/.12 : 0.;
        const double lower = -std::min(vmax[j], std::sqrt(2*amax[j]*std::max(q-qmin[j], 0.)));
        const double upper = std::min(vmax[j], std::sqrt(2*amax[j]*std::max(qmax[j]-q, 0.)));
        desired = std::min(std::max(desired, lower), upper);
        const double acceleration = std::min(std::max((desired-velocity)/.12, -amax[j]), amax[j]);
        const double next_velocity = velocity + step*acceleration;
        z[5+j] = q + step*(velocity+next_velocity)/2;
        z[11+j] = next_velocity;
      }
    }
  }
}

int validate_base(const double* states, std::size_t batch,
                  const double* controls, const double* base) {
  if (!states || !base) return 1;
  for (std::size_t j = 0; j < 7; ++j)
    if (!std::isfinite(base[j]) || base[j] <= 0.) return 2;
  for (std::size_t i = 0; i < batch; ++i) {
    for (std::size_t j = 0; j < 5; ++j)
      if (!std::isfinite(states[17*i+j])) return 2;
    if (controls && (!std::isfinite(controls[8*i]) || !std::isfinite(controls[8*i+1]))) return 2;
  }
  return 0;
}

// Same coupled target/acceleration limits and midpoint update as _advance_base.
void advance_base(double* states, std::size_t batch, const double* controls,
                  const double* base, double dt) {
  const double half_track = base[0]/2;
  const double pi = 3.14159265358979323846264338327950288;
  for (std::size_t i = 0; i < batch; ++i) {
    double* z = states+17*i;
    double v = controls ? controls[8*i] : 0.;
    double w = controls ? controls[8*i+1] : 0.;
    double scale = std::max({1., std::abs(v)/base[1], std::abs(w)/base[2],
        std::abs(v-half_track*w)/base[3], std::abs(v+half_track*w)/base[3]});
    v /= scale; w /= scale;
    const double av = (v-z[3])/.12, aw = (w-z[4])/.12;
    scale = std::max({1., std::abs(av)/base[4], std::abs(aw)/base[5],
        std::abs(av-half_track*aw)/base[6], std::abs(av+half_track*aw)/base[6]});
    const double next_v = z[3]+dt*av/scale, next_w = z[4]+dt*aw/scale;
    const double mid_v = (z[3]+next_v)/2, mid_w = (z[4]+next_w)/2;
    const double angle = z[2]+dt*mid_w/2;
    z[0] += dt*mid_v*std::cos(angle);
    z[1] += dt*mid_v*std::sin(angle);
    double wrapped = std::fmod(z[2]+dt*mid_w+pi, 2*pi);
    if (wrapped < 0.) wrapped += 2*pi;
    z[2] = wrapped-pi; z[3] = next_v; z[4] = next_w;
  }
}
}  // namespace

extern "C" int mppi_position_servo(
    double* states, std::size_t batch, const double* targets,
    const double* qmin, const double* qmax, const double* vmax,
    const double* amax, double dt) {
  const int status = validate_arm(states, batch, targets, qmin, qmax, vmax, amax, dt);
  if (status) return status;
  advance_arm(states, batch, targets, qmin, qmax, vmax, amax, dt);
  return 0;
}

// base = track, vmax, wmax, wheel_v, accel, angular_accel, wheel_a.
// Null controls mean zero base target; null targets mean joint velocity braking.
extern "C" int mppi_whole_body_servo(
    double* states, std::size_t batch, const double* controls, const double* targets,
    const double* base, const double* qmin, const double* qmax,
    const double* vmax, const double* amax, double dt) {
  const int status = validate_arm(states, batch, targets, qmin, qmax, vmax, amax, dt);
  if (status) return status;
  const int base_status = validate_base(states, batch, controls, base);
  if (base_status) return base_status;
  advance_base(states, batch, controls, base, dt);
  advance_arm(states, batch, targets, qmin, qmax, vmax, amax, dt);
  return 0;
}

// Independent legacy velocity model. Measured dq is never clipped, and joint
// positions are never projected: acceptance still owns their original bounds.
extern "C" int mppi_whole_body_legacy_velocity(
    double* states, std::size_t batch, const double* controls,
    const double* base, const double* vmax, const double* amax, double dt) {
  if (!controls || !vmax || !amax || !std::isfinite(dt) || dt <= 0.) return 1;
  const int status = validate_base(states, batch, controls, base);
  if (status) return status;
  for (std::size_t j = 0; j < 6; ++j)
    if (!std::isfinite(vmax[j]) || !std::isfinite(amax[j]) ||
        vmax[j] <= 0. || amax[j] <= 0.) return 2;
  // Validate the complete batch before modifying even its first row.
  for (std::size_t i = 0; i < batch; ++i) {
    for (std::size_t j = 0; j < 6; ++j)
      if (!std::isfinite(states[17*i+5+j]) || !std::isfinite(states[17*i+11+j]) ||
          !std::isfinite(controls[8*i+2+j])) return 2;
  }
  advance_base(states, batch, controls, base, dt);
  for (std::size_t i = 0; i < batch; ++i) {
    double* z = states+17*i;
    for (std::size_t j = 0; j < 6; ++j) {
      const double velocity = z[11+j];
      const double target = std::min(std::max(controls[8*i+2+j], -vmax[j]), vmax[j]);
      const double next = velocity + std::min(std::max(target-velocity, -amax[j]*dt), amax[j]*dt);
      z[5+j] = z[5+j] + dt*(velocity+next)/2;
      z[11+j] = next;
    }
  }
  return 0;
}
