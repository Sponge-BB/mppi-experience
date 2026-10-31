// Pure Eigen/numerical test; no ROS graph, controller, or robot/model calls.
// The Python runner supplies frozen_native_lbfgs.hpp/domain_lbfgs.hpp in /tmp.
#include "frozen_native_lbfgs.hpp"
#include "domain_lbfgs.hpp"
#include <cassert>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

using Vector = Eigen::VectorXd;

struct Fixture {
  double target = .25, upper = .75;
  int calls = 0, rejected = 0, accepted = 0, last_ls = 0;
  double mutable_cache = 0.;
  bool generic_error = false, nonfinite_error = false;
};

static double objective(void* instance, const Vector& x, Vector& g) {
  auto& f = *static_cast<Fixture*>(instance);
  ++f.calls;
  f.mutable_cache = x[0];  // Intentionally represents native mutable caches.
  if (f.generic_error && f.calls > 1)
    throw std::runtime_error("unrelated native failure");
  if (f.nonfinite_error && f.calls > 1) {
    g.setConstant(std::numeric_limits<double>::quiet_NaN());
    return std::numeric_limits<double>::infinity();
  }
  if (x[0] < -1. || x[0] >= f.upper) {
    ++f.rejected;
    // A partially mutated/invalid gradient must never be consulted or accepted.
    g.setConstant(std::numeric_limits<double>::quiet_NaN());
    throw domain_step::RejectedTrial(x[0], 0., -1., -1., f.upper, 1., 2);
  }
  g[0] = 2. * (x[0] - f.target);
  return (x[0] - f.target) * (x[0] - f.target);
}

static int progress(void* instance, const Vector& x, const Vector& g, double,
                    double, int, int ls) {
  auto& f = *static_cast<Fixture*>(instance);
  assert(x.allFinite() && g.allFinite());
  assert(x[0] >= -1. && x[0] < f.upper);
  ++f.accepted;
  f.last_ls = ls;
  return 0;
}

static domain_lbfgs::lbfgs_parameter_t parameters() {
  domain_lbfgs::lbfgs_parameter_t p;
  p.g_epsilon = 1e-10;
  p.past = 0;
  p.max_iterations = 30;
  return p;
}

static void recovery() {
  domain_step::reset();
  Fixture f;
  Vector x = Vector::Zero(1);
  double cost = -1.;
  const int result = domain_lbfgs::lbfgs_optimize(
      x, cost, objective, nullptr, progress, &f, parameters());
  assert(result == domain_lbfgs::LBFGS_CONVERGENCE);
  assert(f.rejected == 1 && f.calls == 4 && f.accepted == 1 && f.last_ls == 3);
  assert(x[0] == .25 && cost == 0. && f.mutable_cache == x[0]);
  assert(domain_step::rejectedTrials() == 1);
  assert(domain_step::acceptedStepsAfterRejection() == 1);
}

static void initial_rejection_is_fatal() {
  domain_step::reset();
  Fixture f;
  Vector x = Vector::Ones(1);
  double cost = -1.;
  bool caught = false;
  try {
    domain_lbfgs::lbfgs_optimize(x, cost, objective, nullptr, progress, &f, parameters());
  } catch (const domain_step::RejectedTrial& error) {
    caught = true;
    assert(error.x == 1. && error.upper_x == .75 && error.body_index == 2);
  }
  assert(caught && f.calls == 1 && f.accepted == 0 && x[0] == 1. && cost == -1.);
  assert(domain_step::rejectedTrials() == 0);
  assert(domain_step::acceptedStepsAfterRejection() == 0);
}

static void exhausted_rejections_restore_numerics() {
  domain_step::reset();
  Fixture f;
  f.target = 1.;
  f.upper = 1e-12;
  auto p = parameters();
  p.max_linesearch = 4;
  Vector x = Vector::Zero(1);
  double cost = -1.;
  const int result = domain_lbfgs::lbfgs_optimize(
      x, cost, objective, nullptr, progress, &f, p);
  assert(result == domain_lbfgs::LBFGSERR_MAXIMUMLINESEARCH);
  assert(f.calls == 5 && f.rejected == 4 && f.accepted == 0);
  assert(x[0] == 0. && cost == 1.);
  assert(domain_step::rejectedTrials() == 4);
  assert(domain_step::acceptedStepsAfterRejection() == 0);
  // The integrating builder MUST return false here: numerical rollback cannot
  // undo arbitrary callback side effects. This is deliberately not deliverable.
  assert(f.mutable_cache != x[0]);

  // Direct line-search API additionally exposes gradient rollback for testing.
  f = Fixture{};
  domain_step::reset();
  f.target = 1.;
  f.upper = 1e-12;
  const Vector xp = Vector::Zero(1), gp = Vector::Constant(1, -2.);
  const Vector direction = Vector::Constant(1, 2.);
  Vector g = gp;
  x = xp;
  cost = 1.;
  double step = .5;
  domain_lbfgs::callback_data_t callbacks;
  callbacks.instance = &f;
  callbacks.proc_evaluate = objective;
  const int direct_result = domain_lbfgs::line_search_lewisoverton(
      x, cost, g, step, direction, xp, gp, p.min_step, p.max_step, callbacks, p);
  assert(direct_result == domain_lbfgs::LBFGSERR_MAXIMUMLINESEARCH);
  assert(f.calls == 4 && f.rejected == 4);
  assert(x[0] == xp[0] && g[0] == gp[0] && cost == 1.);
  assert(domain_step::rejectedTrials() == 4);
  assert(domain_step::acceptedStepsAfterRejection() == 0);
}

static void ordinary_failed_trial_restores_cost() {
  domain_step::reset();
  Fixture f;
  f.target = .1;
  f.upper = 2.;
  auto p = parameters();
  p.max_linesearch = 1;
  Vector x = Vector::Zero(1);
  double cost = -1.;
  const int result = domain_lbfgs::lbfgs_optimize(
      x, cost, objective, nullptr, progress, &f, p);
  assert(result == domain_lbfgs::LBFGSERR_MAXIMUMLINESEARCH);
  assert(f.calls == 2 && f.rejected == 0 && f.accepted == 0);
  assert(x[0] == 0. && cost == .1 * .1);
  assert(domain_step::rejectedTrials() == 0);
  assert(domain_step::acceptedStepsAfterRejection() == 0);
}

static void valid_but_unaccepted_trials_are_not_recovery() {
  domain_step::reset();
  Fixture f;
  f.target = .2;
  auto p = parameters();
  p.max_linesearch = 2;
  Vector x = Vector::Zero(1);
  double cost = -1.;
  const int result = domain_lbfgs::lbfgs_optimize(
      x, cost, objective, nullptr, progress, &f, p);
  // x=1 is a typed domain rejection; x=.5 is finite but fails Armijo.
  assert(result == domain_lbfgs::LBFGSERR_MAXIMUMLINESEARCH);
  assert(f.calls == 3 && f.rejected == 1 && f.accepted == 0);
  assert(x[0] == 0. && cost == .2 * .2);
  assert(domain_step::rejectedTrials() == 1);
  assert(domain_step::acceptedStepsAfterRejection() == 0);

  domain_step::reset();
  f = Fixture{};
  f.target = 1.;
  p = parameters();
  p.min_step = .4;
  x.setZero();
  const int minimum = domain_lbfgs::lbfgs_optimize(
      x, cost, objective, nullptr, progress, &f, p);
  assert(minimum == domain_lbfgs::LBFGSERR_MINIMUMSTEP);
  assert(f.calls == 2 && f.rejected == 1 && f.accepted == 0);
  assert(x[0] == 0. && cost == 1.);
  assert(domain_step::rejectedTrials() == 1);
  assert(domain_step::acceptedStepsAfterRejection() == 0);
}

static void unrelated_errors_remain_fatal() {
  domain_step::reset();
  Fixture f;
  f.generic_error = true;
  Vector x = Vector::Zero(1);
  double cost = -1.;
  bool caught = false;
  try {
    domain_lbfgs::lbfgs_optimize(x, cost, objective, nullptr, progress, &f, parameters());
  } catch (const std::runtime_error& error) {
    caught = std::string(error.what()) == "unrelated native failure";
  }
  assert(caught && f.calls == 2 && f.rejected == 0 && f.accepted == 0);
  f = Fixture{};
  f.nonfinite_error = true;
  x.setZero();
  const int result = domain_lbfgs::lbfgs_optimize(
      x, cost, objective, nullptr, progress, &f, parameters());
  assert(result == domain_lbfgs::LBFGSERR_INVALID_FUNCVAL);
  assert(f.calls == 2 && f.rejected == 0 && f.accepted == 0);
  assert(x[0] == 0. && cost == .25 * .25);
  assert(domain_step::rejectedTrials() == 0);
  assert(domain_step::acceptedStepsAfterRejection() == 0);
}

struct Trace {
  std::vector<Vector> evaluations;
  std::vector<double> accepted;
};

static double quadratic(void* instance, const Vector& x, Vector& g) {
  static_cast<Trace*>(instance)->evaluations.push_back(x);
  g[0] = x[0] - .3;
  g[1] = 3. * (x[1] + .4);
  return .5 * ((x[0] - .3) * (x[0] - .3) +
               3. * (x[1] + .4) * (x[1] + .4));
}

static int record(void* instance, const Vector& x, const Vector& g, double f,
                  double step, int k, int ls) {
  auto& trace = static_cast<Trace*>(instance)->accepted;
  trace.insert(trace.end(), {x[0], x[1], g[0], g[1], f, step, double(k), double(ls)});
  return 0;
}

static void ordinary_quadratic_is_identical_to_native() {
  domain_step::reset();
  Trace native_trace, patched_trace;
  Vector native_x = Vector::Constant(2, 2.), patched_x = native_x;
  double native_cost = -1., patched_cost = -2.;
  lbfgs::lbfgs_parameter_t native_parameters;
  domain_lbfgs::lbfgs_parameter_t patched_parameters;
  const int native_result = lbfgs::lbfgs_optimize(
      native_x, native_cost, quadratic, nullptr, record, &native_trace, native_parameters);
  const int patched_result = domain_lbfgs::lbfgs_optimize(
      patched_x, patched_cost, quadratic, nullptr, record, &patched_trace, patched_parameters);
  assert(native_result >= 0 && native_result == patched_result);
  assert(native_cost == patched_cost && (native_x.array() == patched_x.array()).all());
  assert(native_trace.accepted == patched_trace.accepted);
  assert(native_trace.evaluations.size() == patched_trace.evaluations.size());
  for (std::size_t i = 0; i < native_trace.evaluations.size(); ++i)
    assert((native_trace.evaluations[i].array() == patched_trace.evaluations[i].array()).all());
  assert(domain_step::rejectedTrials() == 0);
  assert(domain_step::acceptedStepsAfterRejection() == 0);
}

static void exception_requires_finite_domain_violation() {
  domain_step::reset();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const std::vector<std::vector<double>> invalid = {
      {nan, 0., -1., -1., 1., 1.}, {inf, 0., -1., -1., 1., 1.},
      {2., 0., nan, -1., 1., 1.}, {2., 0., -1., -1., inf, 1.},
      {2., 0., 1., -1., -1., 1.}, {2., 0., -1., 0., 1., 0.},
      {0., 0., -1., -1., 1., 1.}, {-1., -1., -1., -1., 1., 1.},
  };
  for (const auto& q : invalid) {
    bool caught = false;
    try {
      domain_step::RejectedTrial signal(q[0], q[1], q[2], q[3], q[4], q[5]);
    } catch (const std::invalid_argument&) {
      caught = true;
    }
    assert(caught);
  }
  // Upper grid node is excluded, lower node is included, exactly as native.
  domain_step::RejectedTrial edge(1., 0., -1., -1., 1., 1.);
  assert(edge.x == edge.upper_x);
  assert(domain_step::rejectedTrials() == 0);
}

int main() {
  exception_requires_finite_domain_violation();
  recovery();
  initial_rejection_is_fatal();
  exhausted_rejections_restore_numerics();
  ordinary_failed_trial_restores_cost();
  valid_but_unaccepted_trials_are_not_recovery();
  unrelated_errors_remain_fatal();
  ordinary_quadratic_is_identical_to_native();
  std::cout << "DOMAIN_STEP_CONTROL_PASSED recovery, fatal initial/generic/nonfinite, "
               "bounded exhaustion/rollback, native parity; no ROS\n";
}
