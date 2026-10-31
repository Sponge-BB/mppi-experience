// Standalone numerical/state tests. ros::Time is fake; no ROS graph, planner,
// model, controller, hardware, or per-odom file output is used.
#include "state_contract_probe.hpp"
#include <atomic>
#include <cassert>
#include <iostream>
#include <thread>
#include <vector>

namespace {
bool near(double a, double b, double tolerance = 1e-12) {
  return std::abs(a - b) <= tolerance;
}

nav_msgs::Odometry message(std::uint32_t token, double vx, double vy = 0.0) {
  nav_msgs::Odometry msg;
  msg.header.seq = token;
  msg.header.stamp = ros::Time(100, token);
  msg.header.frame_id = "world";
  msg.child_frame_id = "base_link";
  msg.pose.pose.position.x = token;
  msg.pose.pose.position.y = -static_cast<double>(token);
  msg.twist.twist.linear.x = vx;
  msg.twist.twist.linear.y = vy;
  return msg;
}

void testLegacyParity() {
  const double yaws[] = {0.0, .3, 1.5707963267948966, -2.8};
  const double thresholds[] = {-1.0, 0.0, .01, .1};
  const double velocities[] = {-.2, -.1, -.001, 0.0, .001, .1, .2};
  for (double yaw : yaws) for (double min_speed : thresholds)
    for (double vx : velocities) for (double vy : velocities) {
      Eigen::Vector2d expected(vx, vy);
      int gear;
      if (expected.norm() < min_speed) {
        expected(0) = min_speed * std::cos(yaw);
        expected(1) = min_speed * std::sin(yaw);
        gear = 0;
      } else {
        const Eigen::Vector2d heading(std::cos(yaw), std::sin(yaw));
        gear = heading.dot(expected) >= 0.0 ? 1 : -1;
      }
      // Exact equality, not approximate parity, for the original finite branch.
      const auto out = state_contract::signedRegularization(vx, vy, yaw,
                                                            min_speed, false, -1);
      assert(out.valid && out.vx == expected(0) && out.vy == expected(1));
      assert(out.gear == gear);
      assert(out.floor_bias == 0.0);
      assert(out.regularized == (Eigen::Vector2d(vx, vy).norm() < min_speed));
    }
}

void testSignedCases() {
  const double min_speed = .01;
  auto out = state_contract::signedRegularization(.001, 0., 0., min_speed, true);
  assert(out.valid && out.regularized && out.gear == 1);
  assert(out.vx > min_speed && out.vy == 0. && out.floor_bias > 0.);
  out = state_contract::signedRegularization(-.001, .0002, 0., min_speed, true);
  assert(out.valid && out.regularized && out.gear == -1);
  assert(out.vx < -min_speed && out.vy == 0. && out.floor_bias > 0.);

  // True zero, exact deadband endpoints and invalid history retain legacy gear0.
  for (double vx : {0., 1e-5, -1e-5, 1e-4, -1e-4}) {
    for (int prior : {0, 2, -2}) {
      out = state_contract::signedRegularization(vx, 0., 0., min_speed, true, prior);
      assert(out.gear == 0 && out.vx == min_speed && out.regularized);
    }
  }
  out = state_contract::signedRegularization(-1.00001e-4, 0., 0., min_speed, true);
  assert(out.gear == -1 && out.vx < -min_speed);
  out = state_contract::signedRegularization(1.00001e-4, 0., 0., min_speed, true);
  assert(out.gear == 1 && out.vx > min_speed);
  for (int prior : {-1, 1}) {
    out = state_contract::signedRegularization(0., 0., 0., min_speed, true, prior);
    assert(out.gear == prior && prior * out.vx > min_speed);
  }

  // No direction guess is made from small purely lateral motion.
  out = state_contract::signedRegularization(0., -.002, 0., min_speed, true);
  assert(out.gear == 0 && out.vx == min_speed && out.vy == 0.);
  const double yaw = .73;
  for (int sign : {-1, 1}) {
    out = state_contract::signedRegularization(sign * .001 * std::cos(yaw),
      sign * .001 * std::sin(yaw), yaw, min_speed, true);
    assert(out.gear == sign && out.regularized);
    assert(out.vx == sign * (min_speed + out.floor_bias) * std::cos(yaw));
    assert(out.vy == sign * (min_speed + out.floor_bias) * std::sin(yaw));
    assert(Eigen::Vector2d(out.vx, out.vy).norm() > min_speed);
  }

  // Native strict singularity branch must retain the signed direction at all
  // headings. This test mirrors the inequality only; root separately extracts
  // the frozen native function to verify the full downstream contract.
  const double pi = std::acos(-1.0);
  const double ulp = std::nextafter(min_speed,
    std::numeric_limits<double>::infinity()) - min_speed;
  for (int index = -720; index <= 720; ++index) {
    const double angle = index * pi / 720.;
    for (int sign : {-1, 1}) {
      out = state_contract::signedRegularization(sign * .001 * std::cos(angle),
        sign * .001 * std::sin(angle), angle, min_speed, true, 0);
      assert(out.valid && out.gear == sign && out.regularized);
      const double signed_speed = out.gear * Eigen::Vector2d(out.vx, out.vy).norm();
      const int native_gear = std::abs(signed_speed) > min_speed ?
        (signed_speed > 0. ? 1 : -1) : 0;
      assert(native_gear == sign);
      assert(out.floor_bias >= 0. && out.floor_bias <= 16. * ulp);
      assert(out.vx == sign * (min_speed + out.floor_bias) * std::cos(angle));
      assert(out.vy == sign * (min_speed + out.floor_bias) * std::sin(angle));
    }
  }

  // Exact threshold, just above it, reverse, lateral and turning states do not
  // have velocity magnitude/direction changed by signed mode.
  for (double vx : {-min_speed, min_speed, .010000001, -.010000001}) {
    out = state_contract::signedRegularization(vx, 0., 0., min_speed, true, -1);
    assert(!out.regularized && out.vx == vx && out.vy == 0.);
    assert(out.gear == (vx < 0. ? -1 : 1));
  }
  out = state_contract::signedRegularization(0., .02, 0., min_speed, true, -1);
  assert(!out.regularized && out.vx == 0. && out.vy == .02 && out.gear == 1);
  out = state_contract::signedRegularization(-.02, .03, 1.2, min_speed, true);
  assert(!out.regularized && out.vx == -.02 && out.vy == .03 && out.gear == 1);
}

void testInvalidNumbers() {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (bool signed_mode : {false, true}) {
    for (double bad : {nan, inf, -inf}) {
      const state_contract::Regularization cases[] = {
        state_contract::signedRegularization(bad, 0., 0., .01, signed_mode, 1),
        state_contract::signedRegularization(0., bad, 0., .01, signed_mode, 1),
        state_contract::signedRegularization(0., 0., bad, .01, signed_mode, 1),
        state_contract::signedRegularization(0., 0., 0., bad, signed_mode, 1)};
      for (const auto& out : cases) {
        assert(!out.valid && !out.regularized && out.gear == 0);
        assert(std::isnan(out.vx) && std::isnan(out.vy));
        assert(std::isnan(out.floor_bias));
      }
    }
  }
  // Extremely small floor norms can underflow in the native Eigen norm. The
  // bounded signed-floor search must fail explicitly, not claim a valid gear.
  const auto underflow = state_contract::signedRegularization(0., 0., 0.,
    std::numeric_limits<double>::denorm_min(), true, 1);
  assert(!underflow.valid && std::isnan(underflow.vx));
}

void testMessageDigest() {
  assert(state_contract::fnv1a64(nullptr, 0) == UINT64_C(14695981039346656037));
  const std::uint8_t a[] = {'a'};
  assert(state_contract::fnv1a64(a, 1) == UINT64_C(12638187200555641996));
  auto msg = message(7, -.001, .0002);
  const auto original = state_contract::messageFnv64(msg);
  assert(!original.empty() && original == state_contract::messageFnv64(msg));
  // Independently serialized using Python nav_msgs/Odometry.serialize(BytesIO)
  // and the standard modulo-2^64 FNV1a loop (714 bytes for this exact fixture).
  assert(original == "9582342113619235353");
  msg.pose.covariance[35] = 1.; // Also hash unrecorded pose/twist covariance.
  assert(original != state_contract::messageFnv64(msg));
  msg.pose.covariance[35] = 0.;
  msg.twist.twist.angular.z = .01;
  assert(original != state_contract::messageFnv64(msg));
  msg.twist.twist.angular.z = 0.;
  msg.header.frame_id += "-other";
  assert(original != state_contract::messageFnv64(msg));
}

void testAssociation() {
  state_contract::StateRecorder store;
  const auto absent = store.snapshot();
  assert(!absent.received && !absent.processed_ready && absent.seq == 0);
  assert(std::isnan(absent.world_vx) && std::isnan(absent.processed_vx));
  assert(!store.processed(0, .01, 0., 0));

  ros::Time::setNow(ros::Time(200, 123456789));
  const double yaw = .5;
  const double before = dual_probe::mono();
  const auto first_seq = store.receive(message(7, -.001, .0002), yaw);
  const auto raw = store.snapshot();
  assert(first_seq != 0 && raw.received && raw.raw_finite);
  assert(raw.header_seq == 7 && raw.header_stamp == ros::Time(100, 7));
  assert(raw.frame_id == "world" && raw.child_frame_id == "base_link");
  assert(raw.message_fnv64 == state_contract::messageFnv64(message(7, -.001, .0002)));
  assert(raw.receipt_ros == ros::Time(200, 123456789));
  assert(raw.receipt_mono >= before && raw.receipt_mono <= dual_probe::mono());
  assert(raw.x == 7. && raw.y == -7. && raw.yaw == yaw);
  assert(raw.body_vx == -.001 && raw.body_vy == .0002);
  assert(near(raw.world_vx, std::cos(yaw) * -.001 - std::sin(yaw) * .0002));
  assert(near(raw.world_vy, std::sin(yaw) * -.001 + std::cos(yaw) * .0002));
  assert(!raw.processed_ready);
  assert(store.processed(first_seq, -.01 * std::cos(yaw), -.01 * std::sin(yaw), -1));
  const auto ready = store.snapshot();
  assert(ready.seq == first_seq && ready.processed_seq == first_seq);
  assert(ready.processed_ready && ready.processed_finite && ready.processed_gear == -1);
  assert(ready.body_vx == raw.body_vx && ready.world_vx == raw.world_vx);
  assert(ready.receipt_ros == raw.receipt_ros && ready.header_stamp == raw.header_stamp);
  assert(ready.message_fnv64 == raw.message_fnv64);
  assert(!raw.processed_ready); // Earlier returned copies are immutable to callbacks.
  assert(!store.processed(first_seq, .01, 0., 1)); // Reject duplicate association.

  const auto second_seq = store.receive(message(8, 0.), 0.);
  assert(second_seq == first_seq + 1);
  assert(!store.processed(first_seq, 123., 456., -1)); // Delayed callback cannot win.
  auto second = store.snapshot();
  assert(second.seq == second_seq && !second.processed_ready);
  assert(second.processed_seq == 0 && std::isnan(second.processed_vx));
  assert(second.received_count == 2 && second.processed_count == 1);
  assert(second.rejected_processed_count == 3);
  assert(store.processed(second_seq, state_contract::unavailable(), 0., 0));
  second = store.snapshot();
  assert(second.processed_ready && !second.processed_finite);
  const auto third_seq = store.receive(message(9, state_contract::unavailable()), 0.);
  assert(third_seq == second_seq + 1);
  assert(!store.snapshot().raw_finite && std::isnan(store.snapshot().body_vx));
}

void assertCoherent(const state_contract::Snapshot& state) {
  if (!state.received) return;
  assert(state.x == state.header_seq && state.y == -state.x);
  assert(state.header_stamp == ros::Time(100, state.header_seq));
  assert(state.body_vx == state.x && state.world_vx == state.x);
  assert(state.world_vy == 0. && state.body_vy == 0. && state.yaw == 0.);
  if (state.processed_ready) {
    assert(state.processed_seq == state.seq && state.processed_finite);
    assert(state.processed_vx == state.world_vx + 1000.);
    assert(state.processed_vy == 0. && state.processed_gear == 1);
  } else {
    assert(state.processed_seq == 0 && std::isnan(state.processed_vx));
  }
}

void testConcurrentSnapshots() {
  state_contract::StateRecorder store;
  std::atomic<unsigned> finished{0};
  std::atomic<std::uint32_t> source_serial{0};
  std::vector<std::thread> writers;
  for (unsigned worker = 0; worker < 4; ++worker) {
    writers.emplace_back([&] {
      for (unsigned i = 0; i < 500; ++i) {
        const auto serial = ++source_serial;
        const auto seq = store.receive(message(serial, serial), 0.);
        if ((i % 3) == 0) std::this_thread::yield();
        store.processed(seq, serial + 1000., 0., 1);
      }
      ++finished;
    });
  }
  while (finished.load() != 4) assertCoherent(store.snapshot());
  for (auto& writer : writers) writer.join();
  const auto final = store.snapshot();
  assertCoherent(final);
  assert(final.seq == 2000 && final.received_count == 2000);
  assert(final.processed_count + final.rejected_processed_count == 2000);
}

void testBoundaryMetadata() {
  state_contract::StateRecorder store;
  Eigen::VectorXd p(3), v(3);
  p << 1., 2., 3.;
  v << .1, -.2, state_contract::unavailable();
  const auto missing = store.boundarySnapshot();
  const auto fields = state_contract::boundaryFields(missing, p, v, 0., 0, 201.);
  assert(fields.text.find("\"raw_odom_missing\":true") != std::string::npos);
  assert(fields.text.find("\"raw_odom_processed_missing\":true") != std::string::npos);
  assert(fields.text.find("\"raw_odom_world_vx\":null") != std::string::npos);
  assert(fields.text.find("\"planner_p\":[1,2,3]") != std::string::npos);
  assert(fields.text.find("null]") != std::string::npos);
  assert(fields.text.find("nan") == std::string::npos);
  assert(missing.boundary_count == 1);
  const auto seq = store.receive(message(12, -.001), 0.);
  assert(store.processed(seq, -.01, 0., -1));
  const auto present = store.boundarySnapshot();
  const auto ready_fields = state_contract::boundaryFields(present, p, v, .3, -1, 202.);
  assert(present.boundary_count == 2);
  assert(ready_fields.text.find("\"raw_odom_missing\":false") != std::string::npos);
  assert(ready_fields.text.find("\"raw_odom_processed_missing\":false") != std::string::npos);
  assert(ready_fields.text.find("\"raw_odom_header_seq\":12") != std::string::npos);
  assert(ready_fields.text.find("\"raw_odom_stamp_nsec\":12") != std::string::npos);
  assert(ready_fields.text.find("\"raw_odom_frame_id\":\"world\"") != std::string::npos);
  assert(ready_fields.text.find("\"raw_odom_processed_floor_bias\":0") != std::string::npos);
  assert(ready_fields.text.find("\"raw_odom_message_fnv64\":\"" +
    present.message_fnv64 + "\"") != std::string::npos);
  assert(ready_fields.text.find("fnv1a64_ros1_serialized_standard") != std::string::npos);

  // Global wrapper emits only when explicitly requested, and stays silent here
  // because DUAL_TIMELINE_PATH is unset. No file or ROS graph is created.
  unsetenv("DUAL_TIMELINE_PATH");
  const auto global_seq = state_contract::receive(message(13, 0.), 0.);
  assert(state_contract::processed(global_seq, .01, 0., 0));
  const auto emitted = state_contract::emitBoundary("state_contract_unit", p, v, 0., 0, 203.);
  assert(emitted.received && emitted.processed_ready && emitted.seq == global_seq);
  assert(emitted.boundary_count == 1);

  std::uint64_t counter = std::numeric_limits<std::uint64_t>::max() - 1;
  state_contract::incrementBounded(counter);
  state_contract::incrementBounded(counter);
  assert(counter == std::numeric_limits<std::uint64_t>::max());
}
} // namespace

int main() {
  ros::Time::init();
  ros::Time::setNow(ros::Time(200, 0));
  testLegacyParity();
  testSignedCases();
  testInvalidNumbers();
  testMessageDigest();
  testAssociation();
  testConcurrentSnapshots();
  testBoundaryMetadata();
  std::cout << "STATE_CONTRACT_PROBE_PASSED (no ROS graph)\n";
}
