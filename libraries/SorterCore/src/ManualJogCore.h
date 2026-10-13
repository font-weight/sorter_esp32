#pragma once
#include "MotionCalibration.h"
#define SORTER_MOTION_API_VERSION 4

namespace sorter {

// Supported interpolation range, not the application's selected speed.
constexpr uint16_t kMinJogRateUsPerSecond = 50;
constexpr uint16_t kMaxJogRateUsPerSecond = 1000;

// Bounded pulse interpolation; camera coordination and homing are not inferred here.
struct JogSettings {
  bool outputsEnabled, pinoutConfirmed, limitsConfirmed;
  uint16_t minUs[4], maxUs[4];
  uint16_t maxStepUs, rateUsPerSecond;
  uint32_t holdLeaseMs, maxSessionMs, maxLoopGapMs;
};
inline JogSettings defaultJogSettings() {
  JogSettings s = {};
  for (uint8_t j = 0; j < 4; ++j) { s.minUs[j] = 1400; s.maxUs[j] = 1600; }
  s.maxStepUs = 25; s.rateUsPerSecond = 100;
  s.holdLeaseMs = 15000; s.maxSessionMs = 120000; s.maxLoopGapMs = 250;
  return s;
}
class JogIO {
 public:
  virtual ~JogIO() {}
  virtual bool enableAt(const JointPose&) = 0;
  virtual bool writePose(const JointPose&) = 0;
  virtual void detachOutputs() = 0;
};
enum class JogState : uint8_t { Disarmed, Holding, Jogging, Fault };
enum class JogError : uint8_t { None, Config, Seed, Support, Permit, Busy, Range,
  LeaseExpired, SessionExpired, LoopLate, Output, Estop };

class ManualJogController {
 public:
  ManualJogController(const JogSettings& settings, JogIO& io)
      : s_(settings), io_(io), state_(JogState::Disarmed), error_(JogError::None),
        seeded_(false), supported_(false), outputs_(false), current_{}, target_{},
        from_{}, poseMove_(false), moveDuration_(0), moveProgress_(0),
        started_(0), action_(0), loop_(0), write_(0) {}
  JogState state() const { return state_; }
  JogError error() const { return error_; }
  bool outputsOn() const { return outputs_; }
  bool seedPresent() const { return seeded_; }
  const JointPose& commandedPose() const { return current_; }
  uint32_t holdRemainingMs(uint32_t now) const {
    const uint32_t elapsed = uint32_t(now - action_);
    return outputs_ && elapsed < s_.holdLeaseMs ? s_.holdLeaseMs - elapsed : 0;
  }
  uint32_t sessionRemainingMs(uint32_t now) const {
    if (outputs_ && !s_.maxSessionMs) return UINT32_MAX; // Explicit unlimited session.
    const uint32_t elapsed = uint32_t(now - started_);
    return outputs_ && elapsed < s_.maxSessionMs ? s_.maxSessionMs - elapsed : 0;
  }
  bool configurationValid() const {
    if (!s_.outputsEnabled || !s_.pinoutConfirmed || !s_.limitsConfirmed ||
        s_.maxStepUs < 1 || s_.maxStepUs > 100 || s_.rateUsPerSecond < kMinJogRateUsPerSecond ||
        s_.rateUsPerSecond > kMaxJogRateUsPerSecond || s_.holdLeaseMs < 1000 || s_.holdLeaseMs > 300000 ||
        (s_.maxSessionMs && (s_.maxSessionMs < s_.holdLeaseMs || s_.maxSessionMs > 1800000)) ||
        s_.maxLoopGapMs < 20 || s_.maxLoopGapMs > 250) return false;
    for (uint8_t j = 0; j < 4; ++j)
      if (s_.minUs[j] < 500 || s_.maxUs[j] > 2500 || s_.minUs[j] >= s_.maxUs[j]) return false;
    return true;
  }
  bool seed(const JointPose& pose) {
    if (state_ != JogState::Disarmed) { error_ = JogError::Busy; return false; }
    supported_ = false; seeded_ = false;
    if (!within(pose)) { error_ = JogError::Seed; return false; }
    current_ = target_ = pose; seeded_ = true; error_ = JogError::None; return true;
  }
  bool confirmSupported() {
    if (state_ != JogState::Disarmed || !seeded_) return false;
    supported_ = true; return true;
  }
  bool arm(uint32_t now, bool permit, bool supply) {
    if (state_ != JogState::Disarmed) return false;
    const bool support = supported_; supported_ = false;
    if (!configurationValid()) { error_ = JogError::Config; return false; }
    if (!seeded_) { error_ = JogError::Seed; return false; }
    if (!support) { error_ = JogError::Support; return false; }
    if (!permit || !supply) { error_ = JogError::Permit; return false; }
    started_ = action_ = loop_ = write_ = now;
    outputs_ = true;  // Partial attachment must also be detached on error.
    if (!io_.enableAt(current_)) { stop(JogError::Output, true); return false; }
    target_ = current_; state_ = JogState::Holding; error_ = JogError::None; return true;
  }
  void disarm() { stop(JogError::None, false); }
  void estop() { stop(JogError::Estop, true); }
  void reset() { if (state_ == JogState::Fault) stop(JogError::None, false); }
  // Positive/negative steps affect one joint only. No command queue is retained.
  bool jog(uint8_t joint, int32_t delta, uint32_t now, bool permit, bool supply) {
    if (!ready(now, permit, supply)) return false;
    if (state_ != JogState::Holding) { error_ = JogError::Busy; return false; }
    if (joint > 3 || !delta || delta < -int32_t(s_.maxStepUs) || delta > s_.maxStepUs) {
      error_ = JogError::Range; return false;
    }
    const int32_t next = int32_t(current_.us[joint]) + delta;
    if (next < s_.minUs[joint] || next > s_.maxUs[joint]) { error_ = JogError::Range; return false; }
    target_ = current_; target_.us[joint] = uint16_t(next);
    poseMove_ = false;
    state_ = JogState::Jogging; action_ = now; error_ = JogError::None; return true;
  }
  // Absolute manual PWM target, not a Cartesian path or homing operation.
  // Validate all channels before accepting any part of the move. No queue.
  bool movePose(const JointPose& pose, uint32_t now, bool permit, bool supply) {
    if (!ready(now, permit, supply)) return false;
    if (state_ != JogState::Holding) { error_ = JogError::Busy; return false; }
    if (!configurationValid()) { stop(JogError::Config, true); return false; }
    if (!within(pose)) { error_ = JogError::Range; return false; }
    uint32_t distance = 0;
    for (uint8_t j = 0; j < 4; ++j) {
      const int32_t delta = int32_t(pose.us[j]) - current_.us[j];
      const uint32_t magnitude = uint32_t(delta < 0 ? -delta : delta);
      if (magnitude > distance) distance = magnitude;
    }
    action_ = now; error_ = JogError::None;
    if (!distance) return true; // Renew the lease without issuing a PWM write.
    from_ = current_; target_ = pose;
    moveDuration_ = (distance * 1000U + s_.rateUsPerSecond - 1U) / s_.rateUsPerSecond;
    moveProgress_ = 0; poseMove_ = true; write_ = now;
    state_ = JogState::Jogging; return true;
  }
  bool hold(uint32_t now, bool permit, bool supply) {
    if (!ready(now, permit, supply) || state_ != JogState::Holding) return false;
    action_ = now; error_ = JogError::None; return true;
  }
  void tick(uint32_t now, bool permit, bool supply) {
    if (!outputs_) return;
    if (!ready(now, permit, supply)) return;
    loop_ = now;
    if (state_ != JogState::Jogging || uint32_t(now-write_) < 20) return;
    // A late tick never catches up with a larger jump.
    if (poseMove_) {
      const uint32_t progress = moveDuration_ - moveProgress_ > 20U ?
        moveProgress_ + 20U : moveDuration_;
      JointPose next;
      for (uint8_t j = 0; j < 4; ++j) {
        const int32_t delta = int32_t(target_.us[j]) - from_.us[j];
        const uint32_t magnitude = uint32_t(delta < 0 ? -delta : delta);
        const uint32_t advance = uint32_t((uint64_t(magnitude)*progress +
                                          moveDuration_/2U) / moveDuration_);
        next.us[j] = uint16_t(int32_t(from_.us[j]) +
                             (delta < 0 ? -int32_t(advance) : int32_t(advance)));
      }
      if (!within(next) || !io_.writePose(next)) { stop(JogError::Output, true); return; }
      current_ = next; write_ = now; moveProgress_ = progress;
      if (progress == moveDuration_) {
        poseMove_ = false; state_ = JogState::Holding;
        if (error_ == JogError::Busy) error_ = JogError::None;
      }
      return;
    }
    const uint16_t step = uint16_t(s_.rateUsPerSecond * 20U / 1000U);
    JointPose next = current_;
    bool finished = true;
    for (uint8_t j = 0; j < 4; ++j) {
      const int32_t delta = int32_t(target_.us[j]) - current_.us[j];
      const int32_t increment = delta > step ? step : delta < -int32_t(step) ? -int32_t(step) : delta;
      next.us[j] = uint16_t(int32_t(current_.us[j]) + increment);
      if (next.us[j] != target_.us[j]) finished = false;
    }
    if (!within(next) || !io_.writePose(next)) { stop(JogError::Output, true); return; }
    current_ = next; write_ = now;
    if (finished) {
      state_ = JogState::Holding;
      if (error_ == JogError::Busy) error_ = JogError::None;
    }
  }
 private:
  const JogSettings& s_; JogIO& io_;
  JogState state_; JogError error_;
  bool seeded_, supported_, outputs_;
  JointPose current_, target_, from_;
  bool poseMove_;
  uint32_t moveDuration_, moveProgress_;
  uint32_t started_, action_, loop_, write_;
  bool within(const JointPose& pose) const {
    for (uint8_t j = 0; j < 4; ++j)
      if (pose.us[j] < s_.minUs[j] || pose.us[j] > s_.maxUs[j]) return false;
    return true;
  }
  void stop(JogError error, bool fault) {
    if (outputs_) io_.detachOutputs();
    outputs_ = seeded_ = supported_ = false; target_ = current_;
    poseMove_ = false; moveDuration_ = moveProgress_ = 0;
    error_ = error; state_ = fault ? JogState::Fault : JogState::Disarmed;
  }
  bool ready(uint32_t now, bool permit, bool supply) {
    if (!outputs_) return false;
    if (!permit || !supply) { stop(JogError::Permit, true); return false; }
    if (uint32_t(now-loop_) > s_.maxLoopGapMs) { stop(JogError::LoopLate, true); return false; }
    if (s_.maxSessionMs && uint32_t(now-started_) >= s_.maxSessionMs) { stop(JogError::SessionExpired, true); return false; }
    if (uint32_t(now-action_) >= s_.holdLeaseMs) { stop(JogError::LeaseExpired, false); return false; }
    return true;
  }
};
} // namespace sorter
