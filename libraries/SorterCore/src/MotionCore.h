#pragma once
#include "MotionCalibration.h"
#include "Protocol.h"
#include <stdint.h>
#include <math.h>

namespace sorter {

enum class MotionState : uint8_t { Disarmed, ParkSettling, WaitScene, Moving, Paused, Done, Fault };
enum class MotionError : uint8_t {
  None, BadConfig, ParkNotConfirmed, EnableAbsent, PowerLost, OperatorDisarm,
  EmergencyStop, OutputFailure, CameraTimeout, CameraError, CameraRestart,
  CalibrationMismatch, MalformedScene, NoReachableObject, InvalidRoute,
  NoProgress, BatchLimit, LoopLate, PauseLimit, SequenceExhausted
};
inline const char* motionStateName(MotionState v) {
  switch (v) {
    case MotionState::Disarmed: return "DISARMED";
    case MotionState::ParkSettling: return "PARK_SETTLING";
    case MotionState::WaitScene: return "WAIT_SCENE";
    case MotionState::Moving: return "MOVING";
    case MotionState::Paused: return "PAUSED_HOLD";
    case MotionState::Done: return "DONE";
    default: return "FAULT";
  }
}
inline const char* motionErrorName(MotionError v) {
  switch (v) {
    case MotionError::None: return "NONE";
    case MotionError::BadConfig: return "BAD_CONFIG";
    case MotionError::ParkNotConfirmed: return "PARK_NOT_CONFIRMED";
    case MotionError::EnableAbsent: return "HARDWARE_ENABLE_ABSENT";
    case MotionError::PowerLost: return "SERVO_SUPPLY_LOST";
    case MotionError::OperatorDisarm: return "OPERATOR_DISARM";
    case MotionError::EmergencyStop: return "ESTOP";
    case MotionError::OutputFailure: return "OUTPUT_OR_UART_FAILURE";
    case MotionError::CameraTimeout: return "CAMERA_TIMEOUT";
    case MotionError::CameraError: return "CAMERA_REPORTED_ERROR";
    case MotionError::CameraRestart: return "CAMERA_RESTART";
    case MotionError::CalibrationMismatch: return "CALIBRATION_MISMATCH";
    case MotionError::MalformedScene: return "MALFORMED_SCENE";
    case MotionError::NoReachableObject: return "NO_REACHABLE_OBJECT";
    case MotionError::InvalidRoute: return "INVALID_ROUTE";
    case MotionError::NoProgress: return "TARGET_STILL_PRESENT";
    case MotionError::BatchLimit: return "BATCH_LIMIT";
    case MotionError::LoopLate: return "CONTROL_LOOP_LATE";
    case MotionError::PauseLimit: return "PAUSE_HOLD_LIMIT";
    default: return "SEQUENCE_EXHAUSTED";
  }
}

struct MotionSettings {
  bool virtualMode, outputsEnabled, pinoutConfirmed;
  uint32_t sceneTimeoutMs, parkSettleMs, minMoveMs, dwellMs, gripDwellMs;
  uint32_t maxBatchMs, maxPauseMs, maxLoopGapMs;
  uint16_t targetMatchRadius10;
  uint8_t maxAttempts, maxNoProgress;
};
inline MotionSettings defaultMotionSettings() {
  MotionSettings s = {};
  s.virtualMode = true;
  s.sceneTimeoutMs = 6000; s.parkSettleMs = 800; s.minMoveMs = 400;
  s.dwellMs = 250; s.gripDwellMs = 500;
  s.maxBatchMs = 300000; s.maxPauseMs = 30000; s.maxLoopGapMs = 250;
  s.targetMatchRadius10 = 120; s.maxAttempts = 12; s.maxNoProgress = 2;
  return s;
}

// Implementations must return false on a detectable output/transport failure.
// SG90 PWM cannot confirm actual joint position, grip or motor power.
class MotionIO {
 public:
  virtual ~MotionIO() {}
  virtual bool enableAt(const JointPose&) = 0;
  virtual bool writePose(const JointPose&) = 0;
  virtual void detachOutputs() = 0;
  virtual bool requestScene(uint32_t session, uint32_t seq, uint32_t calibrationId) = 0;
  // Recheck latched physical inputs when an output fails between controller ticks.
  virtual bool hardwareEnablePresent() const { return true; }
  virtual bool servoSupplyPresent() const { return true; }
};

struct RouteStep { JointPose pose; ToolPoint tip; uint32_t dwellMs; const char* name; };

class MotionController {
 public:
  MotionController(const MotionCalibration& calibration, const MotionSettings& settings, MotionIO& io)
      : c_(calibration), s_(settings), io_(io), state_(MotionState::Disarmed),
        error_(MotionError::None), session_(0), seq_(0), cameraBoot_(0),
        lastNow_(0), lastTick_(0), startedAt_(0), stageAt_(0), pauseAt_(0),
        requestAt_(0), moveAt_(0), moveDuration_(0), lastWrite_(0),
        staleReplies_(0), retries_(0), routeSize_(0), routeIndex_(0), attempts_(0),
        clearedTargets_(0), noProgress_(0), outputsOn_(false), parkConfirmed_(false),
        awaiting_(false), moving_(false), verifyTarget_(false),
        pauseState_(MotionState::Disarmed), current_(calibration.parkOpen.joints),
        from_(calibration.parkOpen.joints), target_{} {}

  MotionState state() const { return state_; }
  MotionError error() const { return error_; }
  uint32_t session() const { return session_; }
  uint32_t sequence() const { return seq_; }
  uint32_t staleReplies() const { return staleReplies_; }
  uint8_t attempts() const { return attempts_; }
  uint8_t clearedTargets() const { return clearedTargets_; }
  bool outputsOn() const { return outputsOn_; }
  const JointPose& commandedPose() const { return current_; }
  const char* phaseName() const {
    return (state_ == MotionState::Moving || (state_ == MotionState::Paused &&
            pauseState_ == MotionState::Moving)) && routeIndex_ < routeSize_
            ? route_[routeIndex_].name : motionStateName(state_);
  }
  bool configurationValid() const {
    return calibrationValid(c_, !s_.virtualMode) &&
      (s_.virtualMode || (s_.outputsEnabled && s_.pinoutConfirmed)) &&
      s_.sceneTimeoutMs >= 100 && s_.sceneTimeoutMs <= 30000 &&
      s_.parkSettleMs >= 20 && s_.parkSettleMs <= 10000 &&
      s_.minMoveMs >= 20 && s_.minMoveMs <= 10000 &&
      s_.dwellMs <= 5000 && s_.gripDwellMs <= 5000 &&
      s_.maxBatchMs >= 1000 && s_.maxBatchMs <= 1800000 &&
      s_.maxPauseMs >= 100 && s_.maxPauseMs <= 60000 &&
      s_.maxLoopGapMs >= 20 && s_.maxLoopGapMs <= 500 &&
      s_.targetMatchRadius10 > 0 && s_.targetMatchRadius10 <= 1000 &&
      s_.maxAttempts > 0 && s_.maxAttempts <= 50 && s_.maxNoProgress > 0 &&
      s_.maxNoProgress <= 5;
  }

  // A human must support/position the de-energized arm at the measured park pose.
  // The acknowledgment is consumed by the very next ARM attempt.
  void confirmOperatorPark() {
    if (state_ == MotionState::Disarmed || state_ == MotionState::Done)
      parkConfirmed_ = true;
  }
  bool arm(uint32_t now, uint32_t session, bool hardwareEnable, bool supplyGood) {
    if (state_ != MotionState::Disarmed && state_ != MotionState::Done) return false;
    const bool confirmed = parkConfirmed_;
    parkConfirmed_ = false;
    if (!configurationValid() || !session) { error_ = MotionError::BadConfig; return false; }
    if (!confirmed) { error_ = MotionError::ParkNotConfirmed; return false; }
    if (!hardwareEnable || !supplyGood) {
      error_ = supplyGood ? MotionError::EnableAbsent : MotionError::PowerLost;
      return false;
    }
    session_ = session; seq_ = 0; cameraBoot_ = 0;
    attempts_ = clearedTargets_ = noProgress_ = retries_ = 0;
    verifyTarget_ = awaiting_ = moving_ = false;
    lastNow_ = lastTick_ = startedAt_ = stageAt_ = now;
    current_ = c_.parkOpen.joints;
    error_ = MotionError::None;
    outputsOn_ = true; // Detach is required even when enable only partly succeeds.
    if (!io_.enableAt(current_)) { outputFailure(); return false; }
    state_ = MotionState::ParkSettling;
    return true;
  }
  void disarm(MotionError why = MotionError::OperatorDisarm) {
    io_.detachOutputs(); outputsOn_ = false; awaiting_ = false; moving_ = false;
    parkConfirmed_ = false; state_ = MotionState::Disarmed; error_ = why;
    session_ = 0;
  }
  void emergencyStop() { fault(MotionError::EmergencyStop); }
  void resetFault() {
    if (state_ == MotionState::Fault) disarm(MotionError::None);
  }
  void pause(uint32_t now) {
    if (state_ != MotionState::Moving && state_ != MotionState::WaitScene &&
        state_ != MotionState::ParkSettling) return;
    pauseState_ = state_; pauseAt_ = now; state_ = MotionState::Paused;
    awaiting_ = false; // A response captured during pause may no longer describe the scene.
  }
  void resume(uint32_t now) {
    if (state_ != MotionState::Paused) return;
    if (uint32_t(now - pauseAt_) >= s_.maxPauseMs) { fault(MotionError::PauseLimit); return; }
    if (uint32_t(now - startedAt_) >= s_.maxBatchMs) { fault(MotionError::BatchLimit); return; }
    state_ = pauseState_; lastTick_ = lastNow_ = now;
    if (state_ == MotionState::Moving) {
      if (moving_) startMove(now); // Restart smooth profile from the actually commanded hold pose.
      else stageAt_ += uint32_t(now - pauseAt_);
    } else if (state_ == MotionState::WaitScene) {
      beginRequest(now); // New sequence, never reuse a potentially stale capture.
    } else stageAt_ += uint32_t(now - pauseAt_);
  }

  void tick(uint32_t now, bool hardwareEnable, bool supplyGood) {
    lastNow_ = now;
    if (!outputsOn_) { lastTick_ = now; return; }
    if (!hardwareEnable || !supplyGood) {
      disarm(supplyGood ? MotionError::EnableAbsent : MotionError::PowerLost); return;
    }
    if (uint32_t(now - lastTick_) > s_.maxLoopGapMs) { fault(MotionError::LoopLate); return; }
    lastTick_ = now;
    if (uint32_t(now - startedAt_) >= s_.maxBatchMs) { fault(MotionError::BatchLimit); return; }
    if (state_ == MotionState::Paused) {
      if (uint32_t(now - pauseAt_) >= s_.maxPauseMs) fault(MotionError::PauseLimit);
      return;
    }
    if (state_ == MotionState::ParkSettling) {
      if (uint32_t(now - stageAt_) >= s_.parkSettleMs) beginRequest(now);
    } else if (state_ == MotionState::WaitScene) {
      if (uint32_t(now - requestAt_) >= s_.sceneTimeoutMs) {
        if (retries_ >= 2) { fault(MotionError::CameraTimeout); return; }
        ++retries_; requestAt_ = now;
        if (!io_.requestScene(session_, seq_, c_.calibrationId)) outputFailure();
      }
    } else if (state_ == MotionState::Moving) {
      runMove(now);
    }
  }

  // Parser/CRC and decodeScene run outside this class. Only a requested result is accepted.
  bool receiveScene(uint32_t session, uint32_t seq, const Scene& scene, uint32_t now) {
    if (state_ != MotionState::WaitScene || !awaiting_ || session != session_ || seq != seq_) {
      ++staleReplies_; return false;
    }
    // A caller may feed a queued reply before running the next timer tick.
    // Expired replies never start a move; tick owns the same-sequence retry policy.
    if (uint32_t(now - requestAt_) >= s_.sceneTimeoutMs) { ++staleReplies_; return false; }
    if (uint32_t(now - startedAt_) >= s_.maxBatchMs) { fault(MotionError::BatchLimit); return false; }
    awaiting_ = false;
    if (scene.calibrationId != c_.calibrationId) { fault(MotionError::CalibrationMismatch); return false; }
    if (!scene.cameraBoot || scene.count > kMaxObjects) { fault(MotionError::MalformedScene); return false; }
    if (cameraBoot_ && cameraBoot_ != scene.cameraBoot) { fault(MotionError::CameraRestart); return false; }
    cameraBoot_ = scene.cameraBoot;
    for (uint8_t i = 0; i < scene.count; ++i)
      if (scene.objects[i].classId < 1 || scene.objects[i].classId > 3 || !scene.objects[i].pixels) {
        fault(MotionError::MalformedScene); return false;
      }
    if (verifyTarget_) {
      bool stillPresent = false;
      for (uint8_t i = 0; i < scene.count; ++i) {
        const Detection& d = scene.objects[i];
        const int64_t dx = int64_t(d.x10) - target_.x10, dy = int64_t(d.y10) - target_.y10;
        // Compare components first to avoid squaring untrusted int32 differences.
        const int64_t r = s_.targetMatchRadius10;
        if (d.classId == target_.classId && dx >= -r && dx <= r && dy >= -r && dy <= r &&
            dx*dx + dy*dy <= r*r) stillPresent = true;
      }
      if (stillPresent) ++noProgress_; else { noProgress_ = 0; ++clearedTargets_; }
      verifyTarget_ = false;
      if (noProgress_ >= s_.maxNoProgress) { fault(MotionError::NoProgress); return false; }
    }
    if (!scene.count) {
      io_.detachOutputs(); outputsOn_ = false; parkConfirmed_ = false; state_ = MotionState::Done;
      return true;
    }
    if (attempts_ >= s_.maxAttempts) { fault(MotionError::BatchLimit); return false; }
    bool found = false;
    for (uint8_t i = 0; i < scene.count; ++i)
      if (inPickArea(scene.objects[i].x10, scene.objects[i].y10, c_)) {
        target_ = scene.objects[i]; found = true; break;
      }
    if (!found) { fault(MotionError::NoReachableObject); return false; }
    if (!buildRoute()) { fault(MotionError::InvalidRoute); return false; }
    ++attempts_; routeIndex_ = 0; state_ = MotionState::Moving; startMove(now);
    return true;
  }
  bool receiveError(uint32_t session, uint32_t seq) {
    if (state_ != MotionState::WaitScene || !awaiting_ || session != session_ || seq != seq_) {
      ++staleReplies_; return false;
    }
    fault(MotionError::CameraError); return true;
  }

 private:
  const MotionCalibration& c_;
  const MotionSettings& s_;
  MotionIO& io_;
  MotionState state_;
  MotionError error_;
  uint32_t session_, seq_, cameraBoot_, lastNow_, lastTick_, startedAt_, stageAt_, pauseAt_;
  uint32_t requestAt_, moveAt_, moveDuration_, lastWrite_, staleReplies_;
  uint8_t retries_, routeSize_, routeIndex_, attempts_, clearedTargets_, noProgress_;
  bool outputsOn_, parkConfirmed_, awaiting_, moving_, verifyTarget_;
  MotionState pauseState_;
  JointPose current_, from_;
  Detection target_;
  RouteStep route_[12];

  void fault(MotionError why) {
    io_.detachOutputs(); outputsOn_ = false; awaiting_ = false; moving_ = false;
    parkConfirmed_ = false; state_ = MotionState::Fault; error_ = why;
  }
  void outputFailure() {
    if (!io_.servoSupplyPresent()) disarm(MotionError::PowerLost);
    else if (!io_.hardwareEnablePresent()) disarm(MotionError::EnableAbsent);
    else fault(MotionError::OutputFailure);
  }
  void beginRequest(uint32_t now) {
    if (seq_ == UINT32_MAX) { fault(MotionError::SequenceExhausted); return; }
    ++seq_; retries_ = 0; requestAt_ = now; awaiting_ = true; state_ = MotionState::WaitScene;
    if (!io_.requestScene(session_, seq_, c_.calibrationId)) outputFailure();
  }
  bool addStep(const CalibratedPose& p, bool closed, uint32_t dwell, const char* label) {
    if (routeSize_ >= 12 || !pointWithinBounds(p.tip, c_)) return false;
    RouteStep& r = route_[routeSize_];
    r.pose = p.joints; r.pose.us[3] = closed ? c_.closedUs : c_.openUs;
    r.tip = p.tip; r.dwellMs = dwell; r.name = label;
    if (!poseWithinLimits(r.pose, c_)) return false;
    ++routeSize_; return true;
  }
  bool buildRoute() {
    CalibratedPose low, high;
    routeSize_ = 0;
    if (!interpolateGrid(c_, target_.x10, target_.y10, false, low) ||
        !interpolateGrid(c_, target_.x10, target_.y10, true, high)) return false;
    const uint8_t bin = uint8_t(target_.classId - 1);
    return addStep(c_.hubOpen, false, s_.dwellMs, "HUB_OUT") &&
      addStep(high, false, s_.dwellMs, "APPROACH") &&
      addStep(low, false, s_.dwellMs, "DESCEND") &&
      addStep(low, true, s_.gripDwellMs, "GRIP") &&
      addStep(high, true, s_.dwellMs, "LIFT") &&
      addStep(c_.hubOpen, true, s_.dwellMs, "HUB_LOADED") &&
      addStep(c_.binHighOpen[bin], true, s_.dwellMs, "BIN_APPROACH") &&
      addStep(c_.binDropOpen[bin], true, s_.dwellMs, "BIN_LOWER") &&
      addStep(c_.binDropOpen[bin], false, s_.gripDwellMs, "RELEASE") &&
      addStep(c_.binHighOpen[bin], false, s_.dwellMs, "BIN_LEAVE") &&
      addStep(c_.hubOpen, false, s_.dwellMs, "HUB_RETURN") &&
      addStep(c_.parkOpen, false, s_.dwellMs, "PARK");
  }
  void startMove(uint32_t now) {
    from_ = current_; moveAt_ = lastWrite_ = now; moveDuration_ = s_.minMoveMs;
    for (uint8_t j = 0; j < 4; ++j) {
      const int32_t delta = int32_t(route_[routeIndex_].pose.us[j]) - from_.us[j];
      const uint32_t distance = uint32_t(delta < 0 ? -delta : delta);
      // Cubic smoothstep maximum speed is 1.5 delta/T, acceleration 6 delta/T^2.
      uint32_t byRate = uint32_t(ceil(1500.0 * distance / c_.maxRateUsPerSecond[j]));
      uint32_t byAccel = uint32_t(ceil(1000.0 * sqrt(6.0 * distance / c_.maxAccelerationUsPerSecond2[j])));
      if (byRate > moveDuration_) moveDuration_ = byRate;
      if (byAccel > moveDuration_) moveDuration_ = byAccel;
    }
    moving_ = true;
  }
  void runMove(uint32_t now) {
    if (moving_) {
      const uint32_t elapsed = uint32_t(now - moveAt_);
      if (uint32_t(now - lastWrite_) < 20 && elapsed < moveDuration_) return;
      const double t = elapsed >= moveDuration_ ? 1.0 : double(elapsed) / moveDuration_;
      const double f = t*t*(3.0-2.0*t);
      JointPose next;
      for (uint8_t j = 0; j < 4; ++j)
        next.us[j] = uint16_t(from_.us[j] + f * (int32_t(route_[routeIndex_].pose.us[j]) - from_.us[j]) + 0.5);
      if (!poseWithinLimits(next, c_)) { fault(MotionError::InvalidRoute); return; }
      if (!io_.writePose(next)) { outputFailure(); return; }
      current_ = next; lastWrite_ = now;
      if (elapsed >= moveDuration_) { moving_ = false; stageAt_ = now; }
    } else if (uint32_t(now - stageAt_) >= route_[routeIndex_].dwellMs) {
      if (++routeIndex_ < routeSize_) startMove(now);
      else {
        verifyTarget_ = true; state_ = MotionState::ParkSettling; stageAt_ = now;
      }
    }
  }
};
} // namespace sorter
