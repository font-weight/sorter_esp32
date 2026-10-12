#pragma once
#include <Arduino.h>
#include <esp_system.h>
#include <Protocol.h>
#include <ManualJogCore.h>
#include "LocalConfig.h"
#include "CameraAimConfig.h"
#include "CameraSortPlan.h"

// One commissioning transfer using the owner's successful manual red route.
// Camera frames are captured in PARK. TRIALRED keeps its fixed pick route;
// AIMRED only approaches the interpolated upper pose, with no descent or grip.
// SORT enters from smaller Y; CHECKPICK pauses at SIDE_HIGH/SIDE_LOW/TARGET_LOW.
namespace red_trial {
constexpr uint32_t REPLY_TIMEOUT_MS = 6000;
constexpr uint32_t TRIAL_LIMIT_MS = 180000;
constexpr int32_t PICK_X10 = 200, PICK_Y10 = 800;
// Inclusive camera-coordinate tolerance: +/-15 mm on each axis, in tenths.
// This widens the input filter; the tested pick route remains at 20,80 mm.
constexpr int32_t INPUT_TOLERANCE10 = 150;
const sorter::JointPose START_PARK = {{1500,1200,1500,2100}};
struct Step { sorter::JointPose pose; const char* name; uint32_t dwellMs; };
const Step STEPS[] = {
  {{{1500,1500,1500,2100}}, "HUB_OUT", 250},
  {{{1600,1700,1350,2100}}, "APPROACH_HIGH", 250},
  {{{1600,2060,950,2100}}, "NEAR", 250},
  {{{1600,2150,850,2100}}, "DESCEND", 250},
  {{{1600,2150,850,1250}}, "GRIP", 1000},
  {{{1600,2060,950,1250}}, "LIFT_NEAR", 250},
  {{{1600,1700,1350,1250}}, "LIFT", 250},
  {{{1500,1500,1500,1250}}, "HUB_LOADED", 250},
  {{{2100,1600,850,1250}}, "BIN_APPROACH", 250},
  {{{2100,1800,750,1250}}, "BIN_LOWER", 1000},
  {{{2100,1800,750,2100}}, "RELEASE", 1000},
  {{{2100,1600,850,2100}}, "BIN_LEAVE", 250},
  {{{1500,1500,1500,2100}}, "HUB_RETURN", 250},
  {{{1500,1200,1500,2100}}, "PARK_RETURN", 800}
};
constexpr size_t STEP_COUNT = sizeof(STEPS) / sizeof(STEPS[0]);

class RedPickTrial {
 public:
  RedPickTrial(sorter::ManualJogController& motion, const sorter::JogSettings& settings,
               HardwareSerial& camera)
      : motion_(motion), settings_(settings), camera_(camera), state_(Idle),
        session_(0), sequence_(0), boot_(0), started_(0), requested_(0), dwellAt_(0),
        step_(0), error_("NONE"), aimOnly_(false), aimSteps_{}, batch_(false),
        sorted_{}, sortAttempts_(0), sortPlan_{}, sortSteps_{},
        checkOnly_(false), checkReturning_(false), checkSteps_{} {}

  bool busy() const { return state_ == Before || state_ == Moving || state_ == Dwell || state_ == Verify || state_ == Paused; }
  const char* stateName() const {
    switch (state_) {
      case Idle: return "IDLE";
      case Before: return "WAIT_CAMERA";
      case Moving: return "MOVING";
      case Dwell: return "DWELL";
      case Verify: return "VERIFY_CAMERA";
      case Paused: return "PAUSED";
      case Done: return "DONE";
      default: return "FAILED";
    }
  }
  const char* error() const { return error_; }
  const char* actionName() const { return checkOnly_ ? "CHECKPICK" : (batch_ ? "SORT" : (aimOnly_ ? "AIMRED" : "TRIALRED")); }
  uint8_t sortedRed() const { return sorted_[0]; }
  uint8_t sortedBlue() const { return sorted_[1]; }
  uint8_t sortAttempts() const { return sortAttempts_; }
  void start(uint32_t now, bool uartReady, bool permit, bool aimOnly = false, bool batch = false, bool checkOnly = false) {
    if (busy()) { Serial.println("TRIAL_REFUSED reason=BUSY"); return; }
    if (!uartReady) { Serial.println("TRIAL_REFUSED reason=CAMERA_UART_DISABLED"); return; }
    if (!motion_.outputsOn() || motion_.state() != sorter::JogState::Holding) {
      Serial.println("TRIAL_REFUSED reason=ARM_IN_HOLDING_REQUIRED"); return;
    }
    for (uint8_t j = 0; j < 4; ++j)
      if (motion_.commandedPose().us[j] != START_PARK.us[j]) {
        Serial.println("TRIAL_REFUSED reason=START_PARK_REQUIRED command_us=1500,1200,1500,2100"); return;
      }
    // Check every endpoint before accepting the first physical MOVE.
    for (size_t i = 0; !aimOnly && !batch && !checkOnly && i < STEP_COUNT; ++i) for (uint8_t j = 0; j < 4; ++j)
      if (STEPS[i].pose.us[j] < settings_.minUs[j] || STEPS[i].pose.us[j] > settings_.maxUs[j]) {
        Serial.println("TRIAL_REFUSED reason=ROUTE_OUTSIDE_LIMITS"); return;
      }
    if (!motion_.hold(now, permit, true)) {
      Serial.println("TRIAL_REFUSED reason=MOTOR_PERMISSION_LOST"); return;
    }
    const uint32_t previous = session_;
    session_ = esp_random();
    if (!session_ || session_ == previous) { session_ = previous + 1; if (!session_) session_ = 1; }
    sequence_ = 0; boot_ = 0; step_ = 0; started_ = now; error_ = "NONE";
    checkOnly_ = checkOnly; checkReturning_ = false;
    batch_ = batch && !checkOnly; aimOnly_ = aimOnly && !batch && !checkOnly;
    sortAttempts_ = 0;
    for (uint8_t b = 0; b < sorter::kColorCount; ++b) sorted_[b] = 0;
    request(Before, now);
  }
  void checkCommand(bool back, uint32_t now, bool permit) {
    if (!checkOnly_ || state_ != Paused || checkReturning_) {
      Serial.println("CHECK_REFUSED reason=PAUSED_CHECKPICK_REQUIRED; command not queued"); return;
    }
    if (uint32_t(now - started_) >= timeLimit()) {
      motion_.disarm(); fail("CHECK_TIME_LIMIT"); return;
    }
    if (!motion_.hold(now, permit, true)) {
      if (motion_.outputsOn()) motion_.disarm();
      fail("MOTOR_STOPPED"); return;
    }
    if (!back && step_ >= 3) {
      Serial.println("CHECK_REFUSED reason=LOW_REACHED; record XYZ then BACK; gripper stays open"); return;
    }
    if (back) { checkReturning_ = true; --step_; }
    else ++step_;
    move(now, permit);
  }
  void cancel(const char* reason) {
    if (!busy()) return;
    // Clearing only the camera state would leave an accepted MOVE running.
    motion_.disarm();
    state_ = Failed; error_ = reason;
    Serial.printf("%s_CANCELLED reason=%s\n", prefix(), reason);
  }
  void tick(uint32_t now, bool permit) {
    if (!busy()) return;
    if (!motion_.outputsOn() || motion_.state() == sorter::JogState::Fault || !permit) {
      if (motion_.outputsOn()) motion_.disarm();
      fail("MOTOR_STOPPED"); return;
    }
    if (uint32_t(now - started_) >= timeLimit()) {
      motion_.disarm(); fail(timeoutReason()); return;
    }
    if (state_ == Before || state_ == Verify) {
      if (uint32_t(now - requested_) >= REPLY_TIMEOUT_MS) fail("CAMERA_TIMEOUT");
    } else if (state_ == Moving && motion_.state() == sorter::JogState::Holding) {
      state_ = Dwell; dwellAt_ = now;
    } else if (state_ == Dwell && uint32_t(now - dwellAt_) >= currentStep().dwellMs) {
      if (checkOnly_) { advanceCheck(now, permit); return; }
      if (++step_ == stepCount()) {
        if (aimOnly_) {
          state_ = Done;
          Serial.println("AIM_RESULT holding_above_target=1; gripper open; no descent or grip.");
        } else request(Verify, now);
      } else move(now, permit);
    }
  }
  void receive(const sorter::Packet& packet, uint32_t now, bool permit) {
    if ((state_ != Before && state_ != Verify) || packet.session != session_ || packet.seq != sequence_ ||
        (packet.type != 'D' && packet.type != 'E')) return;
    if (uint32_t(now - requested_) >= REPLY_TIMEOUT_MS) { fail("CAMERA_TIMEOUT"); return; }
    if (uint32_t(now - started_) >= timeLimit()) {
      motion_.disarm(); fail(timeoutReason()); return;
    }
    if (!motion_.outputsOn() || motion_.state() != sorter::JogState::Holding || !permit) {
      if (motion_.outputsOn()) motion_.disarm();
      fail("MOTOR_STOPPED"); return;
    }
    if (packet.type == 'E') {
      Serial.printf("%s_CAMERA_ERROR code=%s\n", prefix(), packet.payload); fail("CAMERA_REPORTED_ERROR"); return;
    }
    sorter::Scene scene = {};
    if (!sorter::decodeScene(packet.payload, scene) || !scene.cameraBoot) { fail("BAD_CAMERA_SCENE"); return; }
    if (scene.calibrationId != manual::CAMERA_CALIBRATION_ID) { fail("CALIBRATION_MISMATCH"); return; }
    Serial.printf("%s_SCENE count=%u calibrationId=%lu boot=%lu\n", prefix(), scene.count,
      (unsigned long)scene.calibrationId, (unsigned long)scene.cameraBoot);
    if (checkOnly_) {
      if (scene.count != 1) { fail("ONE_PART_REQUIRED"); return; }
      const sorter::MotionCalibration c = camera_aim::sortCalibration();
      uint8_t counts[sorter::kColorCount] = {};
      const char* why = camera_sort::inspectSideScene(scene, c, counts);
      if (why) { fail(why); return; }
      if (!camera_sort::makeSidePlan(scene.objects[0], counts, c, settings_, sortPlan_)) {
        fail("CHECK_ROUTE_INVALID"); return;
      }
      for (size_t i = 0; i < 4; ++i) {
        const char* name = camera_sort::sideStepName(i);
        checkSteps_[i] = {sortPlan_.poses[i], name ? name : STEPS[i].name, 500};
      }
      checkSteps_[4] = {START_PARK, "PARK_RETURN", 800};
      for (size_t i = 0; i < 5; ++i)
        if (checkSteps_[i].pose.us[3] != c.openUs || !withinJogLimits(checkSteps_[i].pose)) {
          fail("CHECK_ROUTE_INVALID"); return;
        }
      boot_ = scene.cameraBoot;
      Serial.printf("CHECK_TARGET class=%u x_mm=%.1f y_mm=%.1f; SIDE_HIGH then pause; remove marked part before NEXT\n",
        scene.objects[0].classId, scene.objects[0].x10/10.0, scene.objects[0].y10/10.0);
      printSideApproach(scene.objects[0]);
      for (size_t i = 1; i <= 3; ++i) {
        const sorter::JointPose& p = checkSteps_[i].pose;
        Serial.printf("CHECK_PLAN name=%s command_us=%u,%u,%u,%u; predicted commands, not measured XYZ\n",
          checkName(i), p.us[0], p.us[1], p.us[2], p.us[3]);
      }
      move(now, permit); return;
    }
    if (batch_) {
      receiveSortScene(scene, now, permit);
      return;
    }
    if (state_ == Verify) {
      if (scene.cameraBoot != boot_) { fail("CAMERA_RESTART"); return; }
      if (scene.count) { fail("SCENE_NOT_EMPTY_AFTER_TRANSFER"); return; }
      state_ = Done;
      Serial.println("TRIAL_RESULT scene_cleared=1 returned_park_command=1; confirm the part is inside the red bin by observation.");
      return;
    }
    if (scene.count != 1 || scene.objects[0].classId != sorter::kRedClassId) {
      fail("ONE_RED_PART_REQUIRED"); return;
    }
    const sorter::Detection& target = scene.objects[0];
    if (aimOnly_) {
      if (!prepareAim(target)) return;
      boot_ = scene.cameraBoot;
      move(now, permit);
      return;
    }
    const int64_t dx = int64_t(target.x10) - PICK_X10, dy = int64_t(target.y10) - PICK_Y10;
    Serial.printf("TRIAL_TARGET color=red bin=1 x_mm=%.1f y_mm=%.1f tolerance_mm=%.1f; fixed pick at 20.0,80.0 mm\n",
      target.x10/10.0, target.y10/10.0, INPUT_TOLERANCE10/10.0);
    if (dx < -INPUT_TOLERANCE10 || dx > INPUT_TOLERANCE10 ||
        dy < -INPUT_TOLERANCE10 || dy > INPUT_TOLERANCE10) {
      fail("PART_OUTSIDE_TRIAL_POINT"); return;
    }
    boot_ = scene.cameraBoot;
    move(now, permit);
  }

 private:
  enum State { Idle, Before, Moving, Dwell, Verify, Paused, Done, Failed };
  sorter::ManualJogController& motion_;
  const sorter::JogSettings& settings_;
  HardwareSerial& camera_;
  State state_;
  uint32_t session_, sequence_, boot_, started_, requested_, dwellAt_;
  size_t step_;
  const char* error_;
  bool aimOnly_;
  Step aimSteps_[2];
  bool batch_;
  uint8_t sorted_[sorter::kColorCount];
  uint8_t sortAttempts_;
  camera_sort::Plan sortPlan_;
  Step sortSteps_[camera_sort::ROUTE_SIZE];
  bool checkOnly_, checkReturning_;
  Step checkSteps_[5];

  const char* prefix() const { return checkOnly_ ? "CHECK" : (batch_ ? "SORT" : (aimOnly_ ? "AIM" : "TRIAL")); }
  uint32_t timeLimit() const { return batch_ || checkOnly_ ? camera_sort::BATCH_LIMIT_MS : TRIAL_LIMIT_MS; }
  const char* timeoutReason() const { return checkOnly_ ? "CHECK_TIME_LIMIT" : (batch_ ? "BATCH_TIME_LIMIT" : "TRIAL_TIME_LIMIT"); }
  size_t stepCount() const { return checkOnly_ ? 5 : (aimOnly_ ? 2 : STEP_COUNT); }
  const Step& currentStep() const {
    return checkOnly_ ? checkSteps_[step_] : (batch_ ? sortSteps_[step_] : (aimOnly_ ? aimSteps_[step_] : STEPS[step_]));
  }
  const char* checkName(size_t index) const { return index == 1 ? "SIDE_HIGH" : (index == 2 ? "SIDE_LOW" : "TARGET_LOW"); }
  void printSideApproach(const sorter::Detection& target) const {
    Serial.printf("%s_APPROACH start_x_mm=%.1f start_y_mm=%.1f target_x_mm=%.1f target_y_mm=%.1f distance_mm=%ld direction=+Y; nominal low Z=10 mm\n",
      prefix(), target.x10/10.0, target.y10/10.0-manual::SIDE_APPROACH_MM,
      target.x10/10.0, target.y10/10.0, (long)manual::SIDE_APPROACH_MM);
  }
  void advanceCheck(uint32_t now, bool permit) {
    if (checkReturning_) {
      if (step_ == 4) {
        state_ = Done;
        Serial.println("CHECK_RESULT returned_park_command=1 gripper_open=1; no grip or bin transfer");
      } else {
        step_ = step_ ? step_ - 1 : 4;
        move(now, permit);
      }
    } else if (step_ == 0) {
      ++step_; move(now, permit);
    } else {
      state_ = Paused;
      const sorter::JointPose& p = motion_.commandedPose();
      Serial.printf("CHECKPOINT name=%s target_x_mm=%.1f target_y_mm=%.1f command_us=%u,%u,%u,%u; measure actual XYZ; %s\n",
        checkName(step_), sortPlan_.target.x10/10.0, sortPlan_.target.y10/10.0,
        p.us[0], p.us[1], p.us[2], p.us[3],
        step_ < 3 ? "NEXT only with part removed, or BACK" : "BACK to return; gripper stays open");
    }
  }
  void receiveSortScene(const sorter::Scene& scene, uint32_t now, bool permit) {
    if (boot_ && scene.cameraBoot != boot_) { fail("CAMERA_RESTART"); return; }
    const sorter::MotionCalibration c = camera_aim::sortCalibration();
    uint8_t counts[sorter::kColorCount] = {};
    const char* why = camera_sort::inspectSideScene(scene, c, counts);
    if (why) { fail(why); return; }
    if (state_ == Verify) {
      why = camera_sort::progressError(scene, sortPlan_, counts);
      if (why) { fail(why); return; }
      const uint8_t bin = sorter::classBinIndex(sortPlan_.target.classId);
      ++sorted_[bin];
      Serial.printf("SORT_PICK_DONE class=%u bin=%u sorted=%u\n", sortPlan_.target.classId,
        unsigned(bin + 1), unsigned(sorted_[0] + sorted_[1]));
    }
    boot_ = scene.cameraBoot;
    if (!scene.count) {
      state_ = Done;
      Serial.printf("SORT_RESULT sorted=%u red=%u blue=%u returned_park_command=1; confirm bins by observation.\n",
        unsigned(sorted_[0] + sorted_[1]), sorted_[0], sorted_[1]);
      return;
    }
    if (sortAttempts_ >= camera_sort::MAX_TRANSFERS) {
      fail("BATCH_PART_LIMIT"); return;
    }
    if (!camera_sort::sidePathClear(scene.objects[0], scene, c)) {
      fail("SIDE_PATH_BLOCKED"); return;
    }
    if (!camera_sort::makeSidePlan(scene.objects[0], counts, c, settings_, sortPlan_)) {
      fail("SORT_ROUTE_INVALID"); return;
    }
    static_assert(camera_sort::ROUTE_SIZE == STEP_COUNT, "SORT dwell table must match its route");
    for (size_t i = 0; i < STEP_COUNT; ++i) {
      sortSteps_[i] = STEPS[i];
      sortSteps_[i].pose = sortPlan_.poses[i];
      const char* name = camera_sort::sideStepName(i);
      if (name) sortSteps_[i].name = name;
    }
    step_ = 0;
    ++sortAttempts_;
    const sorter::Detection& target = sortPlan_.target;
    Serial.printf("SORT_TARGET class=%u color=%s bin=%u x_mm=%.1f y_mm=%.1f remaining=%u\n",
      target.classId, target.classId == sorter::kRedClassId ? "red" : "blue",
      unsigned(sorter::classBinIndex(target.classId) + 1), target.x10/10.0, target.y10/10.0, scene.count);
    printSideApproach(target);
    move(now, permit);
  }
  bool withinJogLimits(const sorter::JointPose& pose) const {
    for (uint8_t j = 0; j < 4; ++j)
      if (pose.us[j] < settings_.minUs[j] || pose.us[j] > settings_.maxUs[j]) return false;
    return true;
  }
  bool prepareAim(const sorter::Detection& target) {
    if (!target.pixels) { fail("BAD_CAMERA_SCENE"); return false; }
    const sorter::MotionCalibration c = camera_aim::upperCalibration();
    if (!sorter::inPickArea(target.x10, target.y10, c)) {
      fail("OUTSIDE_AIM_AREA"); return false;
    }
    sorter::CalibratedPose high = {};
    // Validate both endpoints before leaving PARK. Interpolated poses are
    // predictions; AIMRED stops above the part for the operator to observe.
    if (!sorter::interpolateGrid(c, target.x10, target.y10, true, high) ||
        !sorter::knownPoseValid(c.hubOpen, c, true) ||
        !sorter::knownPoseValid(high, c, false) ||
        !withinJogLimits(c.hubOpen.joints) || !withinJogLimits(high.joints)) {
      fail("AIM_POSE_INVALID"); return false;
    }
    aimSteps_[0] = {c.hubOpen.joints, "HUB_OUT", 250};
    aimSteps_[1] = {high.joints, "ABOVE_RED", 500};
    Serial.printf("AIM_TARGET x_mm=%.1f y_mm=%.1f predicted_z_mm=%.1f command_us=%u,%u,%u,%u\n",
      target.x10/10.0, target.y10/10.0, high.tip.z10/10.0,
      high.joints.us[0], high.joints.us[1], high.joints.us[2], high.joints.us[3]);
    return true;
  }

  void fail(const char* reason) {
    state_ = Failed; error_ = reason;
    // Camera failures occur only while holding PARK; never repeat a transfer.
    Serial.printf("%s_FAILED reason=%s; no automatic retry\n", prefix(), reason);
  }
  void request(State waiting, uint32_t now) {
    sorter::Packet packet = {}; packet.type = 'Q'; packet.session = session_; packet.seq = ++sequence_;
    snprintf(packet.payload, sizeof(packet.payload), "%lu", (unsigned long)manual::CAMERA_CALIBRATION_ID);
    char frame[sorter::kMaxFrame];
    if (!sorter::encodePacket(packet, frame, sizeof(frame))) { fail("ENCODE_FAILED"); return; }
    state_ = waiting; requested_ = now;
    if (camera_.write(reinterpret_cast<const uint8_t*>(frame), strlen(frame)) != strlen(frame)) {
      fail("UART_WRITE_FAILED"); return;
    }
    Serial.printf("%s_SCAN phase=%s session=%lu seq=%lu timeout_ms=%lu\n", prefix(), stateName(),
      (unsigned long)session_, (unsigned long)sequence_, (unsigned long)REPLY_TIMEOUT_MS);
  }
  void move(uint32_t now, bool permit) {
    const Step& next = currentStep();
    if (!motion_.movePose(next.pose, now, permit, true)) {
      if (motion_.outputsOn()) motion_.disarm();
      fail("MOVE_REFUSED"); return;
    }
    state_ = Moving;
    Serial.printf("%s_STEP %u/%u name=%s command_us=%u,%u,%u,%u\n",
      prefix(), unsigned(step_ + 1), unsigned(stepCount()), next.name,
      next.pose.us[0], next.pose.us[1], next.pose.us[2], next.pose.us[3]);
  }
};
} // namespace red_trial
