#pragma once
#include <Arduino.h>
#include <esp_system.h>
#include <Protocol.h>
#include <ManualJogCore.h>
#include "Config.h"
#include "ArmMap.h"
#include "SortPlan.h"

namespace app {
constexpr uint32_t REPLY_TIMEOUT_MS = 6000;

class SortController {
 public:
  SortController(sorter::ManualJogController& motion, const sorter::JogSettings& settings,
                 HardwareSerial& camera)
      : motion_(motion), settings_(settings), camera_(camera), state_(Idle),
        session_(0), sequence_(0), boot_(0), started_(0), requested_(0), dwellAt_(0),
        step_(0), error_("NONE"), sorted_{}, sortAttempts_(0), plan_{} {}

  bool busy() const { return state_ == Before || state_ == Moving || state_ == Dwell || state_ == Verify; }
  const char* stateName() const {
    switch (state_) {
      case Idle: return "IDLE";
      case Before: return "WAIT_CAMERA";
      case Moving: return "MOVING";
      case Dwell: return "DWELL";
      case Verify: return "VERIFY_CAMERA";
      case Done: return "DONE";
      default: return "FAILED";
    }
  }
  const char* error() const { return error_; }
  uint8_t sortedRed() const { return sorted_[0]; }
  uint8_t sortedBlue() const { return sorted_[1]; }
  uint8_t sortAttempts() const { return sortAttempts_; }

  bool start(uint32_t now, bool uartReady, bool permit) {
    if (busy()) { Serial.println("SORT_REFUSED reason=BUSY"); return false; }
    if (!uartReady) return refuse("CAMERA_UART_DISABLED");
    if (!motion_.outputsOn() || motion_.state() != sorter::JogState::Holding)
      return refuse("ARM_IN_HOLDING_REQUIRED");
    for (uint8_t j = 0; j < 4; ++j)
      if (motion_.commandedPose().us[j] != arm_map::PARK.us[j])
        return refuse("START_PARK_REQUIRED");
    if (!motion_.hold(now, permit, true)) return refuse("MOTOR_PERMISSION_LOST");
    const uint32_t previous = session_;
    session_ = esp_random();
    if (!session_ || session_ == previous) { session_ = previous + 1; if (!session_) session_ = 1; }
    sequence_ = 0; boot_ = 0; step_ = 0; started_ = now; error_ = "NONE";
    sortAttempts_ = 0;
    for (uint8_t b = 0; b < sorter::kColorCount; ++b) sorted_[b] = 0;
    return request(Before, now);
  }

  void cancel(const char* reason) {
    if (!busy()) return;
    motion_.disarm();
    state_ = Failed; error_ = reason;
    Serial.printf("SORT_CANCELLED reason=%s\n", reason);
  }

  void tick(uint32_t now, bool permit) {
    if (!busy()) return;
    if (!motion_.outputsOn() || motion_.state() == sorter::JogState::Fault || !permit) {
      if (motion_.outputsOn()) motion_.disarm();
      fail("MOTOR_STOPPED"); return;
    }
    if (uint32_t(now - started_) >= camera_sort::BATCH_LIMIT_MS) {
      motion_.disarm(); fail("BATCH_TIME_LIMIT"); return;
    }
    if (state_ == Before || state_ == Verify) {
      if (uint32_t(now - requested_) >= REPLY_TIMEOUT_MS) fail("CAMERA_TIMEOUT");
    } else if (state_ == Moving && motion_.state() == sorter::JogState::Holding) {
      state_ = Dwell; dwellAt_ = now;
    } else if (state_ == Dwell && uint32_t(now - dwellAt_) >= stepInfo(step_).dwellMs) {
      if (++step_ == camera_sort::ROUTE_SIZE) request(Verify, now);
      else move(now, permit);
    }
  }

  // Feed only packets accepted by Protocol::LineParser (framing and CRC).
  void receive(const sorter::Packet& packet, uint32_t now, bool permit) {
    if ((state_ != Before && state_ != Verify) || packet.session != session_ || packet.seq != sequence_ ||
        (packet.type != 'D' && packet.type != 'E')) return;
    if (uint32_t(now - requested_) >= REPLY_TIMEOUT_MS) { fail("CAMERA_TIMEOUT"); return; }
    if (uint32_t(now - started_) >= camera_sort::BATCH_LIMIT_MS) {
      motion_.disarm(); fail("BATCH_TIME_LIMIT"); return;
    }
    if (!motion_.outputsOn() || motion_.state() != sorter::JogState::Holding || !permit) {
      if (motion_.outputsOn()) motion_.disarm();
      fail("MOTOR_STOPPED"); return;
    }
    if (packet.type == 'E') {
      Serial.printf("SORT_CAMERA_ERROR code=%s\n", packet.payload);
      fail("CAMERA_REPORTED_ERROR"); return;
    }
    sorter::Scene scene = {};
    if (!sorter::decodeScene(packet.payload, scene) || !scene.cameraBoot) { fail("BAD_CAMERA_SCENE"); return; }
    if (scene.calibrationId != CAMERA_CALIBRATION_ID) { fail("CALIBRATION_MISMATCH"); return; }
    if (boot_ && scene.cameraBoot != boot_) { fail("CAMERA_RESTART"); return; }
    Serial.printf("SORT_SCENE count=%u calibrationId=%lu boot=%lu\n", scene.count,
      (unsigned long)scene.calibrationId, (unsigned long)scene.cameraBoot);
    const sorter::MotionCalibration c = arm_map::calibration();
    uint8_t counts[sorter::kColorCount] = {};
    const char* why = camera_sort::inspectSideScene(scene, c, counts);
    if (why) { fail(why); return; }
    if (state_ == Verify) {
      why = camera_sort::progressError(scene, plan_, counts);
      if (why) { fail(why); return; }
      const uint8_t bin = sorter::classBinIndex(plan_.target.classId);
      ++sorted_[bin];
      Serial.printf("SORT_PICK_DONE class=%u bin=%u sorted=%u\n", plan_.target.classId,
        unsigned(bin + 1), unsigned(sorted_[0] + sorted_[1]));
    }
    boot_ = scene.cameraBoot;
    if (!scene.count) {
      state_ = Done;
      Serial.printf("SORT_RESULT sorted=%u red=%u blue=%u returned_park_command=1\n",
        unsigned(sorted_[0] + sorted_[1]), sorted_[0], sorted_[1]);
      return;
    }
    if (sortAttempts_ >= camera_sort::MAX_TRANSFERS) { fail("BATCH_PART_LIMIT"); return; }
    if (!camera_sort::sidePathClear(scene.objects[0], scene, c)) { fail("SIDE_PATH_BLOCKED"); return; }
    if (!camera_sort::makeSidePlan(scene.objects[0], counts, c, settings_, plan_)) {
      fail("SORT_ROUTE_INVALID"); return;
    }
    step_ = 0;
    ++sortAttempts_;
    const sorter::Detection& target = plan_.target;
    Serial.printf("SORT_TARGET class=%u color=%s bin=%u x_mm=%.1f y_mm=%.1f remaining=%u\n",
      target.classId, target.classId == sorter::kRedClassId ? "red" : "blue",
      unsigned(sorter::classBinIndex(target.classId) + 1), target.x10/10.0, target.y10/10.0, scene.count);
    Serial.printf("SORT_APPROACH start_x_mm=%.1f start_y_mm=%.1f target_x_mm=%.1f target_y_mm=%.1f distance_mm=%ld direction=+Y; nominal low Z=10 mm\n",
      target.x10/10.0, target.y10/10.0-SIDE_APPROACH_MM,
      target.x10/10.0, target.y10/10.0, (long)SIDE_APPROACH_MM);
    move(now, permit);
  }

 private:
  enum State { Idle, Before, Moving, Dwell, Verify, Done, Failed };
  struct StepInfo { const char* name; uint32_t dwellMs; };
  sorter::ManualJogController& motion_;
  const sorter::JogSettings& settings_;
  HardwareSerial& camera_;
  State state_;
  uint32_t session_, sequence_, boot_, started_, requested_, dwellAt_;
  size_t step_;
  const char* error_;
  uint8_t sorted_[sorter::kColorCount];
  uint8_t sortAttempts_;
  camera_sort::Plan plan_;

  static const StepInfo& stepInfo(size_t index) {
    static const StepInfo steps[] = {
      {"HUB_OUT", 250}, {"SIDE_HIGH", 250}, {"SIDE_LOW", 250}, {"ENTER_Y", 250},
      {"GRIP", 1000}, {"LIFT_NEAR", 250}, {"LIFT", 250}, {"HUB_LOADED", 250},
      {"BIN_APPROACH", 250}, {"BIN_LOWER", 1000}, {"RELEASE", 1000},
      {"BIN_LEAVE", 250}, {"HUB_RETURN", 250}, {"PARK_RETURN", 800}
    };
    static_assert(sizeof(steps) / sizeof(steps[0]) == camera_sort::ROUTE_SIZE,
                  "SORT dwell table must match its route");
    return steps[index];
  }

  bool refuse(const char* reason) {
    error_ = reason;
    Serial.printf("SORT_REFUSED reason=%s\n", reason);
    return false;
  }

  void fail(const char* reason) {
    state_ = Failed; error_ = reason;
    // Camera/scene failures occur while holding PARK. Motion failures disarm.
    Serial.printf("SORT_FAILED reason=%s; no automatic retry\n", reason);
  }

  bool request(State waiting, uint32_t now) {
    sorter::Packet packet = {}; packet.type = 'Q'; packet.session = session_; packet.seq = ++sequence_;
    snprintf(packet.payload, sizeof(packet.payload), "%lu", (unsigned long)CAMERA_CALIBRATION_ID);
    char frame[sorter::kMaxFrame];
    if (!sorter::encodePacket(packet, frame, sizeof(frame))) { fail("ENCODE_FAILED"); return false; }
    state_ = waiting; requested_ = now;
    const size_t length = strlen(frame);
    if (camera_.write(reinterpret_cast<const uint8_t*>(frame), length) != length) {
      fail("UART_WRITE_FAILED"); return false;
    }
    Serial.printf("SORT_SCAN phase=%s session=%lu seq=%lu timeout_ms=%lu\n", stateName(),
      (unsigned long)session_, (unsigned long)sequence_, (unsigned long)REPLY_TIMEOUT_MS);
    return true;
  }

  void move(uint32_t now, bool permit) {
    const sorter::JointPose& next = plan_.poses[step_];
    if (!motion_.movePose(next, now, permit, true)) {
      if (motion_.outputsOn()) motion_.disarm();
      fail("MOVE_REFUSED"); return;
    }
    state_ = Moving;
    Serial.printf("SORT_STEP %u/%u name=%s command_us=%u,%u,%u,%u\n",
      unsigned(step_ + 1), unsigned(camera_sort::ROUTE_SIZE), stepInfo(step_).name,
      next.us[0], next.us[1], next.us[2], next.us[3]);
  }
};
} // namespace app
