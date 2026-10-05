#pragma once
#include <Protocol.h>
#include <string.h>

namespace sorter {
enum class RequestDecision { New, Duplicate, Conflict, Stale, Invalid };

// Synchronous camera service: accept -> capture/encode -> remember. Repeated
// Q returns byte-identical D/E without a second exposure. Older sequence values
// in the current session are rejected. Sequence wrap requires a new session.
class CameraSession {
 public:
  CameraSession() : session_(0), seq_(0), calibration_(0) { frame_[0] = '\0'; }
  RequestDecision accept(uint32_t session, uint32_t seq, uint32_t calibration) {
    if (!session || !seq || !calibration) return RequestDecision::Invalid;
    if (session == session_) {
      if (seq < seq_) return RequestDecision::Stale;
      if (seq == seq_) return calibration == calibration_ ?
                           RequestDecision::Duplicate : RequestDecision::Conflict;
    }
    session_ = session; seq_ = seq; calibration_ = calibration;
    frame_[0] = '\0';
    return RequestDecision::New;
  }
  bool remember(const char* frame) {
    if (!frame) return false;
    const size_t length = strlen(frame);
    if (!length || length >= sizeof(frame_)) return false;
    memcpy(frame_, frame, length + 1);
    return true;
  }
  const char* response() const { return frame_; }
 private:
  uint32_t session_, seq_, calibration_;
  char frame_[kMaxFrame];
};
} // namespace sorter
