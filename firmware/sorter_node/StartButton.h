#pragma once
#include <stdint.h>
#include "Config.h"
namespace app {
class StartButton {
 public:
  void begin(bool pressed,uint32_t now) { raw_=stable_=pressed; changed_=now; armed_=!pressed; }
  bool tick(bool pressed,uint32_t now) {
    if (pressed!=raw_) { raw_=pressed; changed_=now; }
    if (raw_==stable_ || uint32_t(now-changed_)<BUTTON_DEBOUNCE_MS) return false;
    stable_=raw_;
    if (!stable_) { armed_=true; return false; }
    const bool event=armed_; armed_=false; return event;
  }
 private:
  bool raw_=false, stable_=false, armed_=false;
  uint32_t changed_=0;
};
} // namespace app
