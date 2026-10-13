#pragma once
#include <Preferences.h>
#include <Protocol.h>
#include "Config.h"

namespace app {
// Two alternating NVS records. Each putBytes is committed by Preferences.
// This stores commanded pulse widths, never a measured physical position.
class PoseStore {
 public:
  bool begin() { ready_ = preferences_.begin("sorter-pose", false); return ready_; }
  bool load(sorter::JointPose& pose, bool& empty) {
    empty = false;
    if (!ready_) return false;
    uint8_t data[2][SIZE] = {};
    uint32_t seq[2] = {};
    sorter::JointPose poses[2] = {};
    bool valid[2] = {};
    size_t lengths[2] = {};
    for (unsigned i=0;i<2;++i) {
      lengths[i] = preferences_.getBytesLength(key(i));
      valid[i] = lengths[i]==SIZE && preferences_.getBytes(key(i),data[i],SIZE)==SIZE &&
                 decode(data[i],poses[i],seq[i]);
    }
    if (!valid[0] && !valid[1]) {
      empty = !preferences_.isKey(key(0)) && !preferences_.isKey(key(1));
      return empty; // Existing corrupt/incompatible data must not be called first boot.
    }
    active_ = valid[1] && (!valid[0] || newer(seq[1],seq[0])) ? 1 : 0;
    sequence_ = seq[active_]; pose = saved_ = poses[active_]; hasSaved_ = true;
    return true;
  }
  bool save(const sorter::JointPose& pose) {
    if (!ready_ || !poseValid(pose)) return false;
    if (hasSaved_ && same(pose,saved_)) return true;
    const unsigned slot = active_==0 ? 1 : 0;
    uint8_t data[SIZE] = {}, verify[SIZE] = {};
    put32(data,0x53525450u); put32(data+4,POSE_PROFILE_ID);
    put32(data+8,sequence_+1); put16(data+12,1);
    for (unsigned j=0;j<4;++j) put16(data+14+j*2,pose.us[j]);
    put16(data+22,sorter::crc16(data,22));
    if (preferences_.putBytes(key(slot),data,SIZE)!=SIZE ||
        preferences_.getBytes(key(slot),verify,SIZE)!=SIZE || memcmp(data,verify,SIZE)) return false;
    active_ = slot; ++sequence_; saved_ = pose; hasSaved_ = true;
    return true;
  }
  static bool same(const sorter::JointPose& a,const sorter::JointPose& b) {
    for (unsigned j=0;j<4;++j) if (a.us[j]!=b.us[j]) return false;
    return true;
  }
 private:
  static constexpr size_t SIZE = 24;
  Preferences preferences_;
  bool ready_ = false, hasSaved_ = false;
  unsigned active_ = 1;
  uint32_t sequence_ = 0;
  sorter::JointPose saved_ = {};
  static const char* key(unsigned slot) { return slot ? "pose-b" : "pose-a"; }
  static void put16(uint8_t* p,uint16_t n) { p[0]=uint8_t(n); p[1]=uint8_t(n>>8); }
  static void put32(uint8_t* p,uint32_t n) { put16(p,uint16_t(n)); put16(p+2,uint16_t(n>>16)); }
  static uint16_t get16(const uint8_t* p) { return uint16_t(p[0]) | (uint16_t(p[1])<<8); }
  static uint32_t get32(const uint8_t* p) { return uint32_t(get16(p)) | (uint32_t(get16(p+2))<<16); }
  static bool newer(uint32_t a,uint32_t b) { return a!=b && uint32_t(a-b)<0x80000000u; }
  static bool decode(const uint8_t* data,sorter::JointPose& pose,uint32_t& seq) {
    if (get32(data)!=0x53525450u || get32(data+4)!=POSE_PROFILE_ID || get16(data+12)!=1 ||
        get16(data+22)!=sorter::crc16(data,22)) return false;
    for (unsigned j=0;j<4;++j) pose.us[j]=get16(data+14+j*2);
    seq=get32(data+8);
    return poseValid(pose);
  }
};
} // namespace app
