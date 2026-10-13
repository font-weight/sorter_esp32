#pragma once
#include <Arduino.h>
#include <map>
#include <set>
#include <vector>

// In-memory NVS substitute. No flash, files, ports or devices are accessed.
namespace fakenvs {
struct Write {
  std::string key;
  std::vector<uint8_t> bytes;
  uint32_t at;
  bool outputsOn;
};
inline std::map<std::string,std::vector<uint8_t>> records;
inline std::set<std::string> nonBlobKeys;
inline std::vector<Write> writes;
inline bool failBegin=false, failPut=false, failRead=false, corruptRead=false;
inline size_t partialPut=0;
inline unsigned begins=0, reads=0, puts=0;
inline void clear() {
  records.clear(); nonBlobKeys.clear(); writes.clear();
  failBegin=failPut=failRead=corruptRead=false; partialPut=0;
  begins=reads=puts=0;
}
}
class Preferences {
 public:
  bool begin(const char* name,bool) {
    ++fakenvs::begins; namespace_=name; open_=!fakenvs::failBegin; return open_;
  }
  void end() { open_=false; }
  bool isKey(const char* key) {
    return open_ && (fakenvs::records.count(fullKey(key))!=0 || fakenvs::nonBlobKeys.count(fullKey(key))!=0);
  }
  size_t getBytesLength(const char* key) {
    if (!open_) return 0;
    const auto it=fakenvs::records.find(fullKey(key));
    return it==fakenvs::records.end()?0:it->second.size();
  }
  size_t getBytes(const char* key,void* data,size_t size) {
    ++fakenvs::reads;
    if (!open_ || fakenvs::failRead) return 0;
    const auto it=fakenvs::records.find(fullKey(key));
    if (it==fakenvs::records.end() || size<it->second.size()) return 0;
    const size_t length=it->second.size();
    std::memcpy(data,it->second.data(),length);
    if (fakenvs::corruptRead && length) static_cast<uint8_t*>(data)[0]^=1;
    return length;
  }
  size_t putBytes(const char* key,const void* data,size_t size) {
    ++fakenvs::puts;
    if (!open_ || fakenvs::failPut) return 0;
    const size_t length=fakenvs::partialPut?std::min(size,fakenvs::partialPut):size;
    const auto* bytes=static_cast<const uint8_t*>(data);
    const std::vector<uint8_t> value(bytes,bytes+length);
    const std::string full=fullKey(key);
    fakenvs::records[full]=value;
    bool on=false;
    for (bool attached : fakehw::attached) on=on||attached;
    fakenvs::writes.push_back({full,value,millis(),on});
    return length;
  }
 private:
  std::string namespace_;
  bool open_=false;
  std::string fullKey(const char* key) const { return namespace_+"/"+key; }
};
