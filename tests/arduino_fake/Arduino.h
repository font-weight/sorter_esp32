#pragma once
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <algorithm>
using std::min;
constexpr int HIGH=1, LOW=0, INPUT=0, OUTPUT=1, INPUT_PULLDOWN=2, FALLING=3, INPUT_PULLUP=4;
#define ARDUINO_ISR_ATTR
#define SERIAL_8N1 0
namespace fakehw {
inline uint32_t now=0, adcMv=1667;
inline int levels[64]={}, modes[64]={};
inline bool attached[64]={};
inline uint32_t duties[64]={};
inline void (*interrupts[64])()={};
inline unsigned attachCalls=0, detachCalls=0, writeCalls=0, adcReads=0;
inline unsigned modeCalls[64]={}, readCalls[64]={}, digitalWrites[64]={};
inline int failAttach=-1, failWrite=-1;
inline void clear() {
  now=0; adcMv=1667;
  std::memset(levels,0,sizeof(levels)); std::memset(modes,0,sizeof(modes));
  std::memset(attached,0,sizeof(attached)); std::memset(duties,0,sizeof(duties));
  std::memset(interrupts,0,sizeof(interrupts));
  std::memset(modeCalls,0,sizeof(modeCalls)); std::memset(readCalls,0,sizeof(readCalls));
  std::memset(digitalWrites,0,sizeof(digitalWrites));
  attachCalls=detachCalls=writeCalls=adcReads=0; failAttach=failWrite=-1;
}
}
inline uint32_t millis(){return fakehw::now;}
inline void delay(unsigned ms){fakehw::now+=ms;}
inline int digitalRead(uint8_t pin){++fakehw::readCalls[pin];return fakehw::levels[pin];}
inline void digitalWrite(uint8_t pin,int level){++fakehw::digitalWrites[pin];fakehw::levels[pin]=level;}
inline void pinMode(uint8_t pin,int mode){++fakehw::modeCalls[pin];fakehw::modes[pin]=mode;}
inline int digitalPinToInterrupt(uint8_t pin){return pin;}
inline void attachInterrupt(int pin,void (*isr)(),int){fakehw::interrupts[pin]=isr;}
inline uint32_t analogReadMilliVolts(uint8_t){++fakehw::adcReads;return fakehw::adcMv;}
inline bool ledcAttach(uint8_t pin,unsigned hz,unsigned bits){
  ++fakehw::attachCalls;
  if(int(pin)==fakehw::failAttach||hz!=50||bits!=14)return false;
  fakehw::attached[pin]=true;return true;
}
inline bool ledcWrite(uint8_t pin,uint32_t value){
  ++fakehw::writeCalls;
  if(!fakehw::attached[pin]||int(pin)==fakehw::failWrite)return false;
  fakehw::duties[pin]=value;return true;
}
inline bool ledcDetach(uint8_t pin){++fakehw::detachCalls;fakehw::attached[pin]=false;return true;}
struct FakeSerial {
  std::string log,input;size_t cursor=0;
  void begin(unsigned){}
  void print(const char* text){log+=text;}
  void println(const char* text){log+=text;log+='\n';}
  template<class... Args> void printf(const char* fmt,Args... args){char b[768];std::snprintf(b,sizeof(b),fmt,args...);log+=b;}
  int available(){return int(input.size()-cursor);}
  int read(){return input[cursor++];}
};
inline FakeSerial Serial;
struct HardwareSerial : FakeSerial {
  explicit HardwareSerial(int){}
  bool writeOkay=true;bool initialized=false;int rx=-1,tx=-1;unsigned baud=0;
  void begin(unsigned speed,int,int rxPin,int txPin){initialized=true;baud=speed;rx=rxPin;tx=txPin;}
  size_t write(const uint8_t* bytes,size_t length){if(!writeOkay)return 0;log.append(reinterpret_cast<const char*>(bytes),length);return length;}
};
