#include "JSONrw.h"
#include <stdio.h>   // snprintf
#include <stdarg.h>  // va_list

JSONInteraction::JSONInteraction(size_t bufferSize):
bufferSize(bufferSize)
{
  this->buffer = new char[bufferSize];
  configASSERT(this->buffer);
}

static bool appendf(char* buf, size_t cap, size_t& used, const char* fmt, ...) {
  if (used >= cap) return false;
  va_list args;
  va_start(args, fmt);
  int n = vsnprintf(buf + used, cap - used, fmt, args);
  va_end(args);
  if (n <= 0) return false;
  if ((size_t)n >= (cap - used)) return false; // truncated
  used += (size_t)n;
  return true;
}

static const char* jb(bool v) { return v ? "true" : "false"; }
static const char gearName[] = {'N', 'D', 'R'};

bool JSONInteraction::DispatchCommand(const String& msg, DriveTrain* drive)
{
  if (!drive) return false;

  // Minimal, robust parsing using String operations (no full JSON lib)
  // NOTE: legacy {"type":"control",...} messages are deprecated.
  // Use {"type":"cmd","name":"extctrl","on":true} to enable/disable external control
  // and {"type":"cmd","name":"steer","t":123,"s":-45} to send steer/throttle commands.

  // Button/command messages: {"type":"cmd","name":"ind_l","on":true} 
  if (msg.indexOf("\"type\":\"cmd\"") >= 0) {
    int p = msg.indexOf("\"name\":\"");
    if (p < 0) return false;
    p += 8;
    int q = msg.indexOf('"', p);
    if (q < 0) return false;
    String name = msg.substring(p, q);

    const bool on = msg.indexOf("\"on\":true") >= 0;

    // indicators: preserve other side by reading latest status
    if (name == "ind_l" || name == "ind_r") {
      DriveTrainStatus st{};
      drive->GetLatestStatus(st);
      bool left = st.state.indicatorsL;
      bool right = st.state.indicatorsR;
      if (name == "ind_l") left = on; else right = on;
      drive->SendIndicators(left, right);
      return true;
    }

    // External control enable/disable: {"type":"cmd","name":"extctrl","on":true}
    if (name == "extctrl") {
      drive->SendExternalControl(on);
      return true;
    }

    // steer command: {"type":"cmd","name":"steer","t":123,"s":-45}
    if (name == "steer") {
      int t = 0, s = 0;
      int p = msg.indexOf("\"t\":");
      if (p >= 0) t = msg.substring(p + 4).toInt();
      p = msg.indexOf("\"s\":");
      if (p >= 0) s = msg.substring(p + 4).toInt();
      drive->SendSteer((int16_t)t, (int16_t)s);
      return true;
    }

   // gear command: {"type":"cmd","name":"gear","gear":"D"}
    if (name == "gear") {
      int p = msg.indexOf("\"gear\":\"");
      if (p < 0) return false;

      p += 8;
      int q = msg.indexOf('"', p);
      if (q < 0 || q <= p) return false;

      char g = msg.charAt(p);
      if (g == 'D') drive->SendGear(Gear::D);
      else if (g == 'R') drive->SendGear(Gear::R);
      else drive->SendGear(Gear::N);
      return true;
    }

    // power limit: {"type":"cmd","name":"limit","maxThrottle":123,"maxSpeed":456}
    if (name == "limit" || name == "powerlimit") {
      int mt = -1, mr = -1, mf = -1;
      p = msg.indexOf("\"maxThrottle\":");
      if (p >= 0) mt = msg.substring(p + 14).toInt();
      p = msg.indexOf("\"maxSpeedFwd\":");
      if (p >= 0) mf = msg.substring(p + 14).toInt();
      p = msg.indexOf("\"maxSpeedRev\":");
      if (p >= 0) mr = msg.substring(p + 14).toInt();
      if (mt >= 0 && mf >= 0) {
        drive->SendPowerLimit((uint16_t)mt, (uint16_t)mf, (uint16_t)mr);
        return true;
      }
      return false;
    }

    // headlights / drl: name == "low" | "high" | "drl" (mode: 1=drl,2=low,3=high)
    if (name == "drl" || name == "low" || name == "high") {
      uint8_t mode = (name == "drl") ? 1 : (name == "low") ? 2 : 3;
      drive->SendHeadlight(mode, on);
      return true;
    }

    if (name == "poweroff") {
      drive->SendPowerOff();
      return true;
    }

    // tuning: {"type":"cmd","name":"tune_tv","id":12,"v":0.35}
    if (name == "tune_tv")
    {
      int id = -1;
      float v = 0.f;

      int p = msg.indexOf("\"id\":");
      if (p >= 0)
        id = msg.substring(p + 5).toInt();

      p = msg.indexOf("\"v\":");
      if (p >= 0)
        v = msg.substring(p + 4).toFloat();

      if (id >= 0)
      {
        drive->SendTuneTV((uint16_t)id, v); // queues DriveCommand::TuneTVParam
        return true;
      }
      return false;
    }
  }

  // Unknown / unsupported
  return false;
}

size_t JSONInteraction::EncodeStatusJson(const DriveTrainStatus& st)
{
  
  size_t used = 0;

  // Keep schema stable: always emit all keys.
  bool ok =
    appendf(buffer, bufferSize, used,
      "{\"type\":\"status\""
      ",\"t\":%d,\"s\":%d"
      ",\"torque\":{\"fl\":%d,\"fr\":%d,\"rl\":%d,\"rr\":%d}"
      ",\"curr\":{\"fl\":%d,\"fr\":%d,\"rl\":%d,\"rr\":%d}"
      ",\"vel\": {\"fl\":%d,\"fr\":%d,\"rl\":%d,\"rr\":%d}"
      ",\"boards\":{\"vf\":%u,\"vr\":%u,\"tf\":%u,\"tr\":%u}"
      ",\"vehicle\":{\"gear\":\"%c\",\"low\":%s,\"high\":%s,\"il\":%s,\"ir\":%s}"
      "}",
      (int)st.throttle, (int)st.steering, // t:, s:, 
      (int)st.tq_fl, (int)st.tq_fr, (int)st.tq_rl, (int)st.tq_rr, // torque {},
      (int)st.curr_fl, (int)st.curr_fr, (int)st.curr_rl, (int)st.curr_rr, // curr {},
      (int)st.vel_fl,  (int)st.vel_fr,  (int)st.vel_rl,  (int)st.vel_rr, // vel {},
      (unsigned int)st.voltage_front, (unsigned int)st.voltage_rear, (unsigned int)st.temp_front, (unsigned int)st.temp_rear,
      gearName[st.state.currGear], jb(st.state.loBeam), jb(st.state.hiBeam), jb(st.state.indicatorsL), jb(st.state.indicatorsR)
    );

  if (!ok) return 0;
  return used;
}