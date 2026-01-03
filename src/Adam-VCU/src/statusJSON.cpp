#include "statusJSON.h"
#include <stdio.h>   // snprintf
#include <stdarg.h>  // va_list

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

size_t EncodeStatusJson(const DriveTrainStatus& st, char* outBuf, size_t outSize)
{
  if (!outBuf || outSize < 16) return 0;

  size_t used = 0;

  // Keep schema stable: always emit all keys.
  bool ok =
    appendf(outBuf, outSize, used,
      "{\"type\":\"status\""
      ",\"t\":%d,\"s\":%d"
      ",\"torque\":{\"fl\":%d,\"fr\":%d,\"rl\":%d,\"rr\":%d}"
      ",\"curr\":{\"fl\":%d,\"fr\":%d,\"rl\":%d,\"rr\":%d}"
      ",\"boards\":{\"vf\":%u,\"vr\":%u,\"tf\":%u,\"tr\":%u}"
      ",\"vehicle\":{\"gear\":\"%c\",\"low\":%s,\"high\":%s,\"il\":%s,\"ir\":%s}"
      "}",
      (int)st.throttle, (int)st.steering,
      (int)st.tq_fl, (int)st.tq_fr, (int)st.tq_rl, (int)st.tq_rr,
      (int)st.curr_fl, (int)st.curr_fr, (int)st.curr_rl, (int)st.curr_rr,
      (unsigned int)st.voltage_front, (unsigned int)st.voltage_rear, (unsigned int)st.temp_front, (unsigned int)st.temp_rear,
      gearName[st.state.currGear], jb(st.state.loBeam), jb(st.state.hiBeam), jb(st.state.indicatorsL), jb(st.state.indicatorsR)
    );

  if (!ok) return 0;
  return used;
}