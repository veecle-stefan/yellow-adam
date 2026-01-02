#include "StatusJson.h"
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

size_t EncodeStatusJson(const CarStatus& st, char* outBuf, size_t outSize)
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
      ",\"lights\":{\"drl\":%s,\"low\":%s,\"high\":%s,\"il\":%s,\"ir\":%s}"
      "}",
      (int)st.t, (int)st.s,
      (int)st.tq_fl, (int)st.tq_fr, (int)st.tq_rl, (int)st.tq_rr,
      (int)st.curr_fl, (int)st.curr_fr, (int)st.curr_rl, (int)st.curr_rr,
      jb(st.drl), jb(st.low), jb(st.high), jb(st.il), jb(st.ir)
    );

  if (!ok) return 0;
  return used;
}