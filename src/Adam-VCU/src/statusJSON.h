#pragma once
#include <Arduino.h>
#include <stdint.h>

struct CarStatus {
  int16_t t = 0;   // throttle  [-1000..1000]
  int16_t s = 0;   // steering  [-1000..1000]

  int16_t tq_fl = 0;
  int16_t tq_fr = 0;
  int16_t tq_rl = 0;
  int16_t tq_rr = 0;

  int16_t curr_fl = 0;
  int16_t curr_fr = 0;
  int16_t curr_rl = 0;
  int16_t curr_rr = 0;

  bool drl  = false;
  bool low  = false;
  bool high = false;
  bool il   = false;
  bool ir   = false;
};

// Encodes status JSON into outBuf. Returns number of bytes written (excluding '\0').
// Returns 0 if buffer too small.
size_t EncodeStatusJson(const CarStatus& st, char* outBuf, size_t outSize);