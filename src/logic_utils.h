#pragma once

#include <stdint.h>

#include "alarm_types.h"

// Some build environments (e.g. native PlatformIO tests) will not provide the
// Adafruit ADS library.  We only need the gain enumeration for the helper
// logic, so provide a lightweight stand-in when the library is unavailable.

// Use the ARDUINO macro (or presence of the library header) to avoid
// redefining symbols when the real implementation is compiled in firmware
// builds.
#if !defined(ARDUINO) && !defined(_ADAFRUIT_ADS1X15_H_)
typedef enum {
  GAIN_TWOTHIRDS = 0,
  GAIN_ONE,
  GAIN_TWO,
  GAIN_FOUR,
  GAIN_EIGHT,
  GAIN_SIXTEEN
} adsGain_t;
#endif

namespace logic {

struct AlarmThresholds {
  float underSet;
  float underClear;
  float overSet;
  float overClear;
};

inline float adsLSB_mV(adsGain_t g) {
  switch (g) {
    case GAIN_TWOTHIRDS: return 0.1875f;
    case GAIN_ONE:       return 0.1250f;
    case GAIN_TWO:       return 0.0625f;
    case GAIN_FOUR:      return 0.03125f;
    case GAIN_EIGHT:     return 0.015625f;
    case GAIN_SIXTEEN:   return 0.0078125f;
  }
  return 0.1250f;
}

inline float mapCurrentPctToMm(float pct, float fullScaleMm, float offsetMm) {
  float clamped = pct;
  if (clamped < 0.0f) clamped = 0.0f;
  if (clamped > 100.0f) clamped = 100.0f;
  return (clamped / 100.0f) * fullScaleMm + offsetMm;
}

inline AlarmState evalWithHyst(AlarmState cur, float mA,
                               const AlarmThresholds& th) {
  if (cur == ALARM_NORMAL) {
    if (mA < th.underSet) return ALARM_UNDER;
    if (mA > th.overSet)  return ALARM_OVER;
    return ALARM_NORMAL;
  }

  if (cur == ALARM_UNDER) {
    if (mA > th.overSet)    return ALARM_OVER;  // direct jump allowed
    if (mA > th.underClear) return ALARM_NORMAL;
    return ALARM_UNDER;
  }

  // cur == ALARM_OVER
  if (mA < th.underSet)  return ALARM_UNDER;    // direct jump allowed
  if (mA < th.overClear) return ALARM_NORMAL;
  return ALARM_OVER;
}

inline uint8_t gainToCode(adsGain_t g) {
  switch (g) {
    case GAIN_TWOTHIRDS: return 0;
    case GAIN_ONE:       return 1;
    case GAIN_TWO:       return 2;
    case GAIN_FOUR:      return 3;
    case GAIN_EIGHT:     return 4;
    case GAIN_SIXTEEN:   return 5;
  }
  return 1;
}

inline adsGain_t codeToGain(uint8_t code) {
  switch (code) {
    case 0: return GAIN_TWOTHIRDS;
    case 1: return GAIN_ONE;
    case 2: return GAIN_TWO;
    case 3: return GAIN_FOUR;
    case 4: return GAIN_EIGHT;
    case 5: return GAIN_SIXTEEN;
    default: return GAIN_ONE;
  }
}

}  // namespace logic

