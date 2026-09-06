#pragma once
#include <cstdint>

// RejsaRubberTrac BLE protocol (natively supported by RaceChrono and Harry's
// LapTimer). Service 0x1ff7 exposes three NOTIFY characteristics, each carrying
// a fixed 20-byte packet:
//   0x01 -> DataPackOne : even-numbered temp spots + suspension distance
//   0x02 -> DataPackTwo : odd-numbered temp spots + battery state
//   0x03 -> DataPackThr : per-pair max of all 16 spots + suspension distance
// All temps are degrees Celsius x 10. Fields are little-endian (native nRF52).
constexpr uint8_t rejsa_protocol = 0x02;

struct DataPackOne {
  uint8_t protocol;   // protocol version (rejsa_protocol)
  uint8_t unused;
  int16_t distance;   // millimeters (0 when no distance sensor fitted)
  int16_t temps[8];   // even-numbered temp spots (columns 0, 2, 4, ... 14)
} __attribute__((packed));

struct DataPackTwo {
  uint8_t  protocol;  // protocol version (rejsa_protocol)
  uint8_t  charge;    // battery percent 0-100 (0 when not measured)
  uint16_t voltage;   // millivolts (0 when not measured)
  int16_t  temps[8];  // odd-numbered temp spots (columns 1, 3, 5, ... 15)
} __attribute__((packed));

struct DataPackThr {
  uint8_t protocol;   // protocol version (rejsa_protocol)
  uint8_t unused;
  int16_t distance;   // millimeters (0 when no distance sensor fitted)
  int16_t temps[8];   // max of each adjacent column pair
} __attribute__((packed));
