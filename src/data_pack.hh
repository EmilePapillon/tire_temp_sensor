#pragma once
#include <cstdint>

struct DataPack {
  uint8_t  protocol;       // version of protocol
  uint8_t  packet_id;      // 0..3 → which quarter of the data this is
  uint8_t reserved;       // future use or alignment
  int16_t  temps[8];       // 4 averaged temperatures (°C × 10)
} __attribute__((packed));
