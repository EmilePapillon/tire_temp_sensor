#pragma once
#include <cstdint>

struct DataPack {
  uint8_t  protocol{};         // version of protocol used
  uint8_t  unused_one{};
  int16_t  unused_two{};       
  int16_t  temps[6]{};         // all even numbered temp spots (degrees Celsius x 10)
} __attribute__((packed)) ;

struct BLEDataPacks {
  DataPack one;
  DataPack two;
  DataPack three;
} __attribute__((packed)) ;