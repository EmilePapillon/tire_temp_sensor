#include "arduino_wire.hh"
#include <Arduino.h>  // for ::delayMicroseconds
#include <Wire.h>

void ArduinoWire::begin() {
    ::Wire.begin();
}

void ArduinoWire::set_clock(uint32_t freq_hz) {
    ::Wire.setClock(freq_hz);
}

void ArduinoWire::begin_transmission(uint8_t address) {
    ::Wire.beginTransmission(address);
}

int ArduinoWire::end_transmission(bool stop) {
    return ::Wire.endTransmission(stop);
}

std::size_t ArduinoWire::request_from(uint8_t address, std::size_t quantity) {
    return ::Wire.requestFrom(address, quantity);
}

std::size_t ArduinoWire::write(uint8_t data) {
    return ::Wire.write(data);
}

std::size_t ArduinoWire::write(const uint8_t* data, std::size_t quantity) {
    return ::Wire.write(data, quantity);
}

int ArduinoWire::available() {
    return ::Wire.available();
}

int ArduinoWire::read() {
    return ::Wire.read();
}

void ArduinoWire::delay_microseconds(uint32_t us) {
    ::delayMicroseconds(us);
}
