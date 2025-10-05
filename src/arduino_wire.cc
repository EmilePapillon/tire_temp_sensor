#include "arduino_wire.hh"
#include <Wire.h>
#include <Arduino.h> // for ::delayMicroseconds

void Wire::begin() {
    ::Wire.begin();
}

void Wire::setClock(uint32_t freq) {
    ::Wire.setClock(freq);
}

int Wire::endTransmission(bool stop) {
    return ::Wire.endTransmission(stop);
}

void Wire::beginTransmission(uint8_t address) {
    ::Wire.beginTransmission(address);
}

uint8_t Wire::requestFrom(uint8_t address, std::size_t quantity) {
    return ::Wire.requestFrom(address, quantity);
}

std::size_t Wire::write(uint8_t data) {
    return ::Wire.write(data);
}

std::size_t Wire::write(const uint8_t* data, std::size_t quantity) {
    return ::Wire.write(data, quantity);
}

int Wire::available() {
    return ::Wire.available();
}

int Wire::read() {
    return ::Wire.read();
}

int Wire::peek() {
    return ::Wire.peek();
}

void Wire::flush() {
    ::Wire.flush();
}

void Wire::delayMicroseconds(int us) {
    ::delayMicroseconds(us);
}