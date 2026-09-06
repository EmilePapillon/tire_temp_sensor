#include "arduino_wire.hh"
#include <Arduino.h>  // for ::delayMicroseconds, GPIO and the PIN_WIRE_* pins
#include <Wire.h>

namespace {

constexpr uint32_t half_period_us = 5;  // ~100 kHz clock for the recovery pulses

// Open-drain emulation: never drive a line high against a slave pulling it low.
void release(uint32_t pin) {
    pinMode(pin, INPUT_PULLUP);
}

void pull_low(uint32_t pin) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
}

bool sda_high() {
    return digitalRead(PIN_WIRE_SDA) == HIGH;
}

ArduinoWire::BusRecovery recover_bus() {
    release(PIN_WIRE_SDA);
    release(PIN_WIRE_SCL);
    ::delayMicroseconds(half_period_us);
    if (sda_high()) {
        return ArduinoWire::BusRecovery::NotNeeded;
    }
    for (int pulse = 0; pulse < 9 && !sda_high(); pulse++) {
        pull_low(PIN_WIRE_SCL);
        ::delayMicroseconds(half_period_us);
        release(PIN_WIRE_SCL);
        ::delayMicroseconds(half_period_us);
    }
    if (!sda_high()) {
        return ArduinoWire::BusRecovery::Failed;
    }
    // START then STOP with SCL high, which resets the slave's bus state machine.
    pull_low(PIN_WIRE_SDA);
    ::delayMicroseconds(half_period_us);
    release(PIN_WIRE_SDA);
    ::delayMicroseconds(half_period_us);
    return ArduinoWire::BusRecovery::Recovered;
}

}  // namespace

void ArduinoWire::begin() {
    recovery_ = recover_bus();
    ::Wire.begin();
}

ArduinoWire::BusRecovery ArduinoWire::recovery() const {
    return recovery_;
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
