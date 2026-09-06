#include "arduino_wire.hh"
#include <Arduino.h>  // for ::delayMicroseconds, GPIO and the PIN_WIRE_* pins
#include <Wire.h>

namespace {

/// @brief Half period of the recovery clock, microseconds (~100 kHz).
constexpr uint32_t half_period_us = 5;

/// @brief Let a line float high through its pull-up (open-drain "release").
/// @param pin Arduino pin number.
void release(uint32_t pin) {
    pinMode(pin, INPUT_PULLUP);
}

/// @brief Drive a line low (open-drain "assert"); never drives high against a slave.
/// @param pin Arduino pin number.
void pull_low(uint32_t pin) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
}

/// @brief Sample the data line.
/// @return True if SDA reads high (bus idle or released).
bool sda_high() {
    return digitalRead(PIN_WIRE_SDA) == HIGH;
}

/// @brief Free a slave holding SDA low by clocking SCL, then reset it with START+STOP.
///
/// Runs before Wire.begin() takes the pins. Up to nine clocks let the slave
/// finish the byte it was shifting when the master disappeared.
/// @return What was found and whether the bus is now usable.
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
