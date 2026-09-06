#pragma once
#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <vector>
#include "i_wire.hh"

// Scripted "Wire" double for I2CAdapter tests.
//
// Emulates a device with 16-bit registers holding 16-bit big-endian words:
//   - a 2-byte transmission selects the register to read from next,
//   - a 4-byte transmission writes a word to a register,
//   - request_from() serves consecutive words from the selected register.
// Knobs (`end_transmission_status`, `fail_request`, `write_mask`) inject the
// failure modes the adapter is expected to translate into error codes.
class MockWire {
public:
    // --- scripting -----------------------------------------------------------
    std::map<uint16_t, uint16_t> registers;
    int end_transmission_status = 0;  // returned after a register-select (2-byte) transmission
    bool fail_request = false;        // request_from() returns 0 bytes
    uint16_t write_mask = 0xFFFF;     // bits the "device" actually stores on write

    // --- recording -----------------------------------------------------------
    bool begun = false;
    uint32_t clock_hz = 0;
    uint32_t delay_us_total = 0;
    std::vector<std::vector<uint8_t>> transactions;  // bytes of each begin/end_transmission pair
    std::vector<std::size_t> request_sizes;          // quantity of each request_from()

    void begin() { begun = true; }
    void set_clock(uint32_t freq_hz) { clock_hz = freq_hz; }

    void begin_transmission(uint8_t address) {
        in_transaction_ = true;
        last_address_ = address;
        tx_.clear();
    }

    int end_transmission(bool /*stop*/) {
        if (!in_transaction_) {
            return 0;
        }
        in_transaction_ = false;
        transactions.push_back(tx_);
        if (tx_.size() >= 2) {
            selected_register_ = static_cast<uint16_t>((tx_[0] << 8) | tx_[1]);
        }
        if (tx_.size() == 4) {
            registers[selected_register_] = static_cast<uint16_t>(((tx_[2] << 8) | tx_[3]) & write_mask);
            return 0;
        }
        return tx_.size() == 2 ? end_transmission_status : 0;
    }

    std::size_t request_from(uint8_t /*address*/, std::size_t quantity) {
        request_sizes.push_back(quantity);
        if (fail_request) {
            return 0;
        }
        for (std::size_t i = 0; i < quantity / 2; i++) {
            const auto it = registers.find(selected_register_);
            const uint16_t word = (it == registers.end()) ? 0 : it->second;
            rx_.push_back(static_cast<uint8_t>(word >> 8));
            rx_.push_back(static_cast<uint8_t>(word & 0xFF));
            selected_register_++;
        }
        return quantity;
    }

    std::size_t write(uint8_t data) {
        tx_.push_back(data);
        return 1;
    }

    std::size_t write(const uint8_t* data, std::size_t quantity) {
        tx_.insert(tx_.end(), data, data + quantity);
        return quantity;
    }

    int available() { return static_cast<int>(rx_.size()); }

    int read() {
        if (rx_.empty()) {
            return -1;
        }
        const int value = rx_.front();
        rx_.pop_front();
        return value;
    }

    void delay_microseconds(uint32_t us) { delay_us_total += us; }

private:
    bool in_transaction_ = false;
    uint8_t last_address_ = 0;
    uint16_t selected_register_ = 0;
    std::vector<uint8_t> tx_;
    std::deque<uint8_t> rx_;
};

static_assert(is_wire<MockWire>::value, "MockWire must satisfy the Wire shape");
