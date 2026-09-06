// Inline definitions for mock_wire.hh. Included by the header; do not include directly.
#pragma once
#include <algorithm>

inline void MockWire::begin() { begun = true; }

inline void MockWire::set_clock(uint32_t freq_hz) { clock_hz = freq_hz; }

inline void MockWire::begin_transmission(uint8_t address) {
    in_transaction_ = true;
    last_address_ = address;
    tx_.clear();
}

inline int MockWire::end_transmission(bool /*stop*/) {
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
        return write_end_transmission_status;
    }
    return tx_.size() == 2 ? end_transmission_status : 0;
}

inline std::size_t MockWire::request_from(uint8_t /*address*/, std::size_t quantity) {
    request_sizes.push_back(quantity);
    if (fail_request) {
        return 0;
    }
    const std::size_t response_bytes =
        (short_response_bytes == 0) ? quantity : std::min(short_response_bytes, quantity);
    for (std::size_t i = 0; i < response_bytes / 2; i++) {
        const auto it = registers.find(selected_register_);
        const uint16_t word = (it == registers.end()) ? 0 : it->second;
        rx_.push_back(static_cast<uint8_t>(word >> 8));
        rx_.push_back(static_cast<uint8_t>(word & 0xFF));
        selected_register_++;
    }
    return response_bytes;
}

inline std::size_t MockWire::write(uint8_t data) {
    tx_.push_back(data);
    return 1;
}

inline std::size_t MockWire::write(const uint8_t* data, std::size_t quantity) {
    tx_.insert(tx_.end(), data, data + quantity);
    return quantity;
}

inline int MockWire::available() { return static_cast<int>(rx_.size()); }

inline int MockWire::read() {
    if (rx_.empty()) {
        return -1;
    }
    const int value = rx_.front();
    rx_.pop_front();
    return value;
}

inline void MockWire::delay_microseconds(uint32_t us) { delay_us_total += us; }
