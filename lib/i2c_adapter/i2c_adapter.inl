// Inline and template definitions for i2c_adapter.hh. Included by the header; do not include directly.
#pragma once

inline const char* i2c_status_name(I2cStatus status) {
    switch (status) {
        case I2cStatus::Success:        return "success";
        case I2cStatus::Nack:           return "nack";
        case I2cStatus::BusError:       return "bus error";
        case I2cStatus::NoData:         return "no data";
        case I2cStatus::VerifyMismatch: return "verify mismatch";
    }
    return "?";
}

template <typename WireT>
I2CAdapter<WireT>::I2CAdapter(WireT& wire) : wire_(wire) {}

template <typename WireT>
I2cStatus I2CAdapter<WireT>::init(uint32_t freq_khz) {
    wire_.begin();
    set_frequency(freq_khz);
    wire_.delay_microseconds(1000);
    return I2cStatus::Success;
}

template <typename WireT>
void I2CAdapter<WireT>::set_frequency(uint32_t freq_khz) {
    wire_.set_clock(1000u * freq_khz);
}

template <typename WireT>
I2cStatus I2CAdapter<WireT>::read(uint8_t device_address, uint16_t start_register, std::size_t length,
                                  uint16_t* buffer) {
    const std::size_t total_bytes = 2 * length;
    const std::size_t remainder = total_bytes % max_chunk_bytes;
    std::size_t chunks = total_bytes / max_chunk_bytes;
    if (remainder != 0) {
        chunks++;
    }

    uint16_t* out = buffer;
    for (std::size_t chunk = 0; chunk < chunks; chunk++) {
        const uint16_t address = static_cast<uint16_t>(start_register + (chunk * max_chunk_bytes) / 2);

        wire_.end_transmission(true);
        wire_.delay_microseconds(5);
        wire_.begin_transmission(device_address);
        wire_.write(static_cast<uint8_t>(address >> 8));
        wire_.write(static_cast<uint8_t>(address & 0x00FF));

        const int status = wire_.end_transmission(false);  // Arduino TwoWire status code
        if (status == 2 || status == 3) {
            return I2cStatus::Nack;  // address / data NACK
        }
        if (status != 0) {
            return I2cStatus::BusError;  // 1 = tx overflow, 4 = other, 5 = timeout
        }

        std::size_t num_bytes = max_chunk_bytes;
        if (chunk == chunks - 1 && remainder != 0) {
            num_bytes = remainder;
        }
        num_bytes = wire_.request_from(device_address, num_bytes);
        if (num_bytes == 0) {
            return I2cStatus::NoData;
        }

        for (std::size_t i = 0; i < num_bytes / 2; i++) {
            if (wire_.available()) {
                const uint16_t high = static_cast<uint16_t>(wire_.read() << 8);
                const uint16_t low = static_cast<uint16_t>(wire_.read());
                *out++ = static_cast<uint16_t>(high | low);
            }
        }
    }
    return I2cStatus::Success;
}

template <typename WireT>
I2cStatus I2CAdapter<WireT>::write(uint8_t device_address, uint16_t reg, uint16_t value) {
    const uint8_t cmd[4] = {
        static_cast<uint8_t>(reg >> 8),
        static_cast<uint8_t>(reg & 0x00FF),
        static_cast<uint8_t>(value >> 8),
        static_cast<uint8_t>(value & 0x00FF),
    };

    wire_.end_transmission(true);
    wire_.begin_transmission(device_address);
    wire_.delay_microseconds(5);
    wire_.write(cmd, sizeof(cmd));
    wire_.end_transmission(true);

    uint16_t read_back = 0;
    const I2cStatus read_status = read(device_address, reg, 1, &read_back);
    if (read_status != I2cStatus::Success) {
        return read_status;
    }
    if (read_back != value) {
        return I2cStatus::VerifyMismatch;
    }
    return I2cStatus::Success;
}
