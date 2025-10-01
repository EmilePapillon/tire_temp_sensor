#include "i2c_adapter.hh"

bool I2CAdapter::init() {
    wire_.begin();
    return true;
}

void I2CAdapter::set_frequency(int freq) {
    wire_.setClock(1000 * freq); // freq in kHz
} 

bool I2CAdapter::read(uint8_t device_address,
                      uint16_t start_register,
                      std::size_t length,
                      uint16_t* buffer) {
    if (length == 0) {
        return true; // Nothing to read
    }

    char cmd[2] = { static_cast<char>(start_register >> 8),
                    static_cast<char>(start_register & 0x00FF) };

    wire_.endTransmission();
    wire_.delayMicroseconds(5);
    wire_.beginTransmission(device_address);

    int written1 = wire_.write(cmd[0]);
    int written2 = wire_.write(cmd[1]);

    int end = wire_.endTransmission(false);
    if (end == 2 || end == 3) return false;
    if (end == 1 || end == 4) return false;

    int total_bytes = 2 * length;
    const int div = 128; // Read in chunks of 128 bytes (64 words)
    int factor = total_bytes / div;
    int remainder = total_bytes % div;
    if (remainder != 0)
    {
        factor++;
    } 
    
    for (int j = 0; j < factor; j++) {
        int num_bytes = div;
        if (j == (factor - 1) && remainder != 0) num_bytes = remainder;

        num_bytes = wire_.requestFrom((int)device_address, num_bytes); // update number of bytes that were read
        if (!num_bytes) return false;

        for (int i = 0; i < num_bytes / 2; i++) {
            if (wire_.available()) {
                uint16_t high = wire_.read() << 8;
                uint16_t low = wire_.read();
                *buffer++ = high + low;
            }
        }
    }
    return true;
}

bool I2CAdapter::write(uint8_t device_address,
                       uint16_t reg,
                       uint16_t value) {
    char cmd[4] = { static_cast<char>(reg >> 8),
                    static_cast<char>(reg & 0x00FF),
                    static_cast<char>(value >> 8),
                    static_cast<char>(value & 0x00FF) };

    wire_.endTransmission();
    wire_.delayMicroseconds(5);
    wire_.beginTransmission(device_address);

    int written1 = wire_.write(cmd[0]);
    int written2 = wire_.write(cmd[1]);
    int written3 = wire_.write(cmd[2]);
    int written4 = wire_.write(cmd[3]);

    int end = wire_.endTransmission();
    if (end == 2 || end == 3) return false;
    if (end == 1 || end == 4) return false;

    // Validation: read back the register and compare
    uint16_t check_buf[1];
    if (!read(device_address, reg, 1, check_buf))
    {
        return false;
    }
    if (check_buf[0] != value)
    {
        return false;
    }

    return true;
}