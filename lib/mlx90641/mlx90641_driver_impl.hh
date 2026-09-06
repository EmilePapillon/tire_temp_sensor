#pragma once
// Template member definitions for MLX90641Sensor. Included by
// mlx90641_driver.hh; do not include directly.
#include <cmath>
#include <cstdio>
#include <cstring>

namespace mlx90641 {

template <typename I2CAdapterT, typename LoggerT>
MLX90641Sensor<I2CAdapterT, LoggerT>::MLX90641Sensor(I2CAdapterT& i2c_adapter, uint8_t i2c_addr)
    : i2c_(i2c_adapter), i2c_addr_(i2c_addr), ambient_(0.0f), logger_() {
    temps_.fill(0.0f);
    ee_data_.fill(0);
    frame_data_.fill(0);
    std::memset(&calibration_parameters_, 0, sizeof(calibration_parameters_));
}

template <typename I2CAdapterT, typename LoggerT>
bool MLX90641Sensor<I2CAdapterT, LoggerT>::init(const Mlx90641Config& config) {
    log(LogLevel::DEBUG, "Starting MLX90641 sensor initialization");

    log(LogLevel::DEBUG, "Initializing I2C adapter");
    if (i2c_.init(config.i2c_freq_khz) != 0) {
        log(LogLevel::ERROR, "Failed to initialize I2C adapter");
        return false;
    }
    log(LogLevel::DEBUG, "I2C adapter initialized successfully");

    log(LogLevel::DEBUG, "Dumping EEPROM data");
    const int ee_result = dump_ee();
    if (ee_result != 0) {
        char msg[64];
        snprintf(msg, sizeof(msg), "Failed to dump EEPROM data, error: %d", ee_result);
        log(LogLevel::ERROR, msg);
        return false;
    }
    log(LogLevel::DEBUG, "EEPROM data dumped successfully");

    log(LogLevel::DEBUG, "Extracting calibration parameters");
    const int param_result = extract_parameters();
    if (param_result != 0) {
        char msg[64];
        snprintf(msg, sizeof(msg), "Failed to extract parameters, error: %d", param_result);
        log(LogLevel::ERROR, msg);
        return false;
    }
    log(LogLevel::DEBUG, "Calibration parameters extracted successfully");

    const int res_result = set_resolution(config.resolution);
    if (res_result != 0) {
        char msg[64];
        snprintf(msg, sizeof(msg), "Failed to set resolution, error: %d", res_result);
        log(LogLevel::WARN, msg);
    } else {
        log(LogLevel::DEBUG, "Resolution set successfully");
    }

    const int rate_result = set_refresh_rate(config.refresh_rate);
    if (rate_result != 0) {
        char msg[64];
        snprintf(msg, sizeof(msg), "Failed to set refresh rate, error: %d", rate_result);
        log(LogLevel::WARN, msg);
    } else {
        log(LogLevel::DEBUG, "Refresh rate set successfully");
    }

    log(LogLevel::INFO, "MLX90641 sensor initialization completed successfully");
    return true;
}

template <typename I2CAdapterT, typename LoggerT>
bool MLX90641Sensor<I2CAdapterT, LoggerT>::read_frame() {
    if (get_frame_data() != 0) {
        return false;
    }
    ambient_ = get_ta();
    return true;
}

template <typename I2CAdapterT, typename LoggerT>
void MLX90641Sensor<I2CAdapterT, LoggerT>::calculate_temps() {
    const float emissivity = get_emissivity();
    const float tr = ambient_;
    calculate_to(emissivity, tr);
    bad_pixels_correction();
}

template <typename I2CAdapterT, typename LoggerT>
std::array<float, MLX90641Sensor<I2CAdapterT, LoggerT>::num_pixels>
MLX90641Sensor<I2CAdapterT, LoggerT>::get_temps() const {
    return temps_;
}

template <typename I2CAdapterT, typename LoggerT>
float MLX90641Sensor<I2CAdapterT, LoggerT>::get_ambient() const {
    return ambient_;
}

// ------------------- Private member functions -------------------

template <typename I2CAdapterT, typename LoggerT>
int MLX90641Sensor<I2CAdapterT, LoggerT>::dump_ee() {
    int error = i2c_.read(i2c_addr_, eeprom_start_address, ee_data_size, ee_data_.data());
    if (error == 0) {
        error = hamming_decode();
    }
    return error;
}

template <typename I2CAdapterT, typename LoggerT>
int MLX90641Sensor<I2CAdapterT, LoggerT>::hamming_decode() {
    int error = 0;
    int16_t parity[5];
    int8_t d[16];
    int16_t check;
    uint16_t data;
    uint16_t mask;

    for (int addr = 16; addr < 832; addr++) {
        parity[0] = -1;
        parity[1] = -1;
        parity[2] = -1;
        parity[3] = -1;
        parity[4] = -1;

        data = ee_data_[addr];
        mask = 1;
        for (int i = 0; i < 16; i++) {
            d[i] = (data & mask) >> i;
            mask = mask << 1;
        }

        parity[0] = d[0] ^ d[1] ^ d[3] ^ d[4] ^ d[6] ^ d[8] ^ d[10] ^ d[11];
        parity[1] = d[0] ^ d[2] ^ d[3] ^ d[5] ^ d[6] ^ d[9] ^ d[10] ^ d[12];
        parity[2] = d[1] ^ d[2] ^ d[3] ^ d[7] ^ d[8] ^ d[9] ^ d[10] ^ d[13];
        parity[3] = d[4] ^ d[5] ^ d[6] ^ d[7] ^ d[8] ^ d[9] ^ d[10] ^ d[14];
        parity[4] = d[0] ^ d[1] ^ d[2] ^ d[3] ^ d[4] ^ d[5] ^ d[6] ^ d[7] ^ d[8] ^ d[9] ^ d[10] ^ d[11] ^ d[12] ^
                    d[13] ^ d[14] ^ d[15];

        if ((parity[0] != 0) || (parity[1] != 0) || (parity[2] != 0) || (parity[3] != 0) || (parity[4] != 0)) {
            check = (parity[0] << 0) + (parity[1] << 1) + (parity[2] << 2) + (parity[3] << 3) + (parity[4] << 4);

            if ((check > 15) && (check < 32)) {
                switch (check) {
                    case 16: d[15] = 1 - d[15]; break;
                    case 24: d[14] = 1 - d[14]; break;
                    case 20: d[13] = 1 - d[13]; break;
                    case 18: d[12] = 1 - d[12]; break;
                    case 17: d[11] = 1 - d[11]; break;
                    case 31: d[10] = 1 - d[10]; break;
                    case 30: d[9] = 1 - d[9]; break;
                    case 29: d[8] = 1 - d[8]; break;
                    case 28: d[7] = 1 - d[7]; break;
                    case 27: d[6] = 1 - d[6]; break;
                    case 26: d[5] = 1 - d[5]; break;
                    case 25: d[4] = 1 - d[4]; break;
                    case 23: d[3] = 1 - d[3]; break;
                    case 22: d[2] = 1 - d[2]; break;
                    case 21: d[1] = 1 - d[1]; break;
                    case 19: d[0] = 1 - d[0]; break;
                }
                if (error == 0) {
                    error = -9;
                }
                data = 0;
                mask = 1;
                for (int i = 0; i < 16; i++) {
                    data = data + d[i] * mask;
                    mask = mask << 1;
                }
            } else {
                error = -10;
            }
        }
        ee_data_[addr] = data & 0x07FF;
    }
    return error;
}

template <typename I2CAdapterT, typename LoggerT>
int MLX90641Sensor<I2CAdapterT, LoggerT>::get_frame_data() {
    uint16_t data_ready = 0;
    uint16_t control_register_1_value = 0;
    uint16_t status_register_value = 0;
    int error = 1;
    uint8_t cnt = 0;
    uint8_t sub_page = 0;

    while (data_ready == 0) {
        error = i2c_.read(i2c_addr_, status_register, 1, &status_register_value);
        if (error != 0) {
            return error;
        }
        data_ready = status_register_value & 0x0008;
    }
    sub_page = status_register_value & 0x0001;

    while (data_ready != 0 && cnt < 5) {
        error = i2c_.write(i2c_addr_, status_register, 0x0030);
        if (error == -1) {
            return error;
        }
        if (sub_page == 0) {
            error = i2c_.read(i2c_addr_, 0x0400, 32, frame_data_.data());
            if (error != 0) { return error; }
            error = i2c_.read(i2c_addr_, 0x0440, 32, frame_data_.data() + 32);
            if (error != 0) { return error; }
            error = i2c_.read(i2c_addr_, 0x0480, 32, frame_data_.data() + 64);
            if (error != 0) { return error; }
            error = i2c_.read(i2c_addr_, 0x04C0, 32, frame_data_.data() + 96);
            if (error != 0) { return error; }
            error = i2c_.read(i2c_addr_, 0x0500, 32, frame_data_.data() + 128);
            if (error != 0) { return error; }
            error = i2c_.read(i2c_addr_, 0x0540, 32, frame_data_.data() + 160);
            if (error != 0) { return error; }
        } else {
            error = i2c_.read(i2c_addr_, 0x0420, 32, frame_data_.data());
            if (error != 0) { return error; }
            error = i2c_.read(i2c_addr_, 0x0460, 32, frame_data_.data() + 32);
            if (error != 0) { return error; }
            error = i2c_.read(i2c_addr_, 0x04A0, 32, frame_data_.data() + 64);
            if (error != 0) { return error; }
            error = i2c_.read(i2c_addr_, 0x04E0, 32, frame_data_.data() + 96);
            if (error != 0) { return error; }
            error = i2c_.read(i2c_addr_, 0x0520, 32, frame_data_.data() + 128);
            if (error != 0) { return error; }
            error = i2c_.read(i2c_addr_, 0x0560, 32, frame_data_.data() + 160);
            if (error != 0) { return error; }
        }
        error = i2c_.read(i2c_addr_, 0x0580, 48, frame_data_.data() + 192);
        if (error != 0) {
            return error;
        }
        error = i2c_.read(i2c_addr_, status_register, 1, &status_register_value);
        if (error != 0) {
            return error;
        }
        data_ready = status_register_value & 0x0008;
        sub_page = status_register_value & 0x0001;
        cnt = cnt + 1;
    }
    if (cnt > 4) {
        return -8;
    }
    error = i2c_.read(i2c_addr_, control_register_1, 1, &control_register_1_value);
    frame_data_[240] = control_register_1_value;
    frame_data_[241] = status_register_value & 0x0001;
    if (error != 0) {
        return error;
    }
    return frame_data_[241];
}

template <typename I2CAdapterT, typename LoggerT>
int MLX90641Sensor<I2CAdapterT, LoggerT>::extract_parameters() {
    const int error = check_eeprom_valid();
    bool extractions_successful = false;
    if (error == 0) {
        {
            char debug_msg[160];
            snprintf(debug_msg, sizeof(debug_msg),
                     "Raw EEPROM - [34]: 0x%04X, [52]: 0x%04X, [53]: 0x%04X, [54]: 0x%04X, [45]: 0x%04X, [256]: 0x%04X",
                     ee_data_[34],    // KsTa
                     ee_data_[52],    // ksTo scale
                     ee_data_[53],    // ksTo[0]
                     ee_data_[54],    // ksTo[1]
                     ee_data_[45],    // cpAlpha
                     ee_data_[256]);  // alpha[0]
            log(LogLevel::DEBUG, debug_msg);
        }

        extractions_successful = MLX90641EEpromParser(ee_data_).extract_all(calibration_parameters_);

        {
            char debug_msg[160];
            snprintf(debug_msg, sizeof(debug_msg),
                     "Critical params - ksTo[1]: %.6f, tgc: %.6f, cpAlpha: %.6f, alpha[0]: %.6f",
                     static_cast<double>(calibration_parameters_.ksTo[1]),
                     static_cast<double>(calibration_parameters_.tgc),
                     static_cast<double>(calibration_parameters_.cpAlpha),
                     static_cast<double>(calibration_parameters_.alpha[0]));
            log(LogLevel::DEBUG, debug_msg);
        }
    }

    const bool success = extractions_successful && (error == 0);
    return success ? 0 : -1;
}

template <typename I2CAdapterT, typename LoggerT>
int MLX90641Sensor<I2CAdapterT, LoggerT>::set_resolution(Resolution resolution) {
    uint16_t control_register_1_value;
    int value = (static_cast<uint8_t>(resolution) & 0x03) << 10;

    int error = i2c_.read(i2c_addr_, control_register_1, 1, &control_register_1_value);
    if (error != 0) {
        log(LogLevel::ERROR, "Failed to read control register for setting resolution");
    }
    if (error == 0) {
        value = (control_register_1_value & 0xF3FF) | value;
        error = i2c_.write(i2c_addr_, control_register_1, static_cast<uint16_t>(value));
    }
    if (error != 0) {
        log(LogLevel::ERROR, "Failed to write control register for setting resolution");
    }
    return error;
}

template <typename I2CAdapterT, typename LoggerT>
int MLX90641Sensor<I2CAdapterT, LoggerT>::get_cur_resolution() const {
    uint16_t control_register_1_value;
    const int error = i2c_.read(i2c_addr_, control_register_1, 1, &control_register_1_value);
    if (error != 0) {
        return error;
    }
    return (control_register_1_value & 0x0C00) >> 10;
}

template <typename I2CAdapterT, typename LoggerT>
int MLX90641Sensor<I2CAdapterT, LoggerT>::set_refresh_rate(RefreshRate refresh_rate) {
    uint16_t control_register_1_value;
    int value = (static_cast<uint8_t>(refresh_rate) & 0x07) << 7;

    int error = i2c_.read(i2c_addr_, control_register_1, 1, &control_register_1_value);
    if (error == 0) {
        value = (control_register_1_value & 0xFC7F) | value;
        error = i2c_.write(i2c_addr_, control_register_1, static_cast<uint16_t>(value));
    }
    return error;
}

template <typename I2CAdapterT, typename LoggerT>
int MLX90641Sensor<I2CAdapterT, LoggerT>::get_refresh_rate() const {
    uint16_t control_register_1_value;
    const int error = i2c_.read(i2c_addr_, control_register_1, 1, &control_register_1_value);
    if (error != 0) {
        return error;
    }
    return (control_register_1_value & 0x0380) >> 7;
}

template <typename I2CAdapterT, typename LoggerT>
void MLX90641Sensor<I2CAdapterT, LoggerT>::calculate_to(float emissivity, float tr) {
    float vdd;
    float ta;
    float ta4;
    float tr4;
    float ta_tr;
    float gain;
    float ir_data_cp;
    float ir_data;
    float alpha_compensated;
    float sx;
    float to;
    float alpha_corr_r[8];
    int8_t range;
    uint16_t sub_page;

    const ParamsMLX90641& p = calibration_parameters_;

    sub_page = frame_data_[241];
    vdd = get_vdd();
    ta = get_ta();
    ta4 = pow((ta + 273.15), (double)4);
    tr4 = pow((tr + 273.15), (double)4);
    ta_tr = tr4 - (tr4 - ta4) / emissivity;

    alpha_corr_r[1] = 1 / (1 + p.ksTo[1] * 20);
    alpha_corr_r[0] = alpha_corr_r[1] / (1 + p.ksTo[0] * 20);
    alpha_corr_r[2] = 1;
    alpha_corr_r[3] = (1 + p.ksTo[2] * p.ct[2]);
    alpha_corr_r[4] = alpha_corr_r[3] * (1 + p.ksTo[3] * (p.ct[4] - p.ct[3]));
    alpha_corr_r[5] = alpha_corr_r[4] * (1 + p.ksTo[4] * (p.ct[5] - p.ct[4]));
    alpha_corr_r[6] = alpha_corr_r[5] * (1 + p.ksTo[5] * (p.ct[6] - p.ct[5]));
    alpha_corr_r[7] = alpha_corr_r[6] * (1 + p.ksTo[6] * (p.ct[7] - p.ct[6]));

    //------------------------- Gain calculation -----------------------------------
    gain = frame_data_[202];
    if (gain > 32767) {
        gain = gain - 65536;
    }
    gain = p.gainEE / gain;

    //------------------------- To calculation -------------------------------------
    ir_data_cp = frame_data_[200];
    if (ir_data_cp > 32767) {
        ir_data_cp = ir_data_cp - 65536;
    }
    ir_data_cp = ir_data_cp * gain;
    ir_data_cp = ir_data_cp - p.cpOffset * (1 + p.cpKta * (ta - 25)) * (1 + p.cpKv * (vdd - 3.3));

    for (int pixel_number = 0; pixel_number < 192; pixel_number++) {
        ir_data = frame_data_[pixel_number];
        if (ir_data > 32767) {
            ir_data = ir_data - 65536;
        }
        ir_data = ir_data * gain;

        ir_data = ir_data - p.offset[sub_page][pixel_number] * (1 + p.kta[pixel_number] * (ta - 25)) *
                                (1 + p.kv[pixel_number] * (vdd - 3.3));

        ir_data = ir_data - p.tgc * ir_data_cp;

        ir_data = ir_data / emissivity;

        alpha_compensated = (p.alpha[pixel_number] - p.tgc * p.cpAlpha) * (1 + p.KsTa * (ta - 25));

        sx = alpha_compensated * alpha_compensated * alpha_compensated * (ir_data + alpha_compensated * ta_tr);
        sx = sqrt(sqrt(sx)) * p.ksTo[1];

        to = sqrt(sqrt(ir_data / (alpha_compensated * (1 - p.ksTo[1] * 273.15) + sx) + ta_tr)) - 273.15;

        if (to < p.ct[1]) {
            range = 0;
        } else if (to < p.ct[2]) {
            range = 1;
        } else if (to < p.ct[3]) {
            range = 2;
        } else if (to < p.ct[4]) {
            range = 3;
        } else if (to < p.ct[5]) {
            range = 4;
        } else if (to < p.ct[6]) {
            range = 5;
        } else if (to < p.ct[7]) {
            range = 6;
        } else {
            range = 7;
        }

        to = sqrt(sqrt(ir_data / (alpha_compensated * alpha_corr_r[range] * (1 + p.ksTo[range] * (to - p.ct[range]))) +
                       ta_tr)) -
             273.15;
        temps_[pixel_number] = to;
    }
}

template <typename I2CAdapterT, typename LoggerT>
void MLX90641Sensor<I2CAdapterT, LoggerT>::get_image() {
    float vdd;
    float ta;
    float gain;
    float ir_data_cp;
    float ir_data;
    float alpha_compensated;
    float image;
    uint16_t sub_page;

    const ParamsMLX90641& p = calibration_parameters_;

    sub_page = frame_data_[241];
    vdd = get_vdd();
    ta = get_ta();

    //------------------------- Gain calculation -----------------------------------
    gain = frame_data_[202];
    if (gain > 32767) {
        gain = gain - 65536;
    }
    gain = p.gainEE / gain;

    //------------------------- Image calculation -------------------------------------
    ir_data_cp = frame_data_[200];
    if (ir_data_cp > 32767) {
        ir_data_cp = ir_data_cp - 65536;
    }
    ir_data_cp = ir_data_cp * gain;
    ir_data_cp = ir_data_cp - p.cpOffset * (1 + p.cpKta * (ta - 25)) * (1 + p.cpKv * (vdd - 3.3));

    for (int pixel_number = 0; pixel_number < 192; pixel_number++) {
        ir_data = frame_data_[pixel_number];
        if (ir_data > 32767) {
            ir_data = ir_data - 65536;
        }
        ir_data = ir_data * gain;

        ir_data = ir_data - p.offset[sub_page][pixel_number] * (1 + p.kta[pixel_number] * (ta - 25)) *
                                (1 + p.kv[pixel_number] * (vdd - 3.3));

        ir_data = ir_data - p.tgc * ir_data_cp;

        alpha_compensated = (p.alpha[pixel_number] - p.tgc * p.cpAlpha);

        image = ir_data / alpha_compensated;

        temps_[pixel_number] = image;
    }
}

template <typename I2CAdapterT, typename LoggerT>
float MLX90641Sensor<I2CAdapterT, LoggerT>::get_vdd() const {
    float vdd = frame_data_[234];
    if (vdd > 32767) {
        vdd = vdd - 65536;
    }
    const int resolution_ram = (frame_data_[240] & 0x0C00) >> 10;
    const float resolution_correction =
        pow(2, (double)calibration_parameters_.resolutionEE) / pow(2, (double)resolution_ram);
    vdd = (resolution_correction * vdd - calibration_parameters_.vdd25) / calibration_parameters_.kVdd + 3.3;
    return vdd;
}

template <typename I2CAdapterT, typename LoggerT>
float MLX90641Sensor<I2CAdapterT, LoggerT>::get_ta() const {
    const float vdd = get_vdd();

    float ptat = frame_data_[224];
    if (ptat > 32767) {
        ptat = ptat - 65536;
    }

    float ptat_art = frame_data_[192];
    if (ptat_art > 32767) {
        ptat_art = ptat_art - 65536;
    }
    ptat_art = (ptat / (ptat * calibration_parameters_.alphaPTAT + ptat_art)) * pow(2, (double)18);

    float ta = (ptat_art / (1 + calibration_parameters_.KvPTAT * (vdd - 3.3)) - calibration_parameters_.vPTAT25);
    ta = ta / calibration_parameters_.KtPTAT + 25;
    return ta;
}

template <typename I2CAdapterT, typename LoggerT>
int MLX90641Sensor<I2CAdapterT, LoggerT>::get_sub_page_number() const {
    return frame_data_[241];
}

template <typename I2CAdapterT, typename LoggerT>
void MLX90641Sensor<I2CAdapterT, LoggerT>::bad_pixels_correction() {
    const auto& broken = calibration_parameters_.brokenPixels;
    float ap[2];
    uint8_t pix = 0;

    while (pix < broken.size() && broken[pix] < 65535) {
        const uint16_t index = broken[pix];
        const uint8_t line = index >> 5;
        const uint8_t column = index - (line << 5);

        if (column == 0) {
            temps_[index] = temps_[index + 1];
        } else if (column == 1 || column == 14) {
            temps_[index] = (temps_[index - 1] + temps_[index + 1]) / 2.0;
        } else if (column == 15) {
            temps_[index] = temps_[index - 1];
        } else {
            ap[0] = temps_[index + 1] - temps_[index + 2];
            ap[1] = temps_[index - 1] - temps_[index - 2];
            if (fabs(ap[0]) > fabs(ap[1])) {
                temps_[index] = temps_[index - 1] + ap[1];
            } else {
                temps_[index] = temps_[index + 1] + ap[0];
            }
        }

        pix = pix + 1;
    }
}

template <typename I2CAdapterT, typename LoggerT>
float MLX90641Sensor<I2CAdapterT, LoggerT>::get_emissivity() const {
    return calibration_parameters_.emissivityEE;
}

template <typename I2CAdapterT, typename LoggerT>
int MLX90641Sensor<I2CAdapterT, LoggerT>::check_eeprom_valid() const {
    const int device_select = ee_data_[10] & 0x0040;
    if (device_select != 0) {
        return 0;
    }
    return -7;
}

template <typename I2CAdapterT, typename LoggerT>
void MLX90641Sensor<I2CAdapterT, LoggerT>::log(LogLevel level, const char* message) {
    logger_.log(level, message);
}

}  // namespace mlx90641
