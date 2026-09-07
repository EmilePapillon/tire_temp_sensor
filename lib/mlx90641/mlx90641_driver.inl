// Inline and template definitions for mlx90641_driver.hh. Included by the header; do not include directly.
#pragma once
#include <cmath>
#include <cstdio>
#include <cstring>

namespace mlx90641 {

inline const char* status_name(Status status) {
    switch (status) {
        case Status::Success:                     return "success";
        case Status::I2cNack:                     return "i2c nack";
        case Status::I2cBusError:                 return "i2c bus error";
        case Status::I2cNoData:                   return "i2c no data";
        case Status::I2cVerifyMismatch:           return "i2c verify mismatch";
        case Status::NotAnMlx90641:               return "not an MLX90641";
        case Status::EepromCorrupt:               return "eeprom corrupt";
        case Status::CalibrationExtractionFailed: return "calibration extraction failed";
        case Status::DataReadyTimeout:            return "data ready timeout";
        case Status::FrameSyncFailed:             return "frame sync failed";
    }
    return "?";
}

inline Status from_i2c(I2cStatus status) {
    switch (status) {
        case I2cStatus::Success:        return Status::Success;
        case I2cStatus::Nack:           return Status::I2cNack;
        case I2cStatus::BusError:       return Status::I2cBusError;
        case I2cStatus::NoData:         return Status::I2cNoData;
        case I2cStatus::VerifyMismatch: return Status::I2cVerifyMismatch;
    }
    return Status::I2cBusError;
}

inline std::array<float, sensor_columns> column_averages(const std::array<float, num_pixels>& temps) {
    std::array<float, sensor_columns> averages{};
    for (std::size_t col = 0; col < sensor_columns; col++) {
        float sum = 0.0f;
        for (std::size_t row = 0; row < sensor_rows; row++) {
            sum += temps[row * sensor_columns + col];
        }
        averages[col] = sum / static_cast<float>(sensor_rows);
    }
    return averages;
}

template <typename I2CAdapterT, typename LoggerT>
MLX90641Sensor<I2CAdapterT, LoggerT>::MLX90641Sensor(I2CAdapterT& i2c_adapter, uint8_t i2c_addr)
    : i2c_(i2c_adapter), i2c_addr_(i2c_addr), data_ready_max_polls_(0), ambient_(0.0f), logger_() {
    temps_.fill(0.0f);
    ee_data_.fill(0);
    frame_data_.fill(0);
    std::memset(&calibration_parameters_, 0, sizeof(calibration_parameters_));
}

template <typename I2CAdapterT, typename LoggerT>
Status MLX90641Sensor<I2CAdapterT, LoggerT>::init(const Mlx90641Config& config) {
    data_ready_max_polls_ = config.data_ready_max_polls;
    log(LogLevel::DEBUG, "Starting MLX90641 sensor initialization");

    log(LogLevel::DEBUG, "Initializing I2C adapter");
    const I2cStatus bus_status = i2c_.init(config.i2c_freq_khz);
    if (bus_status != I2cStatus::Success) {
        log(LogLevel::ERROR, "Failed to initialize I2C adapter");
        return from_i2c(bus_status);
    }

    log(LogLevel::DEBUG, "Dumping EEPROM data");
    const Status ee_status = dump_ee();
    if (ee_status != Status::Success) {
        char msg[64];
        snprintf(msg, sizeof(msg), "Failed to dump EEPROM data: %s", status_name(ee_status));
        log(LogLevel::ERROR, msg);
        return ee_status;
    }

    log(LogLevel::DEBUG, "Extracting calibration parameters");
    const Status param_status = extract_parameters();
    if (param_status != Status::Success) {
        char msg[64];
        snprintf(msg, sizeof(msg), "Failed to extract parameters: %s", status_name(param_status));
        log(LogLevel::ERROR, msg);
        return param_status;
    }

    const Status res_status = set_resolution(config.resolution);
    if (res_status != Status::Success) {
        char msg[64];
        snprintf(msg, sizeof(msg), "Failed to set resolution: %s", status_name(res_status));
        log(LogLevel::WARN, msg);
    }

    const Status rate_status = set_refresh_rate(config.refresh_rate);
    if (rate_status != Status::Success) {
        char msg[64];
        snprintf(msg, sizeof(msg), "Failed to set refresh rate: %s", status_name(rate_status));
        log(LogLevel::WARN, msg);
    }

    log(LogLevel::INFO, "MLX90641 sensor initialization completed successfully");
    return Status::Success;
}

template <typename I2CAdapterT, typename LoggerT>
Status MLX90641Sensor<I2CAdapterT, LoggerT>::read_frame() {
    const Status status = get_frame_data();
    if (status != Status::Success) {
        return status;
    }
    ambient_ = get_ta();
    return Status::Success;
}

template <typename I2CAdapterT, typename LoggerT>
void MLX90641Sensor<I2CAdapterT, LoggerT>::calculate_temps() {
    calculate_to(get_emissivity(), ambient_);
}

template <typename I2CAdapterT, typename LoggerT>
void MLX90641Sensor<I2CAdapterT, LoggerT>::calculate_temps(float emissivity) {
    calculate_temps(emissivity, ambient_);
}

template <typename I2CAdapterT, typename LoggerT>
void MLX90641Sensor<I2CAdapterT, LoggerT>::calculate_temps(float emissivity, float reflected_temperature_c) {
    calculate_to(emissivity, reflected_temperature_c);
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
float MLX90641Sensor<I2CAdapterT, LoggerT>::signed_word(uint16_t raw) {
    return static_cast<float>(static_cast<int16_t>(raw));
}

template <typename I2CAdapterT, typename LoggerT>
Status MLX90641Sensor<I2CAdapterT, LoggerT>::dump_ee() {
    const I2cStatus status = i2c_.read(i2c_addr_, eeprom_start_address, ee_data_size, ee_data_.data());
    if (status != I2cStatus::Success) {
        return from_i2c(status);
    }
    switch (hamming_decode()) {
        case HammingResult::Clean:
            return Status::Success;
        case HammingResult::Corrected:
            // The Melexis driver reports this as a warning; the corrected data is valid.
            log(LogLevel::WARN, "EEPROM had single-bit errors that were corrected");
            return Status::Success;
        case HammingResult::Uncorrectable:
            break;
    }
    return Status::EepromCorrupt;
}

template <typename I2CAdapterT, typename LoggerT>
typename MLX90641Sensor<I2CAdapterT, LoggerT>::HammingResult MLX90641Sensor<I2CAdapterT, LoggerT>::hamming_decode() {
    HammingResult result = HammingResult::Clean;
    int16_t parity[5];
    int8_t d[16];
    int16_t check;
    uint16_t data;
    uint16_t mask;

    for (int addr = 16; addr < 832; addr++) {
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
                if (result == HammingResult::Clean) {
                    result = HammingResult::Corrected;
                }
                data = 0;
                mask = 1;
                for (int i = 0; i < 16; i++) {
                    data = data + d[i] * mask;
                    mask = mask << 1;
                }
            } else {
                result = HammingResult::Uncorrectable;
            }
        }
        ee_data_[addr] = data & 0x07FF;
    }
    return result;
}

template <typename I2CAdapterT, typename LoggerT>
Status MLX90641Sensor<I2CAdapterT, LoggerT>::get_frame_data() {
    uint16_t control_register_1_value = 0;
    uint16_t status_register_value = 0;
    uint16_t data_ready = 0;
    uint8_t sub_page = 0;
    uint8_t attempts = 0;
    I2cStatus i2c_status;

    // Wait for the sensor to flag a new frame, bounded so a dead sensor cannot hang the caller.
    for (uint32_t polls = 0; data_ready == 0; polls++) {
        if (polls >= data_ready_max_polls_) {
            return Status::DataReadyTimeout;
        }
        i2c_status = i2c_.read(i2c_addr_, status_register, 1, &status_register_value);
        if (i2c_status != I2cStatus::Success) {
            return from_i2c(i2c_status);
        }
        data_ready = status_register_value & status_new_data_mask;
    }
    sub_page = status_register_value & status_sub_page_mask;

    // Acknowledge and read until the new-data flag stays clear, i.e. the frame
    // did not roll over underneath us.
    while (data_ready != 0 && attempts < frame_sync_max_attempts) {
        // The status register is self-clearing, so a read-back mismatch is expected here.
        i2c_status = i2c_.write(i2c_addr_, status_register, status_clear_new_data);
        if (i2c_status != I2cStatus::Success && i2c_status != I2cStatus::VerifyMismatch) {
            return from_i2c(i2c_status);
        }

        const uint16_t bank_base = (sub_page == 0) ? 0x0400 : 0x0420;
        for (uint16_t bank = 0; bank < 6; bank++) {
            i2c_status = i2c_.read(i2c_addr_, static_cast<uint16_t>(bank_base + bank * 0x40), 32,
                                   frame_data_.data() + bank * 32);
            if (i2c_status != I2cStatus::Success) {
                return from_i2c(i2c_status);
            }
        }
        i2c_status = i2c_.read(i2c_addr_, 0x0580, 48, frame_data_.data() + 192);
        if (i2c_status != I2cStatus::Success) {
            return from_i2c(i2c_status);
        }

        i2c_status = i2c_.read(i2c_addr_, status_register, 1, &status_register_value);
        if (i2c_status != I2cStatus::Success) {
            return from_i2c(i2c_status);
        }
        data_ready = status_register_value & status_new_data_mask;
        sub_page = status_register_value & status_sub_page_mask;
        attempts++;
    }
    if (data_ready != 0) {
        return Status::FrameSyncFailed;
    }

    i2c_status = i2c_.read(i2c_addr_, control_register_1, 1, &control_register_1_value);
    if (i2c_status != I2cStatus::Success) {
        return from_i2c(i2c_status);
    }
    frame_data_[240] = control_register_1_value;
    frame_data_[241] = status_register_value & status_sub_page_mask;
    // Both sub-pages are complete frames on the MLX90641; they differ only in
    // which offset calibration set applies (see calculate_to).
    return Status::Success;
}

template <typename I2CAdapterT, typename LoggerT>
Status MLX90641Sensor<I2CAdapterT, LoggerT>::extract_parameters() {
    const Status valid = check_eeprom_valid();
    if (valid != Status::Success) {
        return valid;
    }

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

    if (!MLX90641EEpromParser(ee_data_).extract_all(calibration_parameters_)) {
        return Status::CalibrationExtractionFailed;
    }

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
    return Status::Success;
}

template <typename I2CAdapterT, typename LoggerT>
Status MLX90641Sensor<I2CAdapterT, LoggerT>::set_resolution(Resolution resolution) {
    uint16_t control_register_1_value = 0;
    I2cStatus status = i2c_.read(i2c_addr_, control_register_1, 1, &control_register_1_value);
    if (status != I2cStatus::Success) {
        log(LogLevel::ERROR, "Failed to read control register for setting resolution");
        return from_i2c(status);
    }
    const uint16_t field = static_cast<uint16_t>((static_cast<uint8_t>(resolution) & 0x03) << 10);
    const uint16_t value = static_cast<uint16_t>((control_register_1_value & 0xF3FF) | field);
    status = i2c_.write(i2c_addr_, control_register_1, value);
    if (status != I2cStatus::Success) {
        log(LogLevel::ERROR, "Failed to write control register for setting resolution");
    }
    return from_i2c(status);
}

template <typename I2CAdapterT, typename LoggerT>
Status MLX90641Sensor<I2CAdapterT, LoggerT>::set_refresh_rate(RefreshRate refresh_rate) {
    uint16_t control_register_1_value = 0;
    I2cStatus status = i2c_.read(i2c_addr_, control_register_1, 1, &control_register_1_value);
    if (status != I2cStatus::Success) {
        return from_i2c(status);
    }
    const uint16_t field = static_cast<uint16_t>((static_cast<uint8_t>(refresh_rate) & 0x07) << 7);
    const uint16_t value = static_cast<uint16_t>((control_register_1_value & 0xFC7F) | field);
    status = i2c_.write(i2c_addr_, control_register_1, value);
    return from_i2c(status);
}

template <typename I2CAdapterT, typename LoggerT>
void MLX90641Sensor<I2CAdapterT, LoggerT>::calculate_to(float emissivity, float tr) {
    const ParamsMLX90641& p = calibration_parameters_;
    const uint16_t sub_page = frame_data_[241];

    const float vdd = get_vdd();
    const float ta = get_ta();
    const float ta_k2 = (ta + kelvin_offset) * (ta + kelvin_offset);
    const float tr_k2 = (tr + kelvin_offset) * (tr + kelvin_offset);
    const float ta4 = ta_k2 * ta_k2;
    const float tr4 = tr_k2 * tr_k2;
    const float ta_tr = tr4 - (tr4 - ta4) / emissivity;

    float alpha_corr_r[8];
    alpha_corr_r[1] = 1.0f / (1.0f + p.ksTo[1] * 20.0f);
    alpha_corr_r[0] = alpha_corr_r[1] / (1.0f + p.ksTo[0] * 20.0f);
    alpha_corr_r[2] = 1.0f;
    alpha_corr_r[3] = (1.0f + p.ksTo[2] * p.ct[2]);
    alpha_corr_r[4] = alpha_corr_r[3] * (1.0f + p.ksTo[3] * (p.ct[4] - p.ct[3]));
    alpha_corr_r[5] = alpha_corr_r[4] * (1.0f + p.ksTo[4] * (p.ct[5] - p.ct[4]));
    alpha_corr_r[6] = alpha_corr_r[5] * (1.0f + p.ksTo[5] * (p.ct[6] - p.ct[5]));
    alpha_corr_r[7] = alpha_corr_r[6] * (1.0f + p.ksTo[6] * (p.ct[7] - p.ct[6]));

    //------------------------- Gain calculation -----------------------------------
    const float gain = p.gainEE / signed_word(frame_data_[202]);

    //------------------------- To calculation -------------------------------------
    float ir_data_cp = signed_word(frame_data_[200]) * gain;
    ir_data_cp = ir_data_cp - p.cpOffset * (1.0f + p.cpKta * (ta - 25.0f)) * (1.0f + p.cpKv * (vdd - 3.3f));

    for (std::size_t pixel_number = 0; pixel_number < num_pixels; pixel_number++) {
        float ir_data = signed_word(frame_data_[pixel_number]) * gain;

        ir_data = ir_data - p.offset[sub_page][pixel_number] * (1.0f + p.kta[pixel_number] * (ta - 25.0f)) *
                                (1.0f + p.kv[pixel_number] * (vdd - 3.3f));

        ir_data = ir_data - p.tgc * ir_data_cp;

        ir_data = ir_data / emissivity;

        const float alpha_compensated =
            (p.alpha[pixel_number] - p.tgc * p.cpAlpha) * (1.0f + p.KsTa * (ta - 25.0f));

        float sx = alpha_compensated * alpha_compensated * alpha_compensated * (ir_data + alpha_compensated * ta_tr);
        sx = sqrtf(sqrtf(sx)) * p.ksTo[1];

        float to = sqrtf(sqrtf(ir_data / (alpha_compensated * (1.0f - p.ksTo[1] * kelvin_offset) + sx) + ta_tr)) -
                   kelvin_offset;

        int range;
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

        to = sqrtf(sqrtf(ir_data / (alpha_compensated * alpha_corr_r[range] *
                                    (1.0f + p.ksTo[range] * (to - p.ct[range]))) +
                         ta_tr)) -
             kelvin_offset;
        // A zero divisor (e.g. a corrupt gain word) or a negative radicand makes
        // this NaN/inf; an out-of-spec value means the frame or calibration is
        // bad. Keep the previous frame's value for that pixel so a single glitch
        // cannot poison column_averages() and the BLE publish.
        if (std::isfinite(to) && to > pixel_temp_min_c && to < pixel_temp_max_c) {
            temps_[pixel_number] = to;
        }
    }
}

template <typename I2CAdapterT, typename LoggerT>
float MLX90641Sensor<I2CAdapterT, LoggerT>::get_vdd() const {
    const ParamsMLX90641& p = calibration_parameters_;
    const int resolution_ram = (frame_data_[240] & 0x0C00) >> 10;
    const float resolution_correction =
        static_cast<float>(1 << p.resolutionEE) / static_cast<float>(1 << resolution_ram);
    const float vdd_raw = signed_word(frame_data_[234]);
    return (resolution_correction * vdd_raw - p.vdd25) / p.kVdd + 3.3f;
}

template <typename I2CAdapterT, typename LoggerT>
float MLX90641Sensor<I2CAdapterT, LoggerT>::get_ta() const {
    const ParamsMLX90641& p = calibration_parameters_;
    const float vdd = get_vdd();
    const float ptat = signed_word(frame_data_[224]);
    float ptat_art = signed_word(frame_data_[192]);
    ptat_art = (ptat / (ptat * p.alphaPTAT + ptat_art)) * 262144.0f;  // 2^18

    float ta = (ptat_art / (1.0f + p.KvPTAT * (vdd - 3.3f)) - p.vPTAT25);
    ta = ta / p.KtPTAT + 25.0f;
    return ta;
}

template <typename I2CAdapterT, typename LoggerT>
float MLX90641Sensor<I2CAdapterT, LoggerT>::get_emissivity() const {
    return calibration_parameters_.emissivityEE;
}

template <typename I2CAdapterT, typename LoggerT>
Status MLX90641Sensor<I2CAdapterT, LoggerT>::check_eeprom_valid() const {
    const int device_select = ee_data_[10] & 0x0040;
    return (device_select != 0) ? Status::Success : Status::NotAnMlx90641;
}

template <typename I2CAdapterT, typename LoggerT>
void MLX90641Sensor<I2CAdapterT, LoggerT>::log(LogLevel level, const char* message) {
    logger_.log(level, message);
}

}  // namespace mlx90641
