#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include "i2c_adapter.hh"
#include "logger.hh"
#include "mlx90641_config.hh"
#include "mlx90641_eeprom_parser.hh"
#include "mlx90641_params.hh"

namespace mlx90641 {

/// @brief Outcome of a driver operation. Success is 0, as the coding guidelines require.
///
/// The I2C* values wrap I2cStatus; the rest correspond to the Melexis reference
/// driver's negative codes (-7 not an MLX90641, -8 frame sync, -10 corrupt EEPROM).
enum class Status : uint8_t {
    Success = 0,
    I2cNack,
    I2cBusError,
    I2cNoData,
    I2cVerifyMismatch,
    NotAnMlx90641,                // EEPROM device-select bit not set
    EepromCorrupt,                // uncorrectable Hamming error in the EEPROM dump
    CalibrationExtractionFailed,  // EEPROM parsed but produced no usable parameters
    DataReadyTimeout,             // no new frame within Mlx90641Config::data_ready_max_polls
    FrameSyncFailed,              // the new-data flag never cleared across 5 acknowledgements
};

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

constexpr std::size_t sensor_columns = 16;
constexpr std::size_t sensor_rows = 12;
constexpr std::size_t num_pixels = sensor_columns * sensor_rows;
constexpr std::size_t frame_data_size = 834;

/// @brief Average each of the 16 columns over the 12 rows of a row-major frame.
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

/// @brief MLX90641 driver: EEPROM calibration extraction, frame acquisition and
/// temperature computation, ported from the Melexis reference API.
///
/// Portable: I2C goes through I2CAdapterT (see i2c_adapter.hh); logging through
/// an owned LoggerT (see logger.hh), NullLogger by default. Both are resolved at
/// compile time.
///
/// All arithmetic is single precision: the Cortex-M4F has no double FPU.
template <typename I2CAdapterT, typename LoggerT = NullLogger>
class MLX90641Sensor {
    static_assert(is_i2c_adapter<I2CAdapterT>::value,
                  "I2CAdapterT must implement the I2CAdapter shape (see i2c_adapter.hh)");
    static_assert(is_logger<LoggerT>::value, "LoggerT must implement the Logger shape (see logger.hh)");

public:
    static constexpr std::size_t num_pixels = mlx90641::num_pixels;
    static constexpr std::size_t ee_data_size = eeprom_size;
    static constexpr std::size_t frame_data_size = mlx90641::frame_data_size;

    MLX90641Sensor(I2CAdapterT& i2c_adapter, uint8_t i2c_addr);

    /// @brief Bring up the bus, load calibration, apply `config`.
    Status init(const Mlx90641Config& config);
    /// @brief Acquire the next frame (either sub-page) and its ambient temperature.
    Status read_frame();
    void calculate_temps();
    std::array<float, num_pixels> get_temps() const;
    float get_ambient() const;

private:
    static constexpr uint16_t status_register = 0x8000;
    static constexpr uint16_t control_register_1 = 0x800D;
    static constexpr uint16_t status_new_data_mask = 0x0008;
    static constexpr uint16_t status_sub_page_mask = 0x0001;
    static constexpr uint16_t status_clear_new_data = 0x0030;
    static constexpr uint8_t frame_sync_max_attempts = 5;
    static constexpr float kelvin_offset = 273.15f;

    enum class HammingResult : uint8_t { Clean, Corrected, Uncorrectable };

    Status dump_ee();
    HammingResult hamming_decode();
    Status get_frame_data();
    Status extract_parameters();
    Status set_resolution(Resolution resolution);
    Status set_refresh_rate(RefreshRate refresh_rate);
    void calculate_to(float emissivity, float tr);
    float get_vdd() const;
    float get_ta() const;
    void bad_pixels_correction();
    float get_emissivity() const;
    Status check_eeprom_valid() const;
    void log(LogLevel level, const char* message);
    static float signed_word(uint16_t raw);

    I2CAdapterT& i2c_;
    uint8_t i2c_addr_;
    uint32_t data_ready_max_polls_;
    std::array<uint16_t, ee_data_size> ee_data_;
    std::array<uint16_t, frame_data_size> frame_data_;
    std::array<float, num_pixels> temps_;
    ParamsMLX90641 calibration_parameters_;
    float ambient_;
    LoggerT logger_;
};

}  // namespace mlx90641

#include "mlx90641_driver_impl.hh"
