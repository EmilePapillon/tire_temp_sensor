#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include "i2c_adapter.hh"
#include "logger.hh"
#include "mlx90641_eeprom_parser.hh"
#include "mlx90641_params.hh"

namespace mlx90641 {

// Control register 1 (0x800D) fields, per the MLX90641 datasheet / Melexis
// reference driver. Values are the raw register field encodings.
enum class Resolution : uint8_t {
    Bits16 = 0x00,
    Bits17 = 0x01,
    Bits18 = 0x02,
    Bits19 = 0x03,
};

enum class RefreshRate : uint8_t {
    Hz0_5 = 0x00,
    Hz1 = 0x01,
    Hz2 = 0x02,
    Hz4 = 0x03,
    Hz8 = 0x04,
    Hz16 = 0x05,
    Hz32 = 0x06,
    Hz64 = 0x07,
};

/// @brief Everything init() needs. No defaults: the caller (config.hh) owns every value.
struct Mlx90641Config {
    uint32_t i2c_freq_khz;
    Resolution resolution;
    RefreshRate refresh_rate;
};

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
/// Integer return codes on private helpers follow the Melexis driver
/// (0 = success, negative = error).
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

    /// @brief Bring up the bus, load calibration, apply `config`. False on failure.
    bool init(const Mlx90641Config& config);
    bool read_frame();
    void calculate_temps();
    std::array<float, num_pixels> get_temps() const;
    float get_ambient() const;

private:
    static constexpr uint16_t status_register = 0x8000;
    static constexpr uint16_t control_register_1 = 0x800D;

    int dump_ee();
    int hamming_decode();
    int get_frame_data();
    int extract_parameters();
    int set_resolution(Resolution resolution);
    int get_cur_resolution() const;
    int set_refresh_rate(RefreshRate refresh_rate);
    int get_refresh_rate() const;
    void calculate_to(float emissivity, float tr);
    void get_image();
    float get_vdd() const;
    float get_ta() const;
    int get_sub_page_number() const;
    void bad_pixels_correction();
    float get_emissivity() const;
    int check_eeprom_valid() const;
    void log(LogLevel level, const char* message);

    I2CAdapterT& i2c_;
    uint8_t i2c_addr_;
    std::array<uint16_t, ee_data_size> ee_data_;
    std::array<uint16_t, frame_data_size> frame_data_;
    std::array<float, num_pixels> temps_;
    ParamsMLX90641 calibration_parameters_;
    float ambient_;
    LoggerT logger_;
};

}  // namespace mlx90641

#include "mlx90641_driver_impl.hh"
