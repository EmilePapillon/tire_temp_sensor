#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include "i2c_adapter.hh"
#include "logger.hh"
#include "mlx90641_config.hh"
#include "mlx90641_eeprom_parser.hh"
#include "mlx90641_params.hh"

/// @file mlx90641_driver.hh
/// @brief MLX90641 driver: calibration extraction, frame acquisition and temperature computation.

namespace mlx90641 {

/// @brief Outcome of a driver operation. Success is 0, as the coding guidelines require.
///
/// The I2c* values wrap I2cStatus; the rest correspond to the Melexis reference
/// driver's negative codes (-7 not an MLX90641, -8 frame sync, -10 corrupt EEPROM).
enum class Status : uint8_t {
    Success = 0,                  ///< The operation completed.
    I2cNack,                      ///< The sensor did not acknowledge on the bus.
    I2cBusError,                  ///< Bus-level failure (overflow, timeout).
    I2cNoData,                    ///< The sensor returned no bytes on a read.
    I2cVerifyMismatch,            ///< A register write did not read back as written.
    NotAnMlx90641,                ///< EEPROM device-select bit not set.
    EepromCorrupt,                ///< Uncorrectable Hamming error in the EEPROM dump.
    CalibrationExtractionFailed,  ///< EEPROM parsed but produced no usable parameters.
    DataReadyTimeout,             ///< No new frame within Mlx90641Config::data_ready_max_polls.
    FrameSyncFailed,              ///< The new-data flag never cleared across 5 acknowledgements.
};

/// @brief Human-readable name of a driver status, for log messages.
/// @param status The status to name.
/// @return A static lower-case string such as "data ready timeout"; never null.
const char* status_name(Status status);

/// @brief Lift a bus status into the driver's status space.
/// @param status The I2C outcome.
/// @return The corresponding Status; Success maps to Success.
Status from_i2c(I2cStatus status);

constexpr std::size_t sensor_columns = 16;                            ///< Pixels across the tread.
constexpr std::size_t sensor_rows = 12;                               ///< Pixels along the tread.
constexpr std::size_t num_pixels = sensor_columns * sensor_rows;      ///< Pixels per frame.
constexpr std::size_t frame_data_size = 834;                          ///< Words in a raw RAM frame + 2 status words.

/// @brief Average each of the 16 columns over the 12 rows of a row-major frame.
/// @param temps Per-pixel temperatures, row-major, index = row * 16 + column.
/// @return One average per column, [0] = leftmost.
std::array<float, sensor_columns> column_averages(const std::array<float, num_pixels>& temps);

/// @brief MLX90641 driver, ported from the Melexis reference API.
///
/// Portable: I2C goes through @p I2CAdapterT (see i2c_adapter.hh); logging
/// through an owned @p LoggerT (see logger.hh). Both are resolved at compile
/// time. All arithmetic is single precision: the Cortex-M4F has no double FPU.
///
/// @tparam I2CAdapterT A type satisfying is_i2c_adapter; injected by reference so tests can script the bus.
/// @tparam LoggerT A default-constructible type satisfying is_logger; owned. NullLogger by default.
template <typename I2CAdapterT, typename LoggerT = NullLogger>
class MLX90641Sensor {
    static_assert(is_i2c_adapter<I2CAdapterT>::value,
                  "I2CAdapterT must implement the I2CAdapter shape (see i2c_adapter.hh)");
    static_assert(is_logger<LoggerT>::value, "LoggerT must implement the Logger shape (see logger.hh)");

public:
    static constexpr std::size_t num_pixels = mlx90641::num_pixels;            ///< Pixels per frame.
    static constexpr std::size_t ee_data_size = eeprom_size;                   ///< Words in the EEPROM dump.
    static constexpr std::size_t frame_data_size = mlx90641::frame_data_size;  ///< Words in a raw frame.

    /// @brief Bind to a bus. Nothing is touched until init().
    /// @param i2c_adapter The bus adapter; must outlive this object.
    /// @param i2c_addr 7-bit I2C address of the sensor (0x33 by default on the part).
    MLX90641Sensor(I2CAdapterT& i2c_adapter, uint8_t i2c_addr);

    /// @brief Bring up the bus, dump and validate the EEPROM, extract calibration, apply @p config.
    ///
    /// Failing to program the resolution or refresh rate is logged as a warning
    /// but does not fail init: the sensor still produces frames at its defaults.
    /// @param config Bus speed, resolution, refresh rate and polling limit.
    /// @return Success, or the first fatal failure.
    Status init(const Mlx90641Config& config);

    /// @brief Acquire the next frame (either sub-page) and its ambient temperature.
    /// @return Success, DataReadyTimeout, FrameSyncFailed, or a bus failure.
    Status read_frame();

    /// @brief Compute per-pixel object temperatures from the last frame read.
    ///
    /// Uses the EEPROM emissivity and the frame's own ambient as the reflected temperature.
    void calculate_temps();

    /// @brief Temperatures computed by the last calculate_temps().
    /// @return Per-pixel degrees Celsius, row-major.
    std::array<float, num_pixels> get_temps() const;

    /// @brief Ambient (die) temperature from the last read_frame().
    /// @return Degrees Celsius.
    float get_ambient() const;

private:
    static constexpr uint16_t status_register = 0x8000;         ///< RAM status register.
    static constexpr uint16_t control_register_1 = 0x800D;      ///< RAM control register 1.
    static constexpr uint16_t status_new_data_mask = 0x0008;    ///< Status bit: a new frame is available.
    static constexpr uint16_t status_sub_page_mask = 0x0001;    ///< Status bit: which sub-page it is.
    static constexpr uint16_t status_clear_new_data = 0x0030;   ///< Value written to acknowledge a frame.
    static constexpr uint8_t frame_sync_max_attempts = 5;       ///< Re-reads before FrameSyncFailed.
    static constexpr float kelvin_offset = 273.15f;             ///< Celsius to Kelvin.

    /// @brief Result of Hamming-checking the EEPROM dump.
    enum class HammingResult : uint8_t {
        Clean,          ///< No errors.
        Corrected,      ///< One or more single-bit errors were corrected in place.
        Uncorrectable,  ///< At least one word had a multi-bit error.
    };

    /// @brief Read the whole EEPROM into ee_data_ and Hamming-decode it.
    /// @return Success (also for corrected errors, which are logged), EepromCorrupt, or a bus failure.
    Status dump_ee();

    /// @brief Verify and strip the Hamming check bits from ee_data_[16..831], in place.
    /// @return Whether the data was clean, corrected, or uncorrectable.
    HammingResult hamming_decode();

    /// @brief Wait for, acknowledge and read one raw frame into frame_data_.
    /// @return Success, DataReadyTimeout, FrameSyncFailed, or a bus failure.
    Status get_frame_data();

    /// @brief Check the device-select bit and parse ee_data_ into calibration_parameters_.
    /// @return Success, NotAnMlx90641, or CalibrationExtractionFailed.
    Status extract_parameters();

    /// @brief Program the ADC resolution field of control register 1.
    /// @param resolution The field value.
    /// @return Success or a bus failure.
    Status set_resolution(Resolution resolution);

    /// @brief Program the refresh-rate field of control register 1.
    /// @param refresh_rate The field value.
    /// @return Success or a bus failure.
    Status set_refresh_rate(RefreshRate refresh_rate);

    /// @brief Object temperature calculation from the Melexis reference (MLX90641_CalculateTo).
    /// @param emissivity Emissivity of the target surface, 0..1.
    /// @param tr Reflected temperature in degrees Celsius.
    void calculate_to(float emissivity, float tr);

    /// @brief Supply voltage derived from the frame's VDD pixel and the current resolution.
    /// @return Volts.
    float get_vdd() const;

    /// @brief Ambient temperature derived from the frame's PTAT pixels.
    /// @return Degrees Celsius.
    float get_ta() const;

    /// @brief Replace the (at most two) EEPROM-flagged broken pixels by interpolating their neighbours.
    void bad_pixels_correction();

    /// @brief Emissivity stored in the EEPROM.
    /// @return Emissivity, 0..1.
    float get_emissivity() const;

    /// @brief Check the EEPROM device-select bit that identifies an MLX90641.
    /// @return Success or NotAnMlx90641.
    Status check_eeprom_valid() const;

    /// @brief Forward a message to the owned logger.
    /// @param level Severity.
    /// @param message NUL-terminated text.
    void log(LogLevel level, const char* message);

    /// @brief Reinterpret a raw 16-bit RAM word as the signed value the sensor stores.
    /// @param raw The word as read from RAM.
    /// @return The two's-complement value as a float.
    static float signed_word(uint16_t raw);

    I2CAdapterT& i2c_;                                   ///< The injected bus adapter.
    uint8_t i2c_addr_;                                   ///< Sensor address on the bus.
    uint32_t data_ready_max_polls_;                      ///< From Mlx90641Config, set in init().
    std::array<uint16_t, ee_data_size> ee_data_;         ///< Hamming-decoded EEPROM image.
    std::array<uint16_t, frame_data_size> frame_data_;   ///< Last raw frame; [240] control reg 1, [241] sub-page.
    std::array<float, num_pixels> temps_;                ///< Output of calculate_temps().
    ParamsMLX90641 calibration_parameters_;              ///< Extracted in init().
    float ambient_;                                      ///< Output of read_frame().
    LoggerT logger_;                                     ///< Owned, default-constructed.
};

}  // namespace mlx90641

#include "mlx90641_driver.inl"
