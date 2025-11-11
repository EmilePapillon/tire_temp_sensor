#pragma once
#include <array>
#include <cstdint>
#include "i2c_adapter.hh"
#include "mlx90641_params.hh"
#include "logger.hh"

namespace mlx90641 {
class MLX90641Sensor {
public:
    static constexpr size_t num_pixels = 192;
    static constexpr size_t ee_data_size = 832;
    static constexpr size_t frame_data_size = 834;

    MLX90641Sensor(I2CAdapter& i2c_adapter, uint8_t i2c_addr = 0x33, Logger* logger_ptr = nullptr);

    bool init();
    bool read_frame();
    void calculate_temps();
    std::array<float, num_pixels> get_temps() const;
    float get_ambient() const;

private:
    int dump_ee();
    int hamming_decode();
    int get_frame_data();
    int extract_parameters();
    int set_resolution(uint8_t resolution);
    int get_cur_resolution() const;
    int set_refresh_rate(uint8_t refresh_rate);
    int get_refresh_rate() const;
    void calculate_to(float emissivity, float tr);
    void get_image();
    float get_vdd() const;
    float get_ta() const;
    int get_sub_page_number() const;
    void bad_pixels_correction();
    float get_emissivity() const;
    int extract_deviating_pixels();
    int check_eeprom_valid() const;
    void log(Logger::Level level, const char* message);

    I2CAdapter& i2c_;
    uint8_t i2c_addr_;
    std::array<uint16_t, ee_data_size> ee_data_;
    std::array<uint16_t, frame_data_size> frame_data_;
    std::array<float, num_pixels> temps_;
    ParamsMLX90641 calibration_parameters_;
    float ambient_;
    Logger* logger_; 

};
} // namespace mlx90641