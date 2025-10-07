#pragma once

#include <cstdint>
#include <array>

namespace mlx90641 {

/// @brief Struct containing all calibration parameters for the MLX90641 sensor
struct ParamsMLX90641{
        std::int16_t kVdd;
        std::int16_t vdd25;
        float KvPTAT;
        float KtPTAT;
        std::uint16_t vPTAT25;
        float alphaPTAT;
        std::int16_t gainEE;
        float tgc;
        float cpKv;
        float cpKta;
        std::uint8_t resolutionEE;
        std::uint8_t calibrationModeEE;
        float KsTa;
        std::array<float, 8> ksTo;
        std::array<std::int16_t, 8> ct;
        std::array<float, 192> alpha;    
        std::array<std::array<std::int16_t, 192>, 2> offset;    
        std::array<float, 192> kta;    
        std::array<float, 192> kv;
        float cpAlpha;
        std::int16_t cpOffset;
        float emissivityEE; 
        std::array<std::uint16_t, 2> brokenPixels;
    }; 

} // namespace mlx90641