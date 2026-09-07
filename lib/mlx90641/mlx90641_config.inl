// Inline definitions for mlx90641_config.hh. Included by the header; do not include directly.
#pragma once

namespace mlx90641 {

constexpr uint32_t frame_period_us(RefreshRate rate) {
    return 2000000u >> static_cast<uint8_t>(rate);
}

}  // namespace mlx90641
