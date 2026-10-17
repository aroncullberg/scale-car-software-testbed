#pragma once

#include <array>
#include "esc_passthrough.h"
#include "vehicle/types.hpp"

namespace vehicle::detail {

esp_err_t start_passthrough(const std::array<MotorConfig, motor_count>& motors,
                            esc_passthrough_lifecycle_t lifecycle,
                            esc_passthrough_t** session);
esp_err_t stop_passthrough(esc_passthrough_t** session);
void pump_passthrough(esc_passthrough_t* session, bool& was_connected);

} // namespace vehicle::detail
