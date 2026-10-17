#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include "dshot/telemetry.hpp"
#include "power_monitor.h"
#include "rclink/receiver.hpp"

namespace rc_telemetry {

void publish(rclink::Receiver& receiver, const std::array<dshot::Telemetry, 4>& motors,
             const PowerMonitor::Reading& power, std::optional<uint32_t> ground_speed_cms);

} // namespace rc_telemetry
