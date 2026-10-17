#include "rc_telemetry/publisher.hpp"

#include <algorithm>

namespace rc_telemetry {

void publish(rclink::Receiver& receiver, const std::array<dshot::Telemetry, 4>& motors,
             const PowerMonitor::Reading& power, std::optional<uint32_t> ground_speed_cms) {
    constexpr rclink::telemetry::Options timing{100, 300};
    for (std::size_t i = 0; i < motors.size(); ++i) {
        const auto& motor = motors[i];
        if (!motor.available || (motor.speed.state != dshot::ErpmState::Moving &&
                                 motor.speed.state != dshot::ErpmState::Stopped)) continue;
        rclink::telemetry::Rpm rpm;
        rpm.source_id = i + 1;
        rpm.count = 1;
        rpm.rpm[0] = motor.speed.rpm;
        receiver.submit(rpm, timing);
    }
    if (power.valid) {
        rclink::telemetry::Battery battery;
        battery.voltage_mv = std::max(power.voltage_mv, int32_t{0});
        battery.current_ma = std::max(power.current_ma, int32_t{0});
        battery.consumed_mah = std::min(power.consumed_mah, uint32_t{0xffffff});
        battery.remaining_pct = std::min(power.remaining_pct, uint8_t{100});
        receiver.submit(battery, timing);
    }
    if (ground_speed_cms) {
        rclink::telemetry::Airspeed speed;
        speed.speed_cms = *ground_speed_cms;
        receiver.submit(speed, timing);
    }
}

} // namespace rc_telemetry
