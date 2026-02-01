//
// BNO08x (BNO080/BNO085) backend implementation
//

#pragma once

#include "imu_backend.h"
#include "BNO08x.hpp"

namespace imu
{

struct BNO08xConfig {
    bool install_isr_service{true};  // Set false if gpio_install_isr_service() already called
};

class BNO08xBackend : public Backend
{
public:
    using Config = BNO08xConfig;

    explicit BNO08xBackend(const Config& cfg = {});
    ~BNO08xBackend() override;

    esp_err_t start(const SpiConfig& spi,
                   uint32_t gyro_interval_us,
                   uint32_t accel_interval_us,
                   uint32_t quat_interval_us) override;
    esp_err_t stop() override;

    bool has_fusion() const override { return true; }
    uint32_t max_gyro_rate_hz() const override { return 1000; }
    uint32_t max_accel_rate_hz() const override { return 1000; }
    uint32_t max_quat_rate_hz() const override { return 400; }

    const char* name() const override { return "BNO08x"; }

    // Access to underlying sensor for advanced configuration (calibration, etc.)
    BNO08x& sensor() { return *sensor_; }

    BNO08xBackend(const BNO08xBackend&) = delete;
    BNO08xBackend& operator=(const BNO08xBackend&) = delete;

private:
    void handle_accel();
    void handle_gyro();
    void handle_quat();

    static Accuracy convert_accuracy(BNO08xAccuracy acc);

    Config cfg_;
    BNO08x* sensor_{nullptr};
    bool running_{false};
};

} // namespace imu
