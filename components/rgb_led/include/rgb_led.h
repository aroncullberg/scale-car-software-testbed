#pragma once

#include <cstdint>
#include "esp_err.h"
#include "led/frame.hpp"

struct led_strip_t;

namespace led {

// One SPI3/DMA strip buffer is routed to the existing three light outputs.
// Use from the vehicle task; DShot owns every RMT channel on the S3.
class Controller {
  public:
    Controller() = default;
    ~Controller();
    Controller(const Controller&) = delete;
    Controller& operator=(const Controller&) = delete;
    esp_err_t start();
    esp_err_t update(const Frame& frame);
    void stop();

  private:
    esp_err_t refresh(int gpio, int count, const Color* pixels);
    led_strip_t* strip_{nullptr};
    Frame last_frame_{};
    bool frame_written_ = false;
};

} // namespace led
