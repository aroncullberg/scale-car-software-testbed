#pragma once

#include "esp_err.h"
namespace recording { class Logger; }

namespace sensors {

esp_err_t start(recording::Logger& logger);

} // namespace sensors
