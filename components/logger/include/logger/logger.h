#pragma once
#include "esp_err.h"
#include "logger/sink.h"

namespace logger {

// Drains the blackboard log ring into sink on a background task.
// blackboard::init() must have run first. sink must outlive the task; it is
// borrowed, not owned.
esp_err_t start(Sink* sink);

}
