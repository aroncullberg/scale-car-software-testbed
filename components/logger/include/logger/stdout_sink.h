#pragma once
#include "logger/sink.h"

namespace logger {

// Bring-up sink: prints each record as "BB topic=.. ts=.. len=.. <hex>".
// Bandwidth-limited by the console, so it cannot keep up with full-rate data,
// but it proves the ring drains and lets records be eyeballed.
class StdoutSink : public Sink {
public:
    bool write(const uint8_t* record, size_t len) override;
};

}
