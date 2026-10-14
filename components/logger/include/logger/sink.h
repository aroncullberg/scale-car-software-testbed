#pragma once
#include <cstddef>
#include <cstdint>

namespace logger {

// Destination for drained log records. A record is the raw blackboard frame
// (header + payload). Implementations decide where the bytes go: stdout, TCP,
// SD. Return false if the record could not be handled.
class Sink {
public:
    virtual ~Sink() = default;
    virtual bool write(const uint8_t* record, size_t len) = 0;
};

}
