#include "logger/stdout_sink.h"

#include <cstdio>

#include "blackboard/blackboard.h"

namespace logger {

bool StdoutSink::write(const uint8_t* record, size_t len) {
    if (len < blackboard::RECORD_HEADER_SIZE) {
        return false;
    }

    uint8_t topic = record[0];
    uint8_t payload_len = record[1];
    uint64_t ts = static_cast<uint64_t>(record[2]) |
                  (static_cast<uint64_t>(record[3]) << 8) |
                  (static_cast<uint64_t>(record[4]) << 16) |
                  (static_cast<uint64_t>(record[5]) << 24) |
                  (static_cast<uint64_t>(record[6]) << 32) |
                  (static_cast<uint64_t>(record[7]) << 40);

    const uint8_t* payload = record + blackboard::RECORD_HEADER_SIZE;
    size_t avail = len - blackboard::RECORD_HEADER_SIZE;
    size_t n = payload_len < avail ? payload_len : avail;

    char hex[2 * 255 + 1];
    for (size_t i = 0; i < n; i++) {
        snprintf(hex + i * 2, 3, "%02x", payload[i]);
    }
    hex[n * 2] = '\0';

    printf("BB topic=%u ts=%llu len=%u %s\n", topic, (unsigned long long)ts,
           payload_len, hex);
    return true;
}

}
