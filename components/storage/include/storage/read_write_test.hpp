#pragma once

#include <cstddef>
#include <cstdint>
#include "esp_err.h"

namespace storage {

struct ReadWriteResult {
    std::size_t bytes_written = 0;
    std::size_t bytes_read = 0;
    std::uint64_t write_elapsed_us = 0;
    std::uint64_t read_elapsed_us = 0;
    std::uint64_t longest_write_us = 0;
    std::uint64_t longest_read_us = 0;
    std::uint64_t longest_sync_us = 0;
    std::size_t mismatch_offset = 0;
    int filesystem_errno = 0;
    bool verified = false;
};

using TestProgress = void (*)(void* context);

// Creates path exclusively. Removes only its own file, and only after successful
// verification. Progress runs between blocks, never while holding the SPI bus.
esp_err_t test_read_write(const char* path, std::size_t size_bytes,
                         ReadWriteResult& result, TestProgress progress = nullptr,
                         void* context = nullptr, std::size_t transfer_bytes = 4096);

} // namespace storage
