#include "storage/read_write_test.hpp"

#include <algorithm>
#include <cerrno>
#include <fcntl.h>
#include <memory>
#include <new>
#include <unistd.h>
#include "esp_timer.h"

namespace storage {
namespace {
constexpr std::size_t maximum_transfer_bytes = 4096;
constexpr std::size_t sync_bytes = 64 * 1024;

std::uint8_t expected_byte(std::size_t offset) {
    // Include the block number so swapped or repeated sectors also fail.
    const std::uint32_t mixed = std::uint32_t(offset) * 2654435761U;
    return std::uint8_t(mixed ^ (mixed >> 8) ^ (mixed >> 16) ^ (mixed >> 24));
}

std::uint64_t elapsed_since(std::int64_t start) {
    return std::uint64_t(esp_timer_get_time() - start);
}
}

esp_err_t test_read_write(const char* path, std::size_t size_bytes,
                         ReadWriteResult& result, TestProgress progress, void* context,
                         std::size_t transfer_bytes) {
    result = {};
    if (!path || !*path || !size_bytes || !transfer_bytes || transfer_bytes > maximum_transfer_bytes)
        return ESP_ERR_INVALID_ARG;
    const std::unique_ptr<std::uint8_t[]> buffer(new (std::nothrow) std::uint8_t[transfer_bytes]);
    if (!buffer) return ESP_ERR_NO_MEM;

    int file = open(path, O_WRONLY | O_CREAT | O_EXCL, 0600);
    if (file < 0) {
        result.filesystem_errno = errno;
        return ESP_FAIL;
    }
    auto fail = [&]() {
        result.filesystem_errno = errno;
        if (file >= 0) close(file);
        return ESP_FAIL;
    };
    const auto write_start = esp_timer_get_time();
    std::size_t next_sync = sync_bytes;
    while (result.bytes_written < size_bytes) {
        const auto count = std::min(transfer_bytes, size_bytes - result.bytes_written);
        for (std::size_t i = 0; i < count; ++i)
            buffer[i] = expected_byte(result.bytes_written + i);
        std::size_t sent = 0;
        const auto write_start_block = esp_timer_get_time();
        while (sent < count) {
            const auto written = write(file, buffer.get() + sent, count - sent);
            if (written < 0 && errno == EINTR) continue;
            if (written <= 0) {
                if (written == 0) errno = EIO;
                return fail();
            }
            sent += std::size_t(written);
            result.bytes_written += std::size_t(written);
        }
        result.longest_write_us = std::max(result.longest_write_us, elapsed_since(write_start_block));
        if (result.bytes_written >= next_sync || result.bytes_written == size_bytes) {
            const auto sync_start = esp_timer_get_time();
            if (fsync(file) != 0) return fail();
            result.longest_sync_us = std::max(result.longest_sync_us, elapsed_since(sync_start));
            next_sync += sync_bytes;
        }
        if (progress) progress(context);
    }
    result.write_elapsed_us = elapsed_since(write_start);
    if (close(file) != 0) { file = -1; return fail(); }
    file = open(path, O_RDONLY);
    if (file < 0) return fail();

    const auto read_start = esp_timer_get_time();
    while (result.bytes_read < size_bytes) {
        const auto count = std::min(transfer_bytes, size_bytes - result.bytes_read);
        const auto read_start_block = esp_timer_get_time();
        const auto received = read(file, buffer.get(), count);
        result.longest_read_us = std::max(result.longest_read_us, elapsed_since(read_start_block));
        if (received < 0 && errno == EINTR) continue;
        if (received <= 0) {
            if (received == 0) errno = EIO;
            return fail();
        }
        for (std::size_t i = 0; i < std::size_t(received); ++i) {
            if (buffer[i] != expected_byte(result.bytes_read + i)) {
                result.mismatch_offset = result.bytes_read + i;
                errno = EILSEQ;
                return fail();
            }
        }
        result.bytes_read += std::size_t(received);
        if (progress) progress(context);
    }
    result.read_elapsed_us = elapsed_since(read_start);
    if (close(file) != 0) { file = -1; return fail(); }
    file = -1;
    result.verified = true;
    if (unlink(path) != 0) return fail();
    return ESP_OK;
}

} // namespace storage
