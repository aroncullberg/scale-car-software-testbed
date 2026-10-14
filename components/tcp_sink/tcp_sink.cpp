#include "tcp_sink/tcp_sink.h"

#include <algorithm>
#include <cstring>

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "lwip/sockets.h"

namespace tcp_sink {

static const char* TAG = "tcp_sink";

static constexpr size_t PREFIX = 2;          // per-entry length prefix
static constexpr size_t INTERNAL_FALLBACK = 64 * 1024;

esp_err_t TcpSink::start(const Config& config) {
    cfg_ = config;

    buf_ = static_cast<uint8_t*>(heap_caps_malloc(cfg_.buffer_bytes, MALLOC_CAP_SPIRAM));
    if (buf_) {
        cap_ = cfg_.buffer_bytes;
    } else {
        ESP_LOGW(TAG, "PSRAM alloc of %u bytes failed; falling back to %u bytes internal",
                 (unsigned)cfg_.buffer_bytes, (unsigned)INTERNAL_FALLBACK);
        buf_ = static_cast<uint8_t*>(malloc(INTERNAL_FALLBACK));
        if (!buf_) {
            ESP_LOGE(TAG, "buffer alloc failed");
            return ESP_ERR_NO_MEM;
        }
        cap_ = INTERNAL_FALLBACK;
    }

    mtx_ = xSemaphoreCreateMutex();
    if (!mtx_) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = wifi_init_softap();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SoftAP init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    if (xTaskCreate(server_task_entry, "tcp_srv", 4096, this, 4, &task_) != pdPASS) {
        ESP_LOGE(TAG, "failed to create server task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "ready: SSID '%s' port %u, %u byte ring", cfg_.ssid, cfg_.port,
             (unsigned)cap_);
    return ESP_OK;
}

esp_err_t TcpSink::wifi_init_softap() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        return ret;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    // The default event loop may already exist; ignore that case.
    esp_err_t loop = esp_event_loop_create_default();
    if (loop != ESP_OK && loop != ESP_ERR_INVALID_STATE) {
        return loop;
    }

    esp_netif_create_default_wifi_ap();

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));

    wifi_config_t ap_cfg = {};
    size_t ssid_len = strnlen(cfg_.ssid, sizeof(ap_cfg.ap.ssid));
    memcpy(ap_cfg.ap.ssid, cfg_.ssid, ssid_len);
    ap_cfg.ap.ssid_len = static_cast<uint8_t>(ssid_len);
    ap_cfg.ap.channel = cfg_.channel;
    ap_cfg.ap.max_connection = cfg_.max_clients;

    size_t pw_len = strnlen(cfg_.password, sizeof(ap_cfg.ap.password));
    if (pw_len >= 8) {
        memcpy(ap_cfg.ap.password, cfg_.password, pw_len);
        ap_cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
    } else {
        ap_cfg.ap.authmode = WIFI_AUTH_OPEN;
        if (pw_len > 0) {
            ESP_LOGW(TAG, "password too short for WPA2, opening network");
        }
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    return ESP_OK;
}

// --- ring (caller holds mtx_) ---

void TcpSink::buf_write(const uint8_t* src, size_t n) {
    size_t first = std::min(n, cap_ - head_);
    memcpy(buf_ + head_, src, first);
    if (n > first) {
        memcpy(buf_, src + first, n - first);
    }
    head_ = (head_ + n) % cap_;
    used_ += n;
}

void TcpSink::buf_read(size_t from_tail, uint8_t* dst, size_t n) const {
    size_t pos = (tail_ + from_tail) % cap_;
    size_t first = std::min(n, cap_ - pos);
    memcpy(dst, buf_ + pos, first);
    if (n > first) {
        memcpy(dst + first, buf_, n - first);
    }
}

bool TcpSink::ring_push(const uint8_t* rec, size_t len) {
    if (len == 0 || len > UINT16_MAX) {
        return false;
    }
    const size_t need = PREFIX + len;
    if (need > cap_) {
        return false; // record larger than the whole ring
    }

    xSemaphoreTake(mtx_, portMAX_DELAY);

    // Make room by dropping whole oldest entries (overwrite-oldest).
    while (cap_ - used_ < need) {
        uint8_t hdr[PREFIX];
        buf_read(0, hdr, PREFIX);
        size_t old_len = static_cast<size_t>(hdr[0]) | (static_cast<size_t>(hdr[1]) << 8);
        size_t old_entry = PREFIX + old_len;
        tail_ = (tail_ + old_entry) % cap_;
        used_ -= old_entry;
        dropped_.fetch_add(1, std::memory_order_relaxed);
    }

    uint8_t hdr[PREFIX] = {static_cast<uint8_t>(len & 0xff),
                           static_cast<uint8_t>((len >> 8) & 0xff)};
    buf_write(hdr, PREFIX);
    buf_write(rec, len);

    xSemaphoreGive(mtx_);
    return true;
}

size_t TcpSink::ring_pop(uint8_t* out, size_t max) {
    xSemaphoreTake(mtx_, portMAX_DELAY);
    if (used_ == 0) {
        xSemaphoreGive(mtx_);
        return 0;
    }
    uint8_t hdr[PREFIX];
    buf_read(0, hdr, PREFIX);
    size_t len = static_cast<size_t>(hdr[0]) | (static_cast<size_t>(hdr[1]) << 8);
    size_t copy = std::min(len, max);
    buf_read(PREFIX, out, copy);
    tail_ = (tail_ + PREFIX + len) % cap_;
    used_ -= (PREFIX + len);
    xSemaphoreGive(mtx_);
    return copy;
}

bool TcpSink::write(const uint8_t* record, size_t len) {
    return ring_push(record, len);
}

// --- server ---

void TcpSink::server_task_entry(void* arg) {
    static_cast<TcpSink*>(arg)->server_task();
}

static bool send_all(int fd, const uint8_t* data, size_t len) {
    size_t sent = 0;
    while (sent < len) {
        int n = ::send(fd, data + sent, len - sent, 0);
        if (n <= 0) {
            return false;
        }
        sent += static_cast<size_t>(n);
    }
    return true;
}

void TcpSink::server_task() {
    int listen_fd = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listen_fd < 0) {
        ESP_LOGE(TAG, "socket() failed");
        vTaskDelete(nullptr);
        return;
    }
    int opt = 1;
    ::setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(cfg_.port);

    if (::bind(listen_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0 ||
        ::listen(listen_fd, 1) < 0) {
        ESP_LOGE(TAG, "bind/listen failed on port %u", cfg_.port);
        ::close(listen_fd);
        vTaskDelete(nullptr);
        return;
    }

    uint8_t rec[280]; // max record: header + 255 payload, with margin

    while (true) {
        ESP_LOGI(TAG, "waiting for client on port %u", cfg_.port);
        int client = ::accept(listen_fd, nullptr, nullptr);
        if (client < 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        ESP_LOGI(TAG, "client connected");

        while (true) {
            size_t len = ring_pop(rec, sizeof(rec));
            if (len == 0) {
                vTaskDelay(pdMS_TO_TICKS(5));
                continue;
            }
            if (!send_all(client, rec, len)) {
                break; // client gone; the in-flight record is lost, rest stays buffered
            }
        }

        ::close(client);
        ESP_LOGW(TAG, "client disconnected");
    }
}

}
