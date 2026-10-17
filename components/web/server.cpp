#include "web/server.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <inttypes.h>
#include <memory>
#include <new>
#include <sys/stat.h>
#include <unistd.h>
#include "cJSON.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "sdkconfig.h"
#include "nvs_flash.h"

extern const uint8_t page_start[] asm("_binary_car_page_html_gz_start");
extern const uint8_t page_end[] asm("_binary_car_page_html_gz_end");

namespace web {
namespace {
Server& self(httpd_req_t* r) { return *static_cast<Server*>(r->user_ctx); }
esp_err_t json(httpd_req_t* r, const char* text) {
    httpd_resp_set_type(r, "application/json");
    httpd_resp_set_hdr(r, "Cache-Control", "no-store");
    return httpd_resp_sendstr(r, text);
}
esp_err_t unavailable(httpd_req_t* r, const char* message) {
    httpd_resp_set_status(r, "503 Service Unavailable");
    return httpd_resp_sendstr(r, message);
}
esp_err_t tuning_json(httpd_req_t* r, const vehicle::YawRateConfig& c) {
    char body[256];
    std::snprintf(body, sizeof(body), "{\"max_yaw_deg_s\":%.6g,\"p\":%.6g,\"i\":%.6g,\"d\":%.6g,\"d_cutoff_hz\":%.6g,\"integral_limit\":%.6g}",
                  double(c.maximum_yaw_rate_rads * 57.295779513f), double(c.proportional_gain),
                  double(c.integral_gain), double(c.derivative_gain), double(c.derivative_cutoff_hz), double(c.integral_limit));
    return json(r, body);
}
struct Download {
    Server* server;
    httpd_req_t* request;
    int file;
    uint64_t length;
    char name[13];
};
}

esp_err_t Server::start() {
    if (task_) return ESP_ERR_INVALID_STATE;
    return xTaskCreatePinnedToCore(service_task, "web_service", 8192, this, 1, &task_, 1) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

void Server::service_task(void* context) {
    auto& server = *static_cast<Server*>(context);
    const auto error = server.initialize();
    if (error != ESP_OK) {
        ESP_LOGE("web", "Startup failed: %s", esp_err_to_name(error));
        server.task_ = nullptr;
        vTaskDelete(nullptr);
        return;
    }
    server.monitor_cpu();
}

esp_err_t Server::initialize() {
    const size_t ssid_length = std::strlen(CONFIG_CAR_WIFI_SSID);
    const size_t password_length = std::strlen(CONFIG_CAR_WIFI_PASSWORD);
    if (!ssid_length || ssid_length > 32 || password_length < 8 || password_length > 63) return ESP_ERR_INVALID_ARG;
    auto error = nvs_flash_init();
    if (error != ESP_OK) return error;
    error = esp_netif_init();
    if (error != ESP_OK) return error;
    error = esp_event_loop_create_default();
    if (error != ESP_OK && error != ESP_ERR_INVALID_STATE) return error;
    if (!esp_netif_create_default_wifi_ap()) return ESP_ERR_NO_MEM;
    wifi_init_config_t wifi_init = WIFI_INIT_CONFIG_DEFAULT();
    wifi_init.nvs_enable = false;
    error = esp_wifi_init(&wifi_init);
    if (error != ESP_OK) return error;
    error = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (error != ESP_OK) return error;
    wifi_config_t wifi{};
    std::memcpy(wifi.ap.ssid, CONFIG_CAR_WIFI_SSID, ssid_length);
    std::memcpy(wifi.ap.password, CONFIG_CAR_WIFI_PASSWORD, password_length);
    wifi.ap.ssid_len = ssid_length;
    wifi.ap.channel = CONFIG_CAR_WIFI_CHANNEL;
    wifi.ap.max_connection = 2;
    wifi.ap.authmode = WIFI_AUTH_WPA2_PSK;
    error = esp_wifi_set_mode(WIFI_MODE_AP);
    if (error == ESP_OK) error = esp_wifi_set_config(WIFI_IF_AP, &wifi);
    if (error == ESP_OK) error = esp_wifi_start();
    if (error != ESP_OK) return error;

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.task_priority = 3;
    config.core_id = 1;
    config.stack_size = 8192;
    config.max_uri_handlers = 10;
    config.max_open_sockets = 5;
    config.lru_purge_enable = true;
    config.recv_wait_timeout = 3;
    config.send_wait_timeout = 5;
    config.uri_match_fn = httpd_uri_match_wildcard;
    error = httpd_start(&http_, &config);
    if (error != ESP_OK) { esp_wifi_stop(); return error; }
    struct Route { const char* path; httpd_method_t method; esp_err_t (*handler)(httpd_req_t*); };
    const Route routes[] = {{"/", HTTP_GET, page}, {"/api/status", HTTP_GET, status},
        {"/api/tuning", HTTP_GET, get_tuning}, {"/api/tuning", HTTP_POST, set_tuning},
        {"/api/tuning/reset", HTTP_POST, reset_tuning}, {"/api/logs", HTTP_GET, logs},
        {"/logs/*", HTTP_GET, download}, {"/logs/*", HTTP_DELETE, delete_session}};
    for (const auto& route : routes) {
        httpd_uri_t handler{};
        handler.uri = route.path; handler.method = route.method; handler.handler = route.handler; handler.user_ctx = this;
        error = httpd_register_uri_handler(http_, &handler);
        if (error != ESP_OK) { httpd_stop(http_); http_ = nullptr; esp_wifi_stop(); return error; }
    }
    ESP_LOGI("web", "AP %s; open http://192.168.4.1", CONFIG_CAR_WIFI_SSID);
    return ESP_OK;
}

void Server::monitor_cpu() {
    const std::array<TaskHandle_t, 4> tasks{
        xTaskGetHandle("wifi"), xTaskGetHandle("tiT"), xTaskGetHandle("httpd"), task_};
    uint64_t previous_runtime = 0;
    int64_t previous_time = 0;
    uint32_t previous_download = 0;
    while (true) {
        uint64_t runtime = 0;
        for (const auto task : tasks) {
            if (!task) continue;
            TaskStatus_t details{};
            vTaskGetInfo(task, &details, pdFALSE, eInvalid);
            runtime += details.ulRunTimeCounter;
        }
        const auto now = esp_timer_get_time();
        const auto generation = download_generation_.load();
        cpu_download_window_.store(downloading_.load() || generation != previous_download);
        if (previous_time && runtime >= previous_runtime)
            network_cpu_permille_.store(int(std::min<uint64_t>(2000, (runtime - previous_runtime) * 1000 / (now - previous_time))));
        previous_runtime = runtime; previous_time = now; previous_download = generation;
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

esp_err_t Server::page(httpd_req_t* r) {
    httpd_resp_set_type(r, "text/html; charset=utf-8");
    httpd_resp_set_hdr(r, "Content-Encoding", "gzip");
    httpd_resp_set_hdr(r, "Cache-Control", "no-cache");
    return httpd_resp_send(r, reinterpret_cast<const char*>(page_start), page_end - page_start);
}

esp_err_t Server::status(httpd_req_t* r) {
    auto& server = self(r);
    const auto state = server.logger_.status();
    char body[512];
    const int cpu = server.network_cpu_permille_.load();
    std::snprintf(body, sizeof(body), "{\"recording\":%s,\"filename\":\"%s\",\"bytes_written\":%" PRIu64
        ",\"synced_bytes\":%" PRIu64 ",\"records\":%" PRIu32 ",\"dropped_records\":%" PRIu32
        ",\"storage_errno\":%d,\"network_cpu_percent\":%.1f,\"download_window\":%s,\"uptime_s\":%" PRId64 "}",
        state.recording ? "true" : "false", state.filename, state.bytes_written, state.synced_bytes,
        state.records_written, state.dropped_records, state.error, cpu < 0 ? -1.0 : cpu / 10.0,
        server.cpu_download_window_.load() || server.downloading_.load() ? "true" : "false", esp_timer_get_time() / 1000000);
    return json(r, body);
}

esp_err_t Server::get_tuning(httpd_req_t* r) { return tuning_json(r, self(r).vehicle_.tuning()); }

esp_err_t Server::set_tuning(httpd_req_t* r) {
    char content_type[48];
    if (httpd_req_get_hdr_value_str(r, "Content-Type", content_type, sizeof(content_type)) != ESP_OK ||
        std::strncmp(content_type, "application/json", 16) != 0 || !r->content_len || r->content_len > 512) {
        httpd_resp_send_err(r, HTTPD_400_BAD_REQUEST, "Expected a small JSON tuning object");
        return ESP_FAIL;
    }
    char body[513]; size_t read = 0;
    while (read < r->content_len) {
        const auto count = httpd_req_recv(r, body + read, r->content_len - read);
        if (count <= 0) { httpd_resp_send_err(r, HTTPD_408_REQ_TIMEOUT, "Incomplete tuning request"); return ESP_FAIL; }
        read += size_t(count);
    }
    body[read] = 0;
    std::unique_ptr<cJSON, decltype(&cJSON_Delete)> object(cJSON_ParseWithLengthOpts(body, read + 1, nullptr, true), cJSON_Delete);
    constexpr const char* names[] = {"max_yaw_deg_s", "p", "i", "d", "d_cutoff_hz", "integral_limit"};
    std::array<float, 6> values{};
    bool valid = object && cJSON_IsObject(object.get()) && cJSON_GetArraySize(object.get()) == 6;
    for (size_t i = 0; valid && i < values.size(); ++i) {
        const auto* value = cJSON_GetObjectItemCaseSensitive(object.get(), names[i]);
        valid = cJSON_IsNumber(value) && std::isfinite(value->valuedouble);
        if (valid) values[i] = value->valuedouble;
    }
    auto config = self(r).vehicle_.tuning();
    config.maximum_yaw_rate_rads = values[0] * 0.017453292519943295f;
    config.proportional_gain = values[1]; config.integral_gain = values[2]; config.derivative_gain = values[3];
    config.derivative_cutoff_hz = values[4]; config.integral_limit = values[5];
    if (!valid || self(r).vehicle_.set_tuning(config) != ESP_OK)
        return httpd_resp_send_err(r, HTTPD_400_BAD_REQUEST, "Tuning values are missing or outside the allowed range");
    return tuning_json(r, config);
}

esp_err_t Server::reset_tuning(httpd_req_t* r) {
    if (r->content_len) { httpd_resp_send_err(r, HTTPD_400_BAD_REQUEST, "No request body expected"); return ESP_FAIL; }
    vehicle::YawRateConfig defaults;
    self(r).vehicle_.set_tuning(defaults);
    return tuning_json(r, defaults);
}

esp_err_t Server::logs(httpd_req_t* r) {
    auto* directory = opendir(recording::Logger::directory);
    if (!directory) return unavailable(r, "SD sessions are unavailable");
    const auto current = self(r).logger_.status();
    httpd_resp_set_type(r, "application/json"); httpd_resp_set_hdr(r, "Cache-Control", "no-store");
    esp_err_t error = httpd_resp_sendstr_chunk(r, "[");
    bool first = true;
    while (error == ESP_OK) {
        const auto* entry = readdir(directory);
        if (!entry) break;
        if (!recording::Logger::valid_filename(entry->d_name)) continue;
        char path[64]; std::snprintf(path, sizeof(path), "%s/%.12s", recording::Logger::directory, entry->d_name);
        struct stat info{};
        if (stat(path, &info) != 0 || !S_ISREG(info.st_mode)) continue;
        const bool active = std::strcmp(entry->d_name, current.filename) == 0;
        const auto bytes = active ? current.synced_bytes : uint64_t(info.st_size);
        char row[160];
        std::snprintf(row, sizeof(row), "%s{\"name\":\"%.12s\",\"bytes\":%" PRIu64 ",\"current\":%s}",
                      first ? "" : ",", entry->d_name, bytes, active ? "true" : "false");
        error = httpd_resp_sendstr_chunk(r, row); first = false;
    }
    closedir(directory);
    if (error == ESP_OK) error = httpd_resp_sendstr_chunk(r, "]");
    if (error == ESP_OK) error = httpd_resp_send_chunk(r, nullptr, 0);
    return error;
}

esp_err_t Server::download(httpd_req_t* r) {
    auto& server = self(r);
    const auto* name = r->uri + std::strlen("/logs/");
    if (!recording::Logger::valid_filename(name)) return httpd_resp_send_err(r, HTTPD_400_BAD_REQUEST, "Invalid session name");
    bool idle = false;
    if (!server.downloading_.compare_exchange_strong(idle, true))
        return unavailable(r, "A download is already running");
    auto job = std::unique_ptr<Download>(new (std::nothrow) Download{});
    if (!job) { server.downloading_.store(false); return httpd_resp_send_err(r, HTTPD_500_INTERNAL_SERVER_ERROR, "Not enough RAM"); }
    job->server = &server;
    std::strcpy(job->name, name);
    job->file = server.logger_.open_download(name, job->length);
    if (job->file < 0) {
        server.downloading_.store(false);
        return httpd_resp_send_err(r, HTTPD_404_NOT_FOUND, "Session unavailable");
    }
    const auto error = httpd_req_async_handler_begin(r, &job->request);
    if (error != ESP_OK) { close(job->file); server.downloading_.store(false); return error; }
    ++server.download_generation_;
    auto* worker_context = job.release();
    if (xTaskCreatePinnedToCore(download_task, "log_download", 8192, worker_context, 2, nullptr, 1) != pdPASS) {
        job.reset(worker_context);
        close(job->file);
        httpd_resp_send_err(job->request, HTTPD_500_INTERNAL_SERVER_ERROR, "Cannot start download");
        httpd_req_async_handler_complete(job->request);
        server.downloading_.store(false);
        return ESP_OK;
    }
    return ESP_OK;
}

esp_err_t Server::delete_session(httpd_req_t* r) {
    auto& server = self(r);
    const auto* name = r->uri + std::strlen("/logs/");
    if (r->content_len || !recording::Logger::valid_filename(name))
        return httpd_resp_send_err(r, HTTPD_400_BAD_REQUEST, "Invalid session name or unexpected body");
    if (server.downloading_.load()) return unavailable(r, "Wait for the download to finish before deleting a session");
    if (server.logger_.delete_session(name) != 0) {
        if (errno == EBUSY) {
            httpd_resp_set_status(r, "409 Conflict");
            return httpd_resp_sendstr(r, "The current recording cannot be deleted");
        }
        return httpd_resp_send_err(r, errno == ENOENT ? HTTPD_404_NOT_FOUND : HTTPD_500_INTERNAL_SERVER_ERROR,
                                  "Could not delete session");
    }
    return json(r, "{\"deleted\":true}");
}

void Server::download_task(void* context) {
    std::unique_ptr<Download> job(static_cast<Download*>(context));
    auto* request = job->request;
    const int socket = httpd_req_to_sockfd(request);
    char disposition[80], length[24];
    std::snprintf(disposition, sizeof(disposition), "attachment; filename=\"%s\"", job->name);
    std::snprintf(length, sizeof(length), "%" PRIu64, job->length);
    httpd_resp_set_type(request, "application/octet-stream");
    httpd_resp_set_hdr(request, "Content-Disposition", disposition);
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    httpd_resp_set_hdr(request, "X-Log-Bytes", length);
    std::array<char, 4096> buffer;
    auto remaining = job->length;
    esp_err_t error = ESP_OK;
    while (remaining && error == ESP_OK) {
        const auto count = read(job->file, buffer.data(), std::min<uint64_t>(remaining, buffer.size()));
        if (count < 0 && errno == EINTR) continue;
        if (count <= 0) { error = ESP_FAIL; break; }
        error = httpd_resp_send_chunk(request, buffer.data(), count);
        remaining -= uint64_t(count);
    }
    if (error == ESP_OK) error = httpd_resp_send_chunk(request, nullptr, 0);
    close(job->file);
    auto& server = *job->server;
    httpd_req_async_handler_complete(request);
    if (error != ESP_OK) httpd_sess_trigger_close(server.http_, socket);
    server.downloading_.store(false);
    job.reset();
    vTaskDelete(nullptr);
}
}
