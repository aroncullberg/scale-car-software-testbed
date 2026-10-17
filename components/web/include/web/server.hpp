#pragma once

#include <atomic>
#include "esp_http_server.h"
#include "recording/logger.hpp"
#include "vehicle/controller.hpp"

namespace web {
class Server {
public:
    Server(vehicle::Controller& vehicle, recording::Logger& logger) : vehicle_(vehicle), logger_(logger) {}
    esp_err_t start();
private:
    static void service_task(void* context);
    esp_err_t initialize();
    void monitor_cpu();
    static esp_err_t page(httpd_req_t* request);
    static esp_err_t status(httpd_req_t* request);
    static esp_err_t get_tuning(httpd_req_t* request);
    static esp_err_t set_tuning(httpd_req_t* request);
    static esp_err_t reset_tuning(httpd_req_t* request);
    static esp_err_t logs(httpd_req_t* request);
    static esp_err_t download(httpd_req_t* request);
    static esp_err_t delete_session(httpd_req_t* request);
    static void download_task(void* context);
    vehicle::Controller& vehicle_;
    recording::Logger& logger_;
    httpd_handle_t http_ = nullptr;
    TaskHandle_t task_ = nullptr;
    std::atomic<bool> downloading_{false};
    std::atomic<uint32_t> download_generation_{0};
    std::atomic<int> network_cpu_permille_{-1};
    std::atomic<bool> cpu_download_window_{false};
};
}
