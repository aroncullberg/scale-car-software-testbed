//
// Created by cullb on 2025-02-26.
//

#include "log_monitor.h"
#include  "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "esp_netif.h"
#include "esp_event.h"

#include <cstring>


static const char* TAG = "LogMonitor";

// To keep a kopy of original vprintf function
static vprintf_like_t original_vprintf_fn = nullptr;

int log_monitor_vprintf(const char *format, va_list args) {
    // Keep original behaviour
    int result = original_vprintf_fn(format, args);

    // Format the message for our queue
    char log_buffer[512];
    int length = vsnprintf(log_buffer, sizeof(log_buffer), format, args);
    if (length > 0) {
        LogMonitor::instance().queueLogMessage(log_buffer);
    }

    return result;
}

LogMonitor& LogMonitor::instance() {
    static LogMonitor instance;
    return instance;
}

esp_err_t LogMonitor::init(const Config& config) {
    if (is_running_) {
        return ESP_ERR_INVALID_STATE;
    }

    config_ = config;

    // Initialize NVS for WiFi storage
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Create log message queue
    ret = initQueue();
    if (ret != ESP_OK) return ret;

    // Install our custom log handler
    original_vprintf_fn = esp_log_set_vprintf(log_monitor_vprintf);
    if (!original_vprintf_fn) {
        ESP_LOGE(TAG, "Failed to set custom vprintf function");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t LogMonitor::start() {
    if (is_running_) {
        return ESP_ERR_INVALID_STATE;
    }

    // Setup WiFi
    esp_err_t ret = setupWiFi();
    if (ret != ESP_OK) return ret;

    // Create server task
    BaseType_t task_created = xTaskCreatePinnedToCore(
        serverTask,
        "log_server",
        config_.server_task_stack_size,
        this,
        config_.server_task_priority,
        &server_task_,
        0
    );

    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create server task");
        return ESP_FAIL;
    }

    is_running_ = true;
    return ESP_OK;
}

esp_err_t LogMonitor::stop() {
    if (!is_running_) {
        return ESP_ERR_INVALID_STATE;
    }

    // Clean up resources
    if (server_task_) {
        vTaskDelete(server_task_);
        server_task_ = nullptr;
    }

    // Close sockets
    if (server_socket_ >= 0) {
        close(server_socket_);
        server_socket_ = -1;
    }

    for (int client : client_sockets_) {
        if (client >= 0) {
            close(client);
        }
    }
    client_sockets_.clear();

    // Reset log handler
    if (original_vprintf_fn) {
        esp_log_set_vprintf(original_vprintf_fn);
        original_vprintf_fn = nullptr;
    }

    // Clean up queue
    if (log_queue_) {
        vQueueDelete(log_queue_);
        log_queue_ = nullptr;
    }

    is_running_ = false;
    return ESP_OK;
}

void LogMonitor::queueLogMessage(const char* message) {
    if (!is_running_ || !log_queue_) return;

    // Queue the message, but don't block if the queue is full
    xQueueSendToBack(log_queue_, message, 0);
}

esp_err_t LogMonitor::initQueue() {
    log_queue_ = xQueueCreate(config_.log_queue_size, 512);
    if (!log_queue_) {
        ESP_LOGE(TAG, "Failed to create log queue");
        return ESP_FAIL;
    }
    return ESP_OK;
}

LogMonitor::~LogMonitor() {
    stop();
}

esp_err_t LogMonitor::setupWiFi() {
    // Initialize the TCP/IP stack first
    ESP_ERROR_CHECK(esp_netif_init());

    // Create default event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Create the WiFi AP network interface
    esp_netif_t* ap_netif = esp_netif_create_default_wifi_ap();

    // Now initialize WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    // Configure WiFi in AP mode
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));

    // Configure the access point
    wifi_config_t ap_config = {};
    memcpy(ap_config.ap.ssid, config_.ap_ssid, strlen(config_.ap_ssid));
    ap_config.ap.ssid_len = strlen(config_.ap_ssid);
    memcpy(ap_config.ap.password, config_.ap_password, strlen(config_.ap_password));
    ap_config.ap.max_connection = 4;
    ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi AP started with SSID: %s", config_.ap_ssid);
    return ESP_OK;
}


void LogMonitor::serverTask(void* args) {
    LogMonitor* self = static_cast<LogMonitor*>(args);
    char log_buffer[512];

    // Create TCP socket
    int server_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (server_sock < 0) {
        ESP_LOGE(TAG, "Failed to create socket: %d", errno);
        vTaskDelete(NULL);
        return;
    }

    // Set socket options for better performance
    int opt = 1;
    setsockopt(server_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    setsockopt(server_sock, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt));

    int send_buf_size = 8192;
    setsockopt(server_sock, SOL_SOCKET, SO_SNDBUF, &send_buf_size, sizeof(send_buf_size));

    // Configure server address
    struct sockaddr_in server_addr = {};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(self->config_.tcp_port);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    // Bind the socket
    if (bind(server_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind socket: %d", errno);
        close(server_sock);
        vTaskDelete(NULL);
        return;
    }

    // Listen for connections
    if (listen(server_sock, 4) < 0) {
        ESP_LOGE(TAG, "Failed to listen: %d", errno);
        close(server_sock);
        vTaskDelete(NULL);
        return;
    }

    self->server_socket_ = server_sock;
    ESP_LOGI(TAG, "Log server listening on port %d", self->config_.tcp_port);

    // Set up select() structures
    fd_set read_set, error_set;
    struct timeval select_timeout;
    int max_fd = server_sock;

    while (true) {
        // Setup select structures
        FD_ZERO(&read_set);
        FD_ZERO(&error_set);

        // Always check server socket
        FD_SET(server_sock, &read_set);

        // Add all client sockets
        for (int client : self->client_sockets_) {
            FD_SET(client, &read_set);
            FD_SET(client, &error_set);
            max_fd = std::max(max_fd, client);
        }

        // Set timeout (faster polling)
        select_timeout.tv_sec = 0;
        select_timeout.tv_usec = 1000; // 1ms

        // Wait for activity
        int activity = select(max_fd + 1, &read_set, NULL, &error_set, &select_timeout);

        if (activity < 0) {
            ESP_LOGE(TAG, "select() error: %d", errno);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // Check for new connections on server socket
        if (FD_ISSET(server_sock, &read_set)) {
            struct sockaddr_in client_addr;
            socklen_t addr_len = sizeof(client_addr);
            int new_client = accept(server_sock, (struct sockaddr*)&client_addr, &addr_len);

            if (new_client >= 0) {
                // Set TCP_NODELAY for low latency
                int flag = 1;
                setsockopt(new_client, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

                // Add to client list
                self->client_sockets_.push_back(new_client);
                ESP_LOGI(TAG, "New client connected: %d", new_client);

                // Send welcome message
                const char* welcome = "=== ESP32-S3 RC Car Log Monitor ===\r\n";
                send(new_client, welcome, strlen(welcome), 0);
            }
        }

        // Check for closed connections
        for (auto it = self->client_sockets_.begin(); it != self->client_sockets_.end();) {
            if (FD_ISSET(*it, &error_set)) {
                ESP_LOGI(TAG, "Client disconnected: %d", *it);
                close(*it);
                it = self->client_sockets_.erase(it);
            } else {
                ++it;
            }
        }

        // Flush log queue as quickly as possible
        bool more_messages = true;
        int processed_count = 0;
        const int MAX_BATCH_SIZE = 20;  // Process up to 20 messages per cycle

        while (more_messages && processed_count < MAX_BATCH_SIZE) {
            if (xQueueReceive(self->log_queue_, log_buffer, 0) == pdTRUE) {
                processed_count++;

                // Send to all clients
                for (auto it = self->client_sockets_.begin(); it != self->client_sockets_.end();) {
                    int result = send(*it, log_buffer, strlen(log_buffer), 0);
                    if (result < 0) {
                        ESP_LOGI(TAG, "Client disconnected during send: %d", *it);
                        close(*it);
                        it = self->client_sockets_.erase(it);
                    } else {
                        ++it;
                    }
                }
            } else {
                more_messages = false;  // Queue is empty
            }
        }

        // If we processed the max batch, yield to allow other tasks to run
        if (processed_count >= MAX_BATCH_SIZE) {
            taskYIELD();
        }
    }
}

