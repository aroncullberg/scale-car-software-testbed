//
// Created by cullb on 2025-02-26.
//

#include "log_monitor.h"

#include <algorithm>

#include  "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "config_manager.h"

#include <cstring>


static const char *TAG = "LogMonitor";

// To keep a kopy of original vprintf function
static vprintf_like_t original_vprintf_fn = nullptr;

int log_monitor_vprintf(const char *format, va_list args) {
    // Keep original behaviour
    int result = original_vprintf_fn(format, args);

    char log_buffer[512];
    int length = vsnprintf(log_buffer, sizeof(log_buffer), format, args);
    if (length > 0) {
        LogMonitor::instance().queueLogMessage(log_buffer);
    }

    return result;
}

LogMonitor &LogMonitor::instance() {
    static LogMonitor instance;
    return instance;
}

esp_err_t LogMonitor::init(const Config &config) {
    if (is_running_) {
        return ESP_ERR_INVALID_STATE;
    }

    config_ = config;

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ret = initQueue();
    if (ret != ESP_OK) return ret;

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

    if (server_task_) {
        vTaskDelete(server_task_);
        server_task_ = nullptr;
    }

    if (config_server_socket_ >= 0) {
        close(config_server_socket_);
        config_server_socket_ = -1;
    }

    if (log_server_socket_ >= 0) {
        close(log_server_socket_);
        log_server_socket_ = -1;
    }

    for (const auto &[client, is_config]: client_sockets_) {
        if (client >= 0) {
            close(client);
        }
    }
    client_sockets_.clear();

    if (original_vprintf_fn) {
        esp_log_set_vprintf(original_vprintf_fn);
        original_vprintf_fn = nullptr;
    }

    if (log_queue_) {
        vQueueDelete(log_queue_);
        log_queue_ = nullptr;
    }

    is_running_ = false;
    return ESP_OK;
}

void LogMonitor::queueLogMessage(const char *message) {
    if (!is_running_ || !log_queue_) return;

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
    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));

    wifi_config_t ap_config = {};
    memcpy(ap_config.ap.ssid, config_.ap_ssid, strlen(config_.ap_ssid));
    ap_config.ap.ssid_len = strlen(config_.ap_ssid);
    memcpy(ap_config.ap.password, config_.ap_password, strlen(config_.ap_password));
    ap_config.ap.max_connection = 4;
    ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi AP started with SSID: %s", config_.ap_ssid);

    esp_netif_ip_info_t ip_info;
    esp_netif_get_ip_info(ap_netif, &ip_info);
    ESP_LOGI(TAG, "AP IP Address: " IPSTR, IP2STR(&ip_info.ip));

    wifi_mode_t mode;
    esp_wifi_get_mode(&mode);
    ESP_LOGI(TAG, "WiFi mode set: %d (AP=2)", mode);


    return ESP_OK;
}

void LogMonitor::serverTask(void *args) {
    LogMonitor *self = static_cast<LogMonitor *>(args);
    char log_buffer[512];
    char cmd_buffer[256];
    std::map<int, std::string> command_buffers;

    // Create config server socket
    int config_server = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (config_server < 0) {
        ESP_LOGE(TAG, "Failed to create config socket: %d", errno);
        vTaskDelete(NULL);
        return;
    }

    // Create log server socket
    int log_server = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (log_server < 0) {
        ESP_LOGE(TAG, "Failed to create log socket: %d", errno);
        close(config_server);
        vTaskDelete(NULL);
        return;
    }

    // Set socket options for both sockets
    int opt = 1;
    setsockopt(config_server, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    setsockopt(config_server, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt));

    setsockopt(log_server, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    setsockopt(log_server, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt));

    int send_buf_size = 8192;
    setsockopt(config_server, SOL_SOCKET, SO_SNDBUF, &send_buf_size, sizeof(send_buf_size));
    setsockopt(log_server, SOL_SOCKET, SO_SNDBUF, &send_buf_size, sizeof(send_buf_size));

    // Bind config server
    struct sockaddr_in config_addr = {};
    config_addr.sin_family = AF_INET;
    config_addr.sin_port = htons(self->config_.config_port);
    config_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(config_server, (struct sockaddr *) &config_addr, sizeof(config_addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind config socket: %d", errno);
        close(config_server);
        close(log_server);
        vTaskDelete(NULL);
        return;
    }

    // Bind log server
    struct sockaddr_in log_addr = {};
    log_addr.sin_family = AF_INET;
    log_addr.sin_port = htons(self->config_.log_port);
    log_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(log_server, (struct sockaddr *) &log_addr, sizeof(log_addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind log socket: %d", errno);
        close(config_server);
        close(log_server);
        vTaskDelete(NULL);
        return;
    }

    // Listen on both sockets
    if (listen(config_server, 4) < 0) {
        ESP_LOGE(TAG, "Failed to listen on config socket: %d", errno);
        close(config_server);
        close(log_server);
        vTaskDelete(NULL);
        return;
    }

    if (listen(log_server, 4) < 0) {
        ESP_LOGE(TAG, "Failed to listen on log socket: %d", errno);
        close(config_server);
        close(log_server);
        vTaskDelete(NULL);
        return;
    }

    self->config_server_socket_ = config_server;
    self->log_server_socket_ = log_server;

    ESP_LOGI(TAG, "Config server listening on port %d", self->config_.config_port);
    ESP_LOGI(TAG, "Log server listening on port %d", self->config_.log_port);

    fd_set read_set, error_set;
    struct timeval select_timeout;
    int max_fd = std::max(config_server, log_server);

    while (true) {
        FD_ZERO(&read_set);
        FD_ZERO(&error_set);

        // Add both servers to the read set
        FD_SET(config_server, &read_set);
        FD_SET(log_server, &read_set);

        // Add all clients to the sets
        for (const auto &[client, is_config]: self->client_sockets_) {
            FD_SET(client, &read_set);
            FD_SET(client, &error_set);
            max_fd = std::max(max_fd, client);
        }

        select_timeout.tv_sec = 0;
        select_timeout.tv_usec = 250000;
        // 250ms = 250,000 μs (how to type μ on keyboard? annoying to have to google mu everytime i want it)

        int activity = select(max_fd + 1, &read_set, NULL, &error_set, &select_timeout);

        if (activity < 0) {
            ESP_LOGE(TAG, "select() error: %d", errno);
            vTaskDelay(pdMS_TO_TICKS(250));
            continue;
        }

        // Check for new connections on config server
        if (FD_ISSET(config_server, &read_set)) {
            struct sockaddr_in client_addr;
            socklen_t addr_len = sizeof(client_addr);
            int new_client = accept(config_server, (struct sockaddr *) &client_addr, &addr_len);

            if (new_client >= 0) {
                int flag = 1;
                setsockopt(new_client, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

                self->client_sockets_[new_client] = true; // Mark as config client
                command_buffers[new_client] = "";
                ESP_LOGI(TAG, "New config client connected: %d", new_client);

                const char *welcome = "=== ESP32-S3 RC Car Configuration ===\r\n";
                send(new_client, welcome, strlen(welcome), 0);
                send(new_client, "Type 'help' for available commands.\r\n\n", 41, 0);
            } else {
                ESP_LOGE(TAG, "Failed to accept config connection: %s (%d)", strerror(errno), errno);
            }
        }

        // Check for new connections on log server
        if (FD_ISSET(log_server, &read_set)) {
            struct sockaddr_in client_addr;
            socklen_t addr_len = sizeof(client_addr);
            int new_client = accept(log_server, (struct sockaddr *) &client_addr, &addr_len);

            if (new_client >= 0) {
                int flag = 1;
                setsockopt(new_client, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

                self->client_sockets_[new_client] = false; // Mark as log client
                ESP_LOGI(TAG, "New log client connected: %d", new_client);

                const char *welcome = "=== ESP32-S3 RC Car Log Stream ===\r\n";
                send(new_client, welcome, strlen(welcome), 0);
                send(new_client, "Log stream started. Commands are disabled on this port.\r\n", 56, 0);
            } else {
                ESP_LOGE(TAG, "Failed to accept log connection: %s (%d)", strerror(errno), errno);
            }
        }

        // Check for client activity
        auto it = self->client_sockets_.begin();
        while (it != self->client_sockets_.end()) {
            int client = it->first;
            bool is_config_client = it->second;
            bool client_error = false;

            if (FD_ISSET(client, &error_set)) {
                client_error = true;
            } else if (FD_ISSET(client, &read_set)) {
                if (is_config_client) {
                    // Only process commands from config clients
                    memset(cmd_buffer, 0, sizeof(cmd_buffer));
                    int bytes = recv(client, cmd_buffer, sizeof(cmd_buffer) - 1, 0);

                    if (bytes > 0) {
                        command_buffers[client].append(cmd_buffer, bytes);

                        size_t pos;
                        while ((pos = command_buffers[client].find('\n')) != std::string::npos) {
                            std::string cmd = command_buffers[client].substr(0, pos);
                            if (!cmd.empty() && cmd.back() == '\r') {
                                cmd.pop_back();
                            }

                            if (!cmd.empty()) {
                                self->processCommand(cmd.c_str(), client);
                            }

                            command_buffers[client].erase(0, pos + 1);
                        }
                    } else if (bytes <= 0) {
                        client_error = true;
                    }
                } else {
                    // For log clients, just drain any input (ignore it)
                    char drain_buffer[256];
                    int bytes = recv(client, drain_buffer, sizeof(drain_buffer), 0);
                    if (bytes <= 0) {
                        client_error = true;
                    }
                }
            }

            if (client_error) {
                ESP_LOGI(TAG, "Client disconnected: %d (config: %d)", client, is_config_client);
                close(client);
                command_buffers.erase(client);
                it = self->client_sockets_.erase(it);
            } else {
                ++it;
            }
        }

        // Process log queue
        bool more_messages = true;
        int processed_count = 0;
        const int MAX_BATCH_SIZE = 20;

        while (more_messages && processed_count < MAX_BATCH_SIZE) {
            if (xQueueReceive(self->log_queue_, log_buffer, 0) == pdTRUE) {
                processed_count++;

                // Send only to log clients
                for (auto it = self->client_sockets_.begin(); it != self->client_sockets_.end();) {
                    int client = it->first;
                    bool is_config = it->second;

                    if (!is_config) {
                        // Only send logs to log clients
                        int result = send(client, log_buffer, strlen(log_buffer), 0);
                        if (result < 0) {
                            ESP_LOGI(TAG, "Log client disconnected during send: %d", client);
                            close(client);
                            command_buffers.erase(client);
                            it = self->client_sockets_.erase(it);
                            continue;
                        }
                    }
                    ++it;
                }
            } else {
                more_messages = false; // Queue empty
            }
        }

        if (processed_count >= MAX_BATCH_SIZE) {
            taskYIELD();
        }
    }
}


void LogMonitor::processCommand(const char *command_line, int client_socket) {
    char cmd[16] = {0};
    char module[32] = {0};
    char setting[32] = {0};
    char value[64] = {0};

    int args = sscanf(command_line, "%15s %31s %31s %63s", cmd, module, setting, value);

    if (args >= 1) {
        if (strcmp(cmd, "get") == 0 && args >= 3) {
            handleGetCommand(module, setting, client_socket);
        } else if (strcmp(cmd, "set") == 0 && args >= 4) {
            handleSetCommand(module, setting, value, client_socket);
        } else if (strcmp(cmd, "save") == 0) {
            esp_err_t err = ConfigManager::instance().commit();
            if (err == ESP_OK) {
                sendResponse("Configuration saved successfully", client_socket);
            } else {
                char error_msg[64];
                snprintf(error_msg, sizeof(error_msg), "Error saving configuration: %d", err);
                sendResponse(error_msg, client_socket);
            }
        } else if (strcmp(cmd, "remove") == 0 && args >= 3) {
            std::string key = std::string(module) + "/" + setting;
            esp_err_t err = ConfigManager::instance().removeKey(key.c_str());
            if (err == ESP_OK) {
                char response[128];
                snprintf(response, sizeof(response), "Removed key: %s", key.c_str());
                sendResponse(response, client_socket);
            } else {
                char response[128];
                snprintf(response, sizeof(response), "Error removing key %s: %d", key.c_str(), err);
                sendResponse(response, client_socket);
            }
        }
        // In processCommand function:
        else if (strcmp(cmd, "discard") == 0) {
            esp_err_t err = ConfigManager::instance().revert();
            if (err == ESP_OK) {
                sendResponse("All pending changes discarded", client_socket);
            } else {
                char error_msg[64];
                snprintf(error_msg, sizeof(error_msg), "Error discarding changes: %d", err);
                sendResponse(error_msg, client_socket);
            }
        } else if (strcmp(cmd, "reboot") == 0) {
            sendResponse("Rebooting system...", client_socket);
            vTaskDelay(pdMS_TO_TICKS(500));
            esp_restart();
        } else if (strcmp(cmd, "help") == 0) {
            handleHelpCommand(client_socket);
        } else if (strcmp(cmd, "list") == 0 && args >= 2) {
            handleListCommand(module, client_socket);
        } else {
            sendResponse("Unknown command. Type 'help' for available commands.", client_socket);
        }
    }
}

void LogMonitor::sendResponse(const char *response, int client_socket) {
    if (client_socket >= 0) {
        send(client_socket, response, strlen(response), 0);
        send(client_socket, "\r\n", 2, 0);
    }
}

void LogMonitor::handleGetCommand(const char *module, const char *setting, int client_socket) {
    std::string key = std::string(module) + "/" + setting;

    // ESP_LOGI(TAG, "Getting value for %s", key.c_str());

    // Try to get the value as different types, starting with most common
    // We'll check if the key exists first
    bool exists = ConfigManager::instance().keyExists(key.c_str());

    if (!exists) {
        // Log warning
        ESP_LOGW(TAG, "Key %s not found", key.c_str());
        char response[128];
        snprintf(response, sizeof(response), "Error: Setting '%s/%s' not found", module, setting);
        sendResponse(response, client_socket);
        return;
    }

    // Try to determine the type and get the value
    ConfigManager::ValueType type = ConfigManager::instance().getValueType(key.c_str());

    char response[128];
    switch (type) {
        case ConfigManager::ValueType::BOOL:
            snprintf(response, sizeof(response), "%s/%s = %s",
                     module, setting,
                     ConfigManager::instance().getBool(key.c_str()) ? "true" : "false");
            break;

        case ConfigManager::ValueType::INT:
            snprintf(response, sizeof(response), "%s/%s = %ld",
                     module, setting,
                     ConfigManager::instance().getInt(key.c_str()));
            break;

        case ConfigManager::ValueType::FLOAT:
            snprintf(response, sizeof(response), "%s/%s = %.2f",
                     module, setting,
                     ConfigManager::instance().getFloat(key.c_str()));
            break;

        case ConfigManager::ValueType::STRING:
            snprintf(response, sizeof(response), "%s/%s = \"%s\"",
                     module, setting,
                     ConfigManager::instance().getString(key.c_str()).c_str());
            break;

        default:
            snprintf(response, sizeof(response), "Error: Cannot determine type of '%s/%s'",
                     module, setting);
            break;
    }

    sendResponse(response, client_socket);
}

void LogMonitor::handleSetCommand(const char *module, const char *setting, const char *value, int client_socket) {
    std::string key = std::string(module) + "/" + setting;
    esp_err_t err = ESP_OK;

    // Try to determine value type and set accordingly
    if (strcmp(value, "true") == 0 || strcmp(value, "false") == 0) {
        // Boolean value
        bool bool_value = (strcmp(value, "true") == 0);
        err = ConfigManager::instance().setBool(key.c_str(), bool_value);
    } else if (strchr(value, '.') != nullptr) {
        // Likely a float if it contains a decimal point
        float float_value;
        if (sscanf(value, "%f", &float_value) == 1) {
            err = ConfigManager::instance().setFloat(key.c_str(), float_value);
        } else {
            err = ESP_ERR_INVALID_ARG;
        }
    } else if (value[0] == '"' && value[strlen(value) - 1] == '"') {
        // String value (quoted)
        std::string string_value(value + 1, strlen(value) - 2); // Remove quotes
        err = ConfigManager::instance().setString(key.c_str(), string_value);
    } else {
        // Try to parse as integer
        int int_value;
        if (sscanf(value, "%d", &int_value) == 1) {
            err = ConfigManager::instance().setInt(key.c_str(), int_value);
        } else {
            // If not an integer, treat as string without quotes
            err = ConfigManager::instance().setString(key.c_str(), value);
        }
    }

    if (err == ESP_OK) {
        char response[128];
        snprintf(response, sizeof(response), "Set %s/%s = %s", module, setting, value);
        sendResponse(response, client_socket);
        sendResponse("Note: Use 'save' command to make changes permanent", client_socket);
    } else {
        char response[128];
        snprintf(response, sizeof(response), "Error setting %s/%s: %d", module, setting, err);
        sendResponse(response, client_socket);
    }
}

void LogMonitor::handleHelpCommand(int client_socket) {
    sendResponse("Available commands:", client_socket);
    sendResponse("  get <module> <setting>     - Get a configuration value", client_socket);
    sendResponse("  set <module> <setting> <value> - Set a configuration value", client_socket);
    sendResponse("  remove <module> <setting>  - Remove a configuration key", client_socket);
    sendResponse("  save                       - Save all configuration changes", client_socket);
    sendResponse("  discard                    - Discard all pending changes", client_socket);
    sendResponse("  reboot                     - Restart the system", client_socket);
    sendResponse("  help                       - Show this help", client_socket);
    sendResponse("  list <module>              - List all settings for a module", client_socket);
    sendResponse("  list *                     - List all settings", client_socket);
    sendResponse("", client_socket);
    sendResponse("Modules:", client_socket);
    sendResponse("  system  - System settings", client_socket);
    sendResponse("  imu     - IMU sensor settings", client_socket);
    sendResponse("  gps     - GPS sensor settings", client_socket);
    sendResponse("  sbus    - SBUS settings", client_socket);
    sendResponse("  log     - Logging settings", client_socket);
}

void LogMonitor::handleListCommand(const char *module, int client_socket) {
    std::string prefix;
    if (strcmp(module, "*") != 0) {
        // If not wildcard, use as prefix
        prefix = std::string(module) + "/";
    }

    // Get keys from ConfigManager
    std::vector<std::string> keys = ConfigManager::instance().listKeys(
        prefix.empty() ? nullptr : prefix.c_str());

    if (keys.empty()) {
        char response[128];
        snprintf(response, sizeof(response), "No settings found for '%s'",
                 prefix.empty() ? "any module" : module);
        sendResponse(response, client_socket);
        return;
    }

    // Group keys by module prefix (the part before '/')
    std::map<std::string, std::vector<std::string> > grouped;
    for (const auto &key: keys) {
        size_t pos = key.find('/');
        std::string group = (pos != std::string::npos) ? key.substr(0, pos) : key;
        grouped[group].push_back(key);
    }

    // Iterate over each module group in sorted order.
    for (const auto &groupPair: grouped) {
        std::string header = "Settings for " + groupPair.first + ":";
        sendResponse(header.c_str(), client_socket);

        // Optionally, sort the keys within each group.
        auto settings = groupPair.second;
        std::sort(settings.begin(), settings.end());

        // Process each key within the group.
        for (const auto &key: settings) {
            ConfigManager::ValueType type = ConfigManager::instance().getValueType(key.c_str());
            char valueStr[128] = {0};
            const char *typeStr = "";
            switch (type) {
                case ConfigManager::ValueType::BOOL: {
                    bool b = ConfigManager::instance().getBool(key.c_str());
                    snprintf(valueStr, sizeof(valueStr), "%s = %s", key.c_str(), b ? "true" : "false");
                    typeStr = "[bool]";
                    break;
                }
                case ConfigManager::ValueType::INT: {
                    int32_t i = ConfigManager::instance().getInt(key.c_str());
                    snprintf(valueStr, sizeof(valueStr), "%s = %ld", key.c_str(), i);
                    typeStr = "[int]";
                    break;
                }
                case ConfigManager::ValueType::FLOAT: {
                    float f = ConfigManager::instance().getFloat(key.c_str());
                    snprintf(valueStr, sizeof(valueStr), "%s = %.2f", key.c_str(), f);
                    typeStr = "[float]";
                    break;
                }
                case ConfigManager::ValueType::STRING: {
                    std::string s = ConfigManager::instance().getString(key.c_str());
                    snprintf(valueStr, sizeof(valueStr), "%s = \"%s\"", key.c_str(), s.c_str());
                    typeStr = "[string]";
                    break;
                }
                default:
                    snprintf(valueStr, sizeof(valueStr), "%s = <unknown type>", key.c_str());
                    typeStr = "";
                    break;
            }

            // Align the type string to a fixed column (e.g., column 40)
            char lineBuffer[160] = {0};
            snprintf(lineBuffer, sizeof(lineBuffer), "  %-40s%s", valueStr, typeStr);
            sendResponse(lineBuffer, client_socket);
        }
        // Add an empty line between groups
        sendResponse("", client_socket);
    }
}
