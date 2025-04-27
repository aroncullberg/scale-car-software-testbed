//
// Created by cullb on 2025-03-07.
//

#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#pragma once

#include "esp_err.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <functional>
#include <string>
#include <map>
#include <vector>

class ConfigManager {
public:
    enum class ValueType {
        BOOL,
        INT,
        FLOAT,
        STRING,
        UNKNOWN
    };

    static ConfigManager& instance();

    using ConfigChangeCallback = std::function<void()>;

    void registerCallback(const ConfigChangeCallback &callback);
    void unregisterCallback(ConfigChangeCallback* callback);

    bool keyExists(const char* key);

    static std::vector<std::string> listKeys(const char *prefix);

    bool getBool(const char* key, bool default_value = false);

    float getFloat(const char* key, float default_value = 0.0f);

    int32_t getInt(const char* key, int32_t default_value = 0);

    ValueType getValueType(const char* key);

    esp_err_t removeKey(const char* key);

    esp_err_t init();
    esp_err_t setBool(const char* key, bool value);
    esp_err_t setInt(const char* key, int32_t value);
    esp_err_t setFloat(const char* key, float value);
    esp_err_t setString(const char* key, const std::string& value);
    esp_err_t commit();
    esp_err_t revert();

    std::string getString(const char* key, const std::string& default_value = "");
    std::vector<uint8_t> parseChannelList(const std::string& channels_str);

private:
    ConfigManager() = default;

    ConfigManager(const ConfigManager&) = delete;
    ConfigManager& operator=(const ConfigManager&) = delete;

    nvs_handle_t nvs_handle_{0};

    std::map<std::string, bool> pending_bool_changes_;
    std::map<std::string, int32_t> pending_int_changes_;
    std::map<std::string, float> pending_float_changes_;
    std::map<std::string, std::string> pending_string_changes_;

    std::vector<ConfigChangeCallback> callbacks_;
    SemaphoreHandle_t callback_mutex_ = nullptr;

    void notifyCallbacks();

    bool initialized_{false};
};

#endif //CONFIG_MANAGER_H
