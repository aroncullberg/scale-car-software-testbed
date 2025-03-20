#include "config_manager.h"

#include <algorithm>
#include <cstring>

#include "esp_log.h"


static const char* TAG = "ConfigManager";


ConfigManager& ConfigManager::instance() {
    static ConfigManager instance;
    return instance;
}

esp_err_t ConfigManager::init() {
    callback_mutex_ = xSemaphoreCreateMutex();

    ESP_LOGI(TAG, "Initializing ConfigManager");
    if (initialized_) {
        return ESP_OK;  // Already initialized
    }

    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NVS: %d", err);
        return err;
    }

    // Open NVS handle
    err = nvs_open("config", NVS_READWRITE, &nvs_handle_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %d", err);
        return err;
    }

    initialized_ = true;
    return ESP_OK;
}

void ConfigManager::registerCallback(const ConfigChangeCallback &callback) {
    if (xSemaphoreTake(callback_mutex_, portMAX_DELAY) == pdTRUE) {
        callbacks_.push_back(callback);
        xSemaphoreGive(callback_mutex_);
    }
}

void ConfigManager::unregisterCallback(ConfigChangeCallback *callback) {
    if (xSemaphoreTake(callback_mutex_, portMAX_DELAY) == pdTRUE) {
        // TODO: rewrite to not be a disgusting cpp hack
        callbacks_.erase(
            std::ranges::remove_if(callbacks_,
                                   [callback](const ConfigChangeCallback& cb) {
                                       return &cb == callback;
                                   }
            ).begin(),
            callbacks_.end()
        );
        xSemaphoreGive(callback_mutex_);
    }
}

void ConfigManager::notifyCallbacks() {
    if (xSemaphoreTake(callback_mutex_, portMAX_DELAY) == pdTRUE) {
        // Creating a local copy of callbacks to avoid holding the mutex
        // during potentially long-running callbacks
        std::vector<ConfigChangeCallback> callbacks_copy = callbacks_;
        xSemaphoreGive(callback_mutex_);

        for (const auto& callback : callbacks_copy) {
            callback();
        }
    }
}





bool ConfigManager::getBool(const char* key, bool default_value) {
    if (!initialized_) {
        ESP_LOGW(TAG, "ConfigManager not initialized, returning default for %s", key);
        return default_value;
    }

    // Check if there's a pending change
    auto it = pending_bool_changes_.find(key);
    if (it != pending_bool_changes_.end()) {
        return it->second;
    }

    // Read from NVS
    uint8_t value = default_value ? 1 : 0;
    esp_err_t err = nvs_get_u8(nvs_handle_, key, &value);

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return default_value;
    } else if (err != ESP_OK) {
        ESP_LOGW(TAG, "Error reading bool %s: %d", key, err);
        return default_value;
    }

    return value == 1;
}

int32_t ConfigManager::getInt(const char* key, int32_t default_value) {
    if (!initialized_) {
        ESP_LOGW(TAG, "ConfigManager not initialized, returning default for %s", key);
        return default_value;
    }

    // Check if there's a pending change
    auto it = pending_int_changes_.find(key);
    if (it != pending_int_changes_.end()) {
        return it->second;
    }

    // Read from NVS
    int32_t value;
    esp_err_t err = nvs_get_i32(nvs_handle_, key, &value);

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return default_value;
    } else if (err != ESP_OK) {
        ESP_LOGW(TAG, "Error reading int %s: %d", key, err);
        return default_value;
    }

    return value;
}

float ConfigManager::getFloat(const char* key, float default_value) {
    if (!initialized_) {
        ESP_LOGW(TAG, "ConfigManager not initialized, returning default for %s", key);
        return default_value;
    }

    // Check if there's a pending change
    auto it = pending_float_changes_.find(key);
    if (it != pending_float_changes_.end()) {
        return it->second;
    }

    // Store float as blob
    float value;
    size_t length = sizeof(float);
    esp_err_t err = nvs_get_blob(nvs_handle_, key, &value, &length);

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return default_value;
    } else if (err != ESP_OK || length != sizeof(float)) {
        ESP_LOGW(TAG, "Error reading float %s: %d", key, err);
        return default_value;
    }

    return value;
}

std::string ConfigManager::getString(const char* key, const std::string& default_value) {
    if (!initialized_) {
        ESP_LOGW(TAG, "ConfigManager not initialized, returning default for %s", key);
        return default_value;
    }

    // Check if there's a pending change
    auto it = pending_string_changes_.find(key);
    if (it != pending_string_changes_.end()) {
        return it->second;
    }

    // Read string length first
    size_t required_size;
    esp_err_t err = nvs_get_str(nvs_handle_, key, nullptr, &required_size);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return default_value;
    } else if (err != ESP_OK) {
        ESP_LOGW(TAG, "Error getting string length for %s: %d", key, err);
        return default_value;
    }

    // Allocate buffer and read string
    char* buffer = new char[required_size];
    err = nvs_get_str(nvs_handle_, key, buffer, &required_size);
    if (err != ESP_OK) {
        delete[] buffer;
        ESP_LOGW(TAG, "Error reading string %s: %d", key, err);
        return default_value;
    }

    std::string result(buffer);
    delete[] buffer;
    return result;
}

esp_err_t ConfigManager::setBool(const char* key, bool value) {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    pending_bool_changes_[key] = value;
    return ESP_OK;
}

esp_err_t ConfigManager::setInt(const char* key, int32_t value) {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    pending_int_changes_[key] = value;
    return ESP_OK;
}

esp_err_t ConfigManager::setFloat(const char* key, float value) {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    pending_float_changes_[key] = value;
    return ESP_OK;
}

esp_err_t ConfigManager::setString(const char* key, const std::string& value) {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    pending_string_changes_[key] = value;
    return ESP_OK;
}

esp_err_t ConfigManager::commit() {
    if (!initialized_) {
        ESP_LOGE(TAG, "ConfigManager not initialized for commit");
        return ESP_ERR_INVALID_STATE;
    }

    // ESP_LOGI(TAG, "Starting commit of pending changes");

    // Log the pending changes
    // ESP_LOGI(TAG, "Pending bool changes: %d", pending_bool_changes_.size());
    // ESP_LOGI(TAG, "Pending int changes: %d", pending_int_changes_.size());
    // ESP_LOGI(TAG, "Pending float changes: %d", pending_float_changes_.size());
    // ESP_LOGI(TAG, "Pending string changes: %d", pending_string_changes_.size());


    esp_err_t result = ESP_OK;

    // Commit boolean changes
    for (const auto& pair : pending_bool_changes_) {
        ESP_LOGI(TAG, "Committing bool %s = %d", pair.first.c_str(), pair.second);
        esp_err_t err = nvs_set_u8(nvs_handle_, pair.first.c_str(), pair.second ? 1 : 0);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set bool %s: %d", pair.first.c_str(), err);
            result = err;
        }
    }
    pending_bool_changes_.clear();

    // Commit integer changes
    for (const auto& pair : pending_int_changes_) {
        esp_err_t err = nvs_set_i32(nvs_handle_, pair.first.c_str(), pair.second);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set int %s: %d", pair.first.c_str(), err);
            result = err;
        }
    }
    pending_int_changes_.clear();

    // Commit float changes
    for (const auto& pair : pending_float_changes_) {
        esp_err_t err = nvs_set_blob(nvs_handle_, pair.first.c_str(), &pair.second, sizeof(float));
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set float %s: %d", pair.first.c_str(), err);
            result = err;
        }
    }
    pending_float_changes_.clear();

    // Commit string changes
    for (const auto& pair : pending_string_changes_) {
        esp_err_t err = nvs_set_str(nvs_handle_, pair.first.c_str(), pair.second.c_str());
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set string %s: %d", pair.first.c_str(), err);
            result = err;
        }
    }
    pending_string_changes_.clear();

    // Commit to NVS
    esp_err_t err = nvs_commit(nvs_handle_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit changes to NVS: %d", err);
        result = err;
    } else {
        // ESP_LOGI(TAG, "Successfully committed changes to NVS");
        notifyCallbacks();
    }


    return result;
}

esp_err_t ConfigManager::revert() {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    // Clear all pending changes
    pending_bool_changes_.clear();
    pending_int_changes_.clear();
    pending_float_changes_.clear();
    pending_string_changes_.clear();

    return ESP_OK;
}

std::vector<uint8_t> ConfigManager::parseChannelList(const std::string& channels_str) {
    std::vector<uint8_t> result;
    std::string str = channels_str;

    // Split by commas
    size_t pos = 0;
    std::string token;
    while ((pos = str.find(',')) != std::string::npos) {
        token = str.substr(0, pos);
        str.erase(0, pos + 1);

        // Trim whitespace
        token.erase(0, token.find_first_not_of(" \t"));
        token.erase(token.find_last_not_of(" \t") + 1);

        // Check if it's a range (contains dash)
        size_t dash_pos = token.find('-');
        if (dash_pos != std::string::npos) {
            // Parse range
            int start = std::stoi(token.substr(0, dash_pos));
            int end = std::stoi(token.substr(dash_pos + 1));

            for (int i = start; i <= end; i++) {
                if (i >= 0 && i <= 255) {  // Ensure valid channel number
                    result.push_back(static_cast<uint8_t>(i));
                }
            }
        } else if (!token.empty()) {
            // Parse single number
            int channel = std::stoi(token);
            if (channel >= 0 && channel <= 255) {
                result.push_back(static_cast<uint8_t>(channel));
            }
        }
    }

    // Process the last token after the last comma (or the entire string if no commas)
    if (!str.empty()) {
        // Trim whitespace
        str.erase(0, str.find_first_not_of(" \t"));
        str.erase(str.find_last_not_of(" \t") + 1);

        // Check if it's a range
        size_t dash_pos = str.find('-');
        if (dash_pos != std::string::npos) {
            int start = std::stoi(str.substr(0, dash_pos));
            int end = std::stoi(str.substr(dash_pos + 1));

            for (int i = start; i <= end; i++) {
                if (i >= 0 && i <= 255) {
                    result.push_back(static_cast<uint8_t>(i));
                }
            }
        } else {
            int channel = std::stoi(str);
            if (channel >= 0 && channel <= 255) {
                result.push_back(static_cast<uint8_t>(channel));
            }
        }
    }

    return result;
}

bool ConfigManager::keyExists(const char* key) {
    if (!initialized_) {
        return false;
    }

    // Check pending changes first
    if (pending_bool_changes_.contains(key) ||
        pending_int_changes_.contains(key) ||
        pending_float_changes_.contains(key) ||
        pending_string_changes_.contains(key)) {
        ESP_LOGI(TAG, "Key %s exists in pending changes", key);
        return true;
        }

    // The most reliable way to check if a key exists without knowing its type
    // is to try to get its length as a string (which doesn't modify any value)
    size_t length = 0;
    esp_err_t err = nvs_get_str(nvs_handle_, key, nullptr, &length);

    if (err == ESP_OK) {
        // Add debu statement that says it worked and where it worked
        ESP_LOGI(TAG, "Key %s exists in NVS as string", key);
        return true;
    }

    // If that fails, try as a blob
    err = nvs_get_blob(nvs_handle_, key, nullptr, &length);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Key %s exists in NVS as blob", key);
        return true;
    }

    // Finally, try specific data types
    int32_t int_val;
    err = nvs_get_i32(nvs_handle_, key, &int_val);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Key %s exists in NVS as int", key);
        return true;
    }

    uint8_t u8_val;
    err = nvs_get_u8(nvs_handle_, key, &u8_val);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Key %s exists in NVS as u8", key);
        return true;
    }

    ESP_LOGI(TAG, "Key %s does not exist in NVS (last err=%d)", key, err);
    return false;
}

std::vector<std::string> ConfigManager::listKeys(const char* prefix) {
    std::vector<std::string> keys;
    nvs_iterator_t it = nullptr;
    esp_err_t res = nvs_entry_find("nvs", "config", NVS_TYPE_ANY, &it);

    while (res == ESP_OK) {
        nvs_entry_info_t info;
        nvs_entry_info(it, &info);
        std::string key = info.key;

        if (!prefix || key.find(prefix) == 0) {
            keys.push_back(key);
        }

        res = nvs_entry_next(&it);
    }
    nvs_release_iterator(it);

    return keys;
}

esp_err_t ConfigManager::removeKey(const char* key) {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    pending_bool_changes_.erase(key);
    pending_int_changes_.erase(key);
    pending_float_changes_.erase(key);
    pending_string_changes_.erase(key);

    esp_err_t err = nvs_erase_key(nvs_handle_, key);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "Failed to remove key %s: %d", key, err);
        return err;
    }

    err = nvs_commit(nvs_handle_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit after key removal: %d", err);
        return err;
    }

    return ESP_OK;
}



ConfigManager::ValueType ConfigManager::getValueType(const char* key) {
    if (!initialized_) {
        return ValueType::UNKNOWN;
    }

    // Check pending changes first
    if (pending_bool_changes_.find(key) != pending_bool_changes_.end()) {
        return ValueType::BOOL;
    }
    if (pending_int_changes_.find(key) != pending_int_changes_.end()) {
        return ValueType::INT;
    }
    if (pending_float_changes_.find(key) != pending_float_changes_.end()) {
        return ValueType::FLOAT;
    }
    if (pending_string_changes_.find(key) != pending_string_changes_.end()) {
        return ValueType::STRING;
    }

    // Try to determine from NVS
    // Unfortunately, NVS doesn't provide a way to get the type directly
    // We have to try each type

    // Try as u8 (we use u8 for boolean values)
    uint8_t bool_value;
    esp_err_t err = nvs_get_u8(nvs_handle_, key, &bool_value);
    if (err == ESP_OK && (bool_value == 0 || bool_value == 1)) {
        return ValueType::BOOL;
    }

    // Try as i32
    int32_t int_value;
    err = nvs_get_i32(nvs_handle_, key, &int_value);
    if (err == ESP_OK) {
        return ValueType::INT;
    }

    // Try as blob (float)
    float float_value;
    size_t length = sizeof(float);
    err = nvs_get_blob(nvs_handle_, key, &float_value, &length);
    if (err == ESP_OK && length == sizeof(float)) {
        return ValueType::FLOAT;
    }

    // Try as string
    size_t required_size;
    err = nvs_get_str(nvs_handle_, key, nullptr, &required_size);
    if (err == ESP_OK) {
        return ValueType::STRING;
    }

    return ValueType::UNKNOWN;
}