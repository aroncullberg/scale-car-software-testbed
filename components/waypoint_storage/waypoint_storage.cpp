#include "waypoint_storage.h"

#include <cstring>
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

static const char* TAG = "WaypointStorage";

namespace waypoint_storage {

// NVS namespace and keys
static constexpr const char* NVS_NAMESPACE = "waypoints";
static constexpr const char* NVS_KEY_PRESET_A = "preset_a";
static constexpr const char* NVS_KEY_PRESET_B = "preset_b";

// Helper to get NVS key for preset ID
static const char* get_preset_key(PresetId id) {
    return (id == PresetId::A) ? NVS_KEY_PRESET_A : NVS_KEY_PRESET_B;
}

esp_err_t init() {
    // Initialize NVS (may already be initialized by other components)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated or version mismatch
        ESP_LOGW(TAG, "NVS erase required, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NVS: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Waypoint storage initialized");
    return ESP_OK;
}

WaypointPreset load_preset(PresetId id) {
    WaypointPreset preset{};
    preset.count = 0;  // Default to empty

    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        if (ret == ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGD(TAG, "NVS namespace not found, preset %c is empty",
                     (id == PresetId::A) ? 'A' : 'B');
        } else {
            ESP_LOGW(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        }
        return preset;
    }

    // Try to read preset
    size_t required_size = sizeof(WaypointPreset);
    ret = nvs_get_blob(nvs_handle, get_preset_key(id), &preset, &required_size);

    if (ret == ESP_OK) {
        // Validate loaded data
        if (preset.count > MAX_WAYPOINTS) {
            ESP_LOGW(TAG, "Invalid preset count %d, resetting to empty", preset.count);
            preset.count = 0;
        } else {
            ESP_LOGI(TAG, "Loaded preset %c with %d waypoints",
                     (id == PresetId::A) ? 'A' : 'B', preset.count);
        }
    } else if (ret == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGD(TAG, "Preset %c not found in NVS, starting empty",
                 (id == PresetId::A) ? 'A' : 'B');
    } else {
        ESP_LOGW(TAG, "Failed to read preset %c: %s",
                 (id == PresetId::A) ? 'A' : 'B', esp_err_to_name(ret));
    }

    nvs_close(nvs_handle);
    return preset;
}

esp_err_t save_preset(PresetId id, const WaypointPreset& preset) {
    // Validate preset
    if (preset.count > MAX_WAYPOINTS) {
        ESP_LOGE(TAG, "Invalid preset count %d (max %d)", preset.count, MAX_WAYPOINTS);
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for write: %s", esp_err_to_name(ret));
        return ret;
    }

    // Write preset as blob
    ret = nvs_set_blob(nvs_handle, get_preset_key(id), &preset, sizeof(WaypointPreset));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write preset %c: %s",
                 (id == PresetId::A) ? 'A' : 'B', esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }

    // Commit changes
    ret = nvs_commit(nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Saved preset %c with %d waypoints",
                 (id == PresetId::A) ? 'A' : 'B', preset.count);
    }

    nvs_close(nvs_handle);
    return ret;
}

bool add_waypoint(PresetId id, const Waypoint& wp) {
    // Load current preset
    WaypointPreset preset = load_preset(id);

    // Check if preset is full
    if (preset.count >= MAX_WAYPOINTS) {
        ESP_LOGW(TAG, "Preset %c is full (%d waypoints), cannot add",
                 (id == PresetId::A) ? 'A' : 'B', MAX_WAYPOINTS);
        return false;
    }

    // Add waypoint
    preset.waypoints[preset.count] = wp;
    preset.count++;

    // Save updated preset
    esp_err_t ret = save_preset(id, preset);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save preset after adding waypoint");
        return false;
    }

    ESP_LOGI(TAG, "Added waypoint to preset %c (now %d waypoints)",
             (id == PresetId::A) ? 'A' : 'B', preset.count);
    return true;
}

esp_err_t clear_preset(PresetId id) {
    WaypointPreset empty_preset{};
    empty_preset.count = 0;

    esp_err_t ret = save_preset(id, empty_preset);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Cleared preset %c", (id == PresetId::A) ? 'A' : 'B');
    }
    return ret;
}

} // namespace waypoint_storage
