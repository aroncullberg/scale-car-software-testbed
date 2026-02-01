#pragma once

#include <cstdint>
#include "esp_err.h"

namespace waypoint_storage {

// Maximum waypoints per preset
constexpr uint8_t MAX_WAYPOINTS = 10;

// Waypoint structure (matches nav::Location format)
struct Waypoint {
    int32_t lat_e7;  // Latitude × 10^7
    int32_t lon_e7;  // Longitude × 10^7
};

// Preset containing up to 10 waypoints
struct WaypointPreset {
    Waypoint waypoints[MAX_WAYPOINTS];
    uint8_t count;  // Number of valid waypoints (0-10)
};

// Preset identifier
enum class PresetId : uint8_t {
    A = 0,
    B = 1
};

/**
 * @brief Initialize NVS for waypoint storage
 * @return ESP_OK on success
 */
esp_err_t init();

/**
 * @brief Load preset from NVS
 * @param id Preset identifier (A or B)
 * @return WaypointPreset with count=0 if invalid/empty
 */
WaypointPreset load_preset(PresetId id);

/**
 * @brief Save preset to NVS
 * @param id Preset identifier (A or B)
 * @param preset Preset data to save
 * @return ESP_OK on success
 */
esp_err_t save_preset(PresetId id, const WaypointPreset& preset);

/**
 * @brief Add waypoint to preset
 * @param id Preset identifier (A or B)
 * @param wp Waypoint to add
 * @return true if added successfully, false if preset already has 10 waypoints
 */
bool add_waypoint(PresetId id, const Waypoint& wp);

/**
 * @brief Clear all waypoints from preset
 * @param id Preset identifier (A or B)
 * @return ESP_OK on success
 */
esp_err_t clear_preset(PresetId id);

} // namespace waypoint_storage
