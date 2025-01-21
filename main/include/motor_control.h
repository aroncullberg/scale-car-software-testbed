#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "DShotRMT.h"
#include "esp_log.h"

static const char *TAG = "motor_control";

// Motor telemetry data structure
struct MotorTelemetry {
    uint32_t rpm;           // Actual RPM
    float voltage;          // Voltage
    uint8_t temperature;    // Temperature
    float current;          // Current
    float telemetry_success_rate;
};

// Motor control settings
struct MotorControl {
    bool is_armed;
    uint16_t current_throttle;
    MotorTelemetry telemetry;
};

class MotorManager {
public:
    MotorManager(gpio_num_t dshot_pin) : dshot_pin(dshot_pin) {}
    
    bool begin() {
        // Initialize DShot with bidirectional mode
        dshot = new DShotRMT(dshot_pin);
        dshot->begin(DSHOT300, ENABLE_BIDIRECTION, 14); // DShot300, Bidirectional, 14 poles
        
        // Initialize control structure
        control.is_armed = false;
        control.current_throttle = 0;
        
        ESP_LOGI(TAG, "Motor control initialized on pin %d", dshot_pin);
        return true;
    }
    
    bool update_throttle(uint16_t sbus_value) {
        if (!dshot) return false;
        
        // Map SBUS value (0-2047) to DShot range (48-2047)
        // We use 48 as minimum as per DShot spec
        uint16_t dshot_value;
        
        if (sbus_value < 48) { // Below minimum DShot value
            dshot_value = 0;  // Motor stop
        } else {
            dshot_value = sbus_value;
        }
        
        control.current_throttle = dshot_value;
        
        // Send DShot command
        return (dshot->send_dshot_value(dshot_value, true, ENABLE_TELEMETRIC) == SEND_SUCCESS);
    }
    
    bool update_telemetry() {
        if (!dshot) return false;
        
        uint32_t value;
        extended_telem_type_t packet_type;
        
        dshot_get_packet_exit_mode_t result = dshot->get_dshot_packet(&value, &packet_type);
        
        if (result == DECODE_SUCCESS) {
            switch (packet_type) {
                case TELEM_TYPE_ERPM:
                    control.telemetry.rpm = value;  // Already converted to RPM
                    break;
                case TELEM_TYPE_TEMPRATURE:
                    control.telemetry.temperature = value;
                    break;
                case TELEM_TYPE_VOLTAGE:
                    control.telemetry.voltage = dshot->convert_packet_to_volts(value);
                    break;
                case TELEM_TYPE_CURRENT:
                    control.telemetry.current = value;  // Need conversion factor
                    break;
                default:
                    // Handle other cases or log a warning/error
                    // TODO: Lol
                    break;
            }
            
            // Update success rate
            control.telemetry.telemetry_success_rate = dshot->get_telem_success_rate();
            return true;
        }
        
        return false;
    }
    
    MotorControl* get_control() {
        return &control;
    }

private:
    gpio_num_t dshot_pin;
    DShotRMT* dshot = nullptr;
    MotorControl control;
};

#endif // MOTOR_CONTROL_H