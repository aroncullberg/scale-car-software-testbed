#pragma once


namespace sensor {


/// @brief Common quality metrics for all sensors
struct SensorQuality {
    bool valid_data{false};        // Indicates if data is valid
    uint32_t error_count{0};       // Cumulative error counter
    float update_rate_hz{0.0f};    // Current update rate in Hz
};

enum class SbusChannelType : uint8_t {
    SYMMETRIC = 0,
    UNIPOLAR = 1,
    BINARY = 2,
    TRISTATE = 3,
    NONE = 8,
};


enum class SbusChannel : uint8_t {
    THROTTLE = 0,
    STEERING = 1,
    AUX1 = 2,
    AUX2 = 3,
    AUX3 = 4,
    AUX4 = 5,
    AUX5 = 6,
    AUX6 = 7,
    AUX7 = 8,
    AUX8 = 9,
    AUX9 = 10,
    AUX10 = 11,
    AUX11 = 12,
    AUX12 = 13,
    AUX13 = 14,
    AUX14 = 15,
    CHANNEL_COUNT = 16
};

struct SbusData {
    uint16_t channels[16]{1500};
    struct {
        uint8_t frame_loss_percent{0};
        uint32_t error_count{0};
        float frame_interval_ms{0.0f};
        bool valid_signal{false};
    } quality;
    SbusData() = default;
};


struct GpsData {
    int32_t latitude{0};                    //  Degrees * 10^7
    int32_t longitude{0};                   //  Degrees * 10^7
    int32_t altitude_mm{0};                 //  Altitude in milimeters

    uint32_t speed_mmps{0};                 //  Milimeters per second
    uint32_t ground_course{0};              //  Course in degree * 10^2 (0-36000)
    bool speed_valid{false};   
    
    struct {
        uint8_t fix_type{0};      
        uint8_t satellites{0};     
        uint8_t satellites_used{0};
        uint16_t hdop{0};         
    } quality;

    union {
        uint8_t flags{0};
        struct {
            uint8_t valid_fix : 1;    
            uint8_t north_south : 1;  
            uint8_t east_west : 1;    
            uint8_t reserved : 5;     
        } bits;
    } status;

    // NOTE: UTC-time
    struct {
        uint8_t hours{0};
        uint8_t minutes{0};
        uint8_t seconds{0};
        uint16_t milliseconds{0};
    } time;

    GpsData() = default;
};

struct ImuData {
    // Raw accelerometer data (in g's)
    float accel_x{0.0f};
    float accel_y{0.0f};
    float accel_z{0.0f};
    
    // Raw gyroscope data (in deg/s)
    float gyro_x{0.0f};
    float gyro_y{0.0f};
    float gyro_z{0.0f};

    float accel_cal_x{0.0f};
    float accel_cal_y{0.0f};
    float accel_cal_z{0.0f};

    float gyro_cal_x{0.0f};
    float gyro_cal_y{0.0f};
    float gyro_cal_z{0.0f};
    
    // Quaternion orientation
    float quat_w{1.0f};  // Real component
    float quat_x{0.0f};  // i component
    float quat_y{0.0f};  // j component
    float quat_z{0.0f};  // k component
    uint16_t quat_accuracy{0}; // DMP accuracy indicator

    // game roation vector (for smooth rotations)
    float game_quat_w{1.0f};
    float game_quat_x{0.0f};
    float game_quat_y{0.0f};
    float game_quat_z{0.0f};

    // grav vector
    float linear_accel_x{0.0f};
    float linear_accel_y{0.0f};
    float linear_accel_z{0.0f};

    // Quality metrics
    struct {
        bool valid_data{false};      // Indicates if data is valid
        uint32_t error_count{0};     // Cumulative error counter
        float update_rate_hz{0.0f};  // Current update rate
        float accel_accuracy{0.0f};
        float gyro_accuracy{0.0f};
    } quality;

    // Default constructor for zero-initialization
    ImuData() = default;
};



} // Namespace sensor