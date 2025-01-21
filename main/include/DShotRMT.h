#ifndef _DSHOTRMT_h
#define _DSHOTRMT_h

#include <string>
#include "esp_log.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_attr.h"

// Defines the library version
static const char* DSHOT_LIB_VERSION = "0.4.0";

// Constants related to the DShot protocol
#define DSHOT_CLK_DIVIDER 8    // Slow down RMT clock to 0.1 microseconds / 100 nanoseconds per cycle
#define DSHOT_PACKET_LENGTH 17  // Last pack is the pause
#define DSHOT_THROTTLE_MIN 48   // The smallest value considered to be explicitly a "throttle" value
#define DSHOT_THROTTLE_MAX 2047 // The largest value that can be passed back in a single packet
#define DSHOT_NULL_PACKET 0b0000000000000000
#define DSHOT_PAUSE 21          // 21-bit is recommended
#define DSHOT_PAUSE_BIT 16
#define F_CPU_RMT 80000000
#define RMT_CYCLES_PER_SEC (F_CPU_RMT / DSHOT_CLK_DIVIDER)

#define RX_SYMBOL_MAX 11

// The official DShot Commands
typedef enum dshot_cmd_e
{
    DSHOT_CMD_MOTOR_STOP = 0,
    DSHOT_CMD_BEACON1,
    DSHOT_CMD_BEACON2,
    DSHOT_CMD_BEACON3,
    DSHOT_CMD_BEACON4,
    DSHOT_CMD_BEACON5,
    DSHOT_CMD_ESC_INFO,
    DSHOT_CMD_SPIN_DIRECTION_1,
    DSHOT_CMD_SPIN_DIRECTION_2,
    DSHOT_CMD_3D_MODE_OFF,
    DSHOT_CMD_3D_MODE_ON,
    DSHOT_CMD_SETTINGS_REQUEST,
    DSHOT_CMD_SAVE_SETTINGS,
    DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE,
    DSHOT_CMD_EXTENDED_TELEMETRY_DISABLE,
    DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20,
    DSHOT_CMD_SPIN_DIRECTION_REVERSED = 21,
    DSHOT_CMD_LED0_ON,
    DSHOT_CMD_LED1_ON,
    DSHOT_CMD_LED2_ON,
    DSHOT_CMD_LED3_ON,
    DSHOT_CMD_LED0_OFF,
    DSHOT_CMD_LED1_OFF,
    DSHOT_CMD_LED2_OFF,
    DSHOT_CMD_LED3_OFF,
    DSHOT_CMD_AUDIO_STREAM_MODE_ON_OFF = 30,
    DSHOT_CMD_SILENT_MODE_ON_OFF = 31,
    DSHOT_CMD_SIGNAL_LINE_TELEMETRY_DISABLE,
    DSHOT_CMD_SIGNAL_LINE_TELEMETRY_ENABLE,
    DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY,
    DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_PERIOD_TELEMETRY,
    DSHOT_CMD_36,
    DSHOT_CMD_37,
    DSHOT_CMD_38,
    DSHOT_CMD_39,
    DSHOT_CMD_40,
    DSHOT_CMD_41,
    DSHOT_CMD_SIGNAL_LINE_TEMPERATURE_TELEMETRY,
    DSHOT_CMD_SIGNAL_LINE_VOLTAGE_TELEMETRY,
    DSHOT_CMD_SIGNAL_LINE_CURRENT_TELEMETRY,
    DSHOT_CMD_SIGNAL_LINE_CONSUMPTION_TELEMETRY,
    DSHOT_CMD_SIGNAL_LINE_ERPM_TELEMETRY,
    DSHOT_CMD_SIGNAL_LINE_ERPM_PERIOD_TELEMETRY,
    DSHOT_CMD_MAX = 47,
} dshot_cmd_t;

typedef enum dshot_mode_e
{
    DSHOT_OFF,
    DSHOT150,
    DSHOT300,
    DSHOT600,
    DSHOT1200
} dshot_mode_t;

typedef enum dshot_get_packet_exit_mode_e
{
    DECODE_SUCCESS = 0,
    ERR_EMPTY_QUEUE,
    ERR_NO_PACKETS,
    ERR_CHECKSUM_FAIL,
    ERR_BIDIRECTION_DISABLED,
} dshot_get_packet_exit_mode_t;

typedef enum dshot_send_packet_exit_mode_e
{
    SEND_SUCCESS = 0,
    ERR_RMT_DISABLE_FAILURE,
} dshot_send_packet_exit_mode_t;

typedef enum extended_telem_type_e
{
    TELEM_TYPE_ERPM = 0x1,
    TELEM_TYPE_TEMPRATURE = 0x2,
    TELEM_TYPE_VOLTAGE = 0x4,
    TELEM_TYPE_CURRENT = 0x6,
    TELEM_TYPE_DEBUG_A = 0x8,
    TELEM_TYPE_DEBUG_B = 0xA,
    TELEM_TYPE_STRESS_LEVEL = 0xC,
    TELEM_TYPE_STATUS = 0xE,
} extended_telem_type_t;

static const char* const dshot_mode_name[] = {
    "DSHOT_OFF",
    "DSHOT150",
    "DSHOT300",
    "DSHOT600",
    "DSHOT1200"
};

typedef std::string dshot_name_t;

typedef enum telemetric_request_e
{
    NO_TELEMETRIC,
    ENABLE_TELEMETRIC,
} telemetric_request_t;

typedef enum bidirectional_mode_e
{
    NO_BIDIRECTION,
    ENABLE_BIDIRECTION,
} bidirectional_mode_t;

typedef struct tx_callback_datapack_s
{
    gpio_num_t gpio_num;
    rmt_channel_handle_t channel_handle;
    rmt_receive_config_t channel_config;
    rmt_symbol_word_t* raw_symbols;
    size_t raw_sym_size;
} tx_callback_datapack_t;

typedef struct rx_callback_datapack_s
{
    QueueHandle_t receive_queue;
} rx_callback_datapack_t;

typedef struct rx_frame_data_s
{
    size_t num_symbols;
    rmt_symbol_word_t received_symbols[RX_SYMBOL_MAX];
} rx_frame_data_t;

typedef union {
    struct {
        uint16_t crc: 4;
        uint16_t telemetry: 1;
        uint16_t throttle: 11;
    };
    uint16_t val;
} dshot_esc_frame_t;

typedef struct dshot_config_s
{
    dshot_mode_t mode;
    dshot_name_t name_str;
    bidirectional_mode_t bidirectional;
    uint16_t num_motor_poles;
    gpio_num_t gpio_num;
    uint16_t ticks_per_bit;
    uint16_t ticks_zero_high;
    uint16_t ticks_zero_low;
    uint16_t ticks_one_high;
    uint16_t ticks_one_low;
    uint32_t micros_per_frame;
    uint32_t micros_per_shortest;
    tx_callback_datapack_t tx_callback_datapack;
    rx_callback_datapack_t rx_callback_datapack;
    rmt_channel_handle_t tx_chan;
    rmt_channel_handle_t rx_chan;
    rmt_encoder_handle_t copy_encoder;
} dshot_config_t;

class DShotRMT
{
public:
    DShotRMT(uint8_t pin);
    ~DShotRMT();

    void begin(dshot_mode_t dshot_mode = DSHOT_OFF, bidirectional_mode_t is_bidirectional = NO_BIDIRECTION, uint16_t magnet_count = 14);
    dshot_send_packet_exit_mode_t send_dshot_value(uint16_t throttle_value, bool get_onewire_telemetry = true, telemetric_request_t telemetric_request = NO_TELEMETRIC);
    dshot_get_packet_exit_mode_t get_dshot_packet(uint32_t* value, extended_telem_type_t* packetType = NULL);
    float convert_packet_to_volts(uint8_t value);
    float get_telem_success_rate();

private:
    QueueHandle_t receive_queue;
    rmt_symbol_word_t dshot_rx_rmt_item[64] = {};
    rmt_symbol_word_t dshot_tx_rmt_item[DSHOT_PACKET_LENGTH] = {};
    dshot_config_t dshot_config;
    uint32_t successful_packets = 0;
    uint32_t error_packets = 0;

    void encode_dshot_to_rmt(uint16_t parsed_packet);
    uint16_t calc_dshot_chksum(const dshot_esc_frame_t& dshot_frame);
    uint32_t decode_eRPM_telemetry_value(uint16_t value);
    uint32_t erpmToRpm(uint16_t erpm, uint16_t motorPoleCount);
    static void handle_error(esp_err_t);
};

#endif