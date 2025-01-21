#include "DShotRMT.h"
#include "hal/gpio_hal.h"
#include "hal/gpio_ll.h"
#include <cstring>
#include "esp_log.h"

static const char* TAG = "DShotRMT";

extern "C" {

#define CONFIG_RMT_ISR_IRAM_SAFE 1
#if CONFIG_RMT_ISR_IRAM_SAFE
#define TEST_RMT_CALLBACK_ATTR IRAM_ATTR
#else
#define TEST_RMT_CALLBACK_ATTR
#endif

TEST_RMT_CALLBACK_ATTR
static bool rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;
    rx_callback_datapack_t* config = (rx_callback_datapack_t*)user_data;
    rx_frame_data_t out_data = {};

    size_t sym_count = edata->num_symbols > RX_SYMBOL_MAX ? RX_SYMBOL_MAX : edata->num_symbols;
    memcpy(&out_data.received_symbols, edata->received_symbols, sym_count * sizeof(rmt_symbol_word_t));
    out_data.num_symbols = sym_count;

    size_t last_sym = edata->num_symbols - 1;
    if(edata->received_symbols[last_sym].duration0 != 0
        && edata->received_symbols[last_sym].duration1 != 0)
    {
        // Non-terminating packet detected
        ESP_LOGD(TAG, "Non-terminating packet detected");
    }

    xQueueSendFromISR(config->receive_queue, &out_data, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

TEST_RMT_CALLBACK_ATTR
static bool tx_done_callback(rmt_channel_handle_t tx_chan, const rmt_tx_done_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_wakeup = pdFALSE;
    tx_callback_datapack_t* config = (tx_callback_datapack_t*)user_ctx;

    rmt_enable(config->channel_handle);
    gpio_ll_od_enable(GPIO_LL_GET_HW(GPIO_PORT_0), config->gpio_num);
    rmt_receive(config->channel_handle, config->raw_symbols, config->raw_sym_size, &config->channel_config);
    
    return high_task_wakeup;
}

}

static const unsigned char GCR_encode[16] = {
    0x19, 0x1B, 0x12, 0x13,
    0x1D, 0x15, 0x16, 0x17,
    0x1A, 0x09, 0x0A, 0x0B,
    0x1E, 0x0D, 0x0E, 0x0F
};

static const unsigned char GCR_decode[32] = {
    0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 9, 10, 11,
    0xFF, 13, 14, 15,
    0xFF, 0xFF, 2, 3,
    0xFF, 5, 6, 7,
    0xFF, 0, 8, 1,
    0xFF, 4, 12, 0xFF,
};

DShotRMT::DShotRMT(uint8_t pin)
{
    dshot_config.gpio_num = (gpio_num_t)pin;
}

DShotRMT::~DShotRMT()
{
    if(dshot_config.bidirectional)
    {
        rmt_del_channel(dshot_config.rx_chan);
        vQueueDelete(receive_queue);
    }
    rmt_del_channel(dshot_config.tx_chan);
    rmt_del_encoder(dshot_config.copy_encoder);
}

void DShotRMT::begin(dshot_mode_t dshot_mode, bidirectional_mode_t is_bidirectional, uint16_t magnet_count)
{
    if(dshot_mode == DSHOT150)
        dshot_config.bidirectional = NO_BIDIRECTION;
    else
        dshot_config.bidirectional = is_bidirectional;

    dshot_config.mode = dshot_mode;
    dshot_config.name_str = dshot_mode_name[dshot_mode];

    switch (dshot_config.mode)
    {
    case DSHOT150:
        dshot_config.ticks_per_bit = 64;
        dshot_config.ticks_zero_high = 24;
        dshot_config.ticks_one_high = 48;
        break;

    case DSHOT300:
        dshot_config.ticks_per_bit = 32;
        dshot_config.ticks_zero_high = 12;
        dshot_config.ticks_one_high = 24;
        break;

    case DSHOT600:
        dshot_config.ticks_per_bit = 16;
        dshot_config.ticks_zero_high = 6;
        dshot_config.ticks_one_high = 12;
        break;

    case DSHOT1200:
        dshot_config.ticks_per_bit = 8;
        dshot_config.ticks_zero_high = 3;
        dshot_config.ticks_one_high = 6;
        break;

    default:
        dshot_config.ticks_per_bit = 0;
        dshot_config.ticks_zero_high = 0;
        dshot_config.ticks_one_high = 0;
        break;
    }

    dshot_config.micros_per_frame = 16 * dshot_config.ticks_per_bit * 110;
    dshot_config.micros_per_shortest = dshot_config.ticks_zero_high * 90;

    dshot_config.ticks_zero_low = (dshot_config.ticks_per_bit - dshot_config.ticks_zero_high);
    dshot_config.ticks_one_low = (dshot_config.ticks_per_bit - dshot_config.ticks_one_high);

    dshot_config.num_motor_poles = magnet_count;

    rmt_tx_channel_config_t tx_chan_config = {
        .gpio_num = dshot_config.gpio_num,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RMT_CYCLES_PER_SEC,
        .mem_block_symbols = 64,
        .trans_queue_depth = 10
    };

    if(dshot_config.bidirectional)
    {
        tx_chan_config.flags.io_od_mode = 1;
        tx_chan_config.flags.io_loop_back = 1;
        tx_chan_config.flags.invert_out = 1;

        rmt_rx_channel_config_t rx_chan_config = {
            .gpio_num = dshot_config.gpio_num,
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .resolution_hz = RMT_CYCLES_PER_SEC,
            .mem_block_symbols = 64,
            .flags = {
                .invert_in = 1,
                .io_loop_back = 1,
            }
        };

        handle_error(rmt_new_rx_channel(&rx_chan_config, &dshot_config.rx_chan));

        rmt_rx_event_callbacks_t callback = {
            .on_recv_done = rx_done_callback
        };

        receive_queue = xQueueCreate(1, sizeof(rx_frame_data_t));
        dshot_config.rx_callback_datapack.receive_queue = receive_queue;
        handle_error(rmt_rx_register_event_callbacks(dshot_config.rx_chan, &callback, &dshot_config.rx_callback_datapack));

        dshot_config.tx_callback_datapack.channel_config = {
            .signal_range_min_ns = dshot_config.micros_per_shortest,
            .signal_range_max_ns = dshot_config.micros_per_frame
        };

        dshot_config.tx_callback_datapack.channel_handle = dshot_config.rx_chan;
        dshot_config.tx_callback_datapack.raw_symbols = dshot_rx_rmt_item;
        dshot_config.tx_callback_datapack.raw_sym_size = sizeof(rmt_symbol_word_t) * DSHOT_PACKET_LENGTH;
        dshot_config.tx_callback_datapack.gpio_num = dshot_config.gpio_num;
    }

    handle_error(rmt_new_tx_channel(&tx_chan_config, &dshot_config.tx_chan));

    if(dshot_config.bidirectional)
    {
        rmt_tx_event_callbacks_t callback = {
            .on_trans_done = tx_done_callback
        };

        handle_error(rmt_tx_register_event_callbacks(dshot_config.tx_chan, &callback, &dshot_config.tx_callback_datapack));
        handle_error(rmt_enable(dshot_config.rx_chan));
    }

    handle_error(rmt_enable(dshot_config.tx_chan));

    rmt_copy_encoder_config_t copy_encoder_config = {};
    handle_error(rmt_new_copy_encoder(&copy_encoder_config, &dshot_config.copy_encoder));
}

dshot_send_packet_exit_mode_t DShotRMT::send_dshot_value(uint16_t throttle_value, bool get_onewire_telemetry, telemetric_request_t telemetric_request)
{
    dshot_esc_frame_t dshot_frame = {};

    if (throttle_value > DSHOT_THROTTLE_MAX)
        throttle_value = DSHOT_THROTTLE_MAX;

    dshot_frame.throttle = throttle_value;
    dshot_frame.telemetry = telemetric_request;
    dshot_frame.crc = calc_dshot_chksum(dshot_frame);

    encode_dshot_to_rmt(dshot_frame.val);

    gpio_ll_od_disable(GPIO_LL_GET_HW(GPIO_PORT_0), dshot_config.gpio_num);

    rmt_transmit_config_t tx_config = {};

    if(dshot_config.bidirectional == ENABLE_BIDIRECTION) {
        esp_err_t err = rmt_disable(dshot_config.rx_chan);
        switch(err) {
            case ESP_OK:
                break;
            default:
                handle_error(err);
                return ERR_RMT_DISABLE_FAILURE;
        }
    }

    rmt_transmit(dshot_config.tx_chan,
                dshot_config.copy_encoder,
                &dshot_tx_rmt_item,
                sizeof(rmt_symbol_word_t) * DSHOT_PACKET_LENGTH,
                &tx_config);

    return SEND_SUCCESS;
}

void DShotRMT::encode_dshot_to_rmt(uint16_t parsed_packet)
{
    for (int i = 0; i < DSHOT_PAUSE_BIT; i++, parsed_packet <<= 1) 
    {
        if (parsed_packet & 0b1000000000000000)
        {
            dshot_tx_rmt_item[i].duration0 = dshot_config.ticks_one_high;
            dshot_tx_rmt_item[i].duration1 = dshot_config.ticks_one_low;
        }
        else
        {
            dshot_tx_rmt_item[i].duration0 = dshot_config.ticks_zero_high;
            dshot_tx_rmt_item[i].duration1 = dshot_config.ticks_zero_low;
        }
        dshot_tx_rmt_item[i].level0 = 1;
        dshot_tx_rmt_item[i].level1 = 0;
    }

    dshot_tx_rmt_item[DSHOT_PAUSE_BIT].level0 = 0;
    dshot_tx_rmt_item[DSHOT_PAUSE_BIT].level1 = 1;
    dshot_tx_rmt_item[DSHOT_PAUSE_BIT].duration1 = (DSHOT_PAUSE / 2);
}

uint16_t DShotRMT::calc_dshot_chksum(const dshot_esc_frame_t &dshot_frame)
{
    uint16_t packet = DSHOT_NULL_PACKET;
    uint16_t chksum = DSHOT_NULL_PACKET;

    packet = (dshot_frame.throttle << 1) | dshot_frame.telemetry;

    if (dshot_config.bidirectional) {
        chksum = (~(packet ^ (packet >> 4) ^ (packet >> 8))) & 0x0F;
    } else {
        chksum = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
    }

    return chksum;
}

uint32_t DShotRMT::decode_eRPM_telemetry_value(uint16_t value)
{
    if (value == 0x0fff) {
        return 0;
    }

    value = (value & 0x01ff) << ((value & 0xfe00) >> 9);
    if (!value) {
        return 0;
    }

    return (1000000 * 60 / 100 + value / 2) / value;
}

uint32_t DShotRMT::erpmToRpm(uint16_t erpm, uint16_t motorPoleCount)
{
    return (erpm * 200) / motorPoleCount;
}

dshot_get_packet_exit_mode_t DShotRMT::get_dshot_packet(uint32_t* value, extended_telem_type_t* packetType)
{
    if(dshot_config.bidirectional != ENABLE_BIDIRECTION)
    {
        return ERR_BIDIRECTION_DISABLED;
    }

    rx_frame_data_t rx_data;

    if(xQueueReceive(receive_queue, &rx_data, 0))
    {
        if (rx_data.num_symbols > 1)
        {
            int i,j;
            unsigned short bitTime = dshot_config.ticks_one_high;
            unsigned short bitCount0 = 0;
            unsigned short bitCount1 = 0;
            unsigned short bitShiftLevel = 20;
            unsigned int assembledFrame = 0;

            for (i = 0; i < rx_data.num_symbols; ++i)
            {
                bitCount0 = rx_data.received_symbols[i].duration0 / bitTime + 
                           (rx_data.received_symbols[i].duration0 % bitTime > bitTime - 4);
                bitCount1 = rx_data.received_symbols[i].duration1 / bitTime + 
                           (rx_data.received_symbols[i].duration1 % bitTime > bitTime - 4);

                if (rx_data.received_symbols[i].level0 == 0)
                {
                    bitShiftLevel -= bitCount0;
                    for (j = 0; j < bitCount1; ++j)
                    {
                        assembledFrame |= 1 << bitShiftLevel;
                        --bitShiftLevel;
                    }
                }
                else
                {
                    for (j = 0; j < bitCount0; ++j)
                    {
                        assembledFrame |= 1 << bitShiftLevel;
                        --bitShiftLevel;
                    }
                    bitShiftLevel -= bitCount1;
                }
            }

            assembledFrame = (assembledFrame ^ (assembledFrame >> 1));

            unsigned char nibble = 0;
            unsigned char fiveBitSubset = 0;
            unsigned int decodedFrame = 0;

            for (i = 0; i < 4; ++i)
            {
                fiveBitSubset = (assembledFrame >> (i * 5)) & 0b11111;
                nibble = GCR_decode[fiveBitSubset];
                decodedFrame |= nibble << (i * 4);
            }

            uint16_t frameData = (decodedFrame >> 4) & (0b111111111111);
            uint8_t crc = decodedFrame & (0b1111);
            uint8_t alsocrc = (~(frameData ^ (frameData >> 4) ^ (frameData >> 8))) & 0x0F;

            if(crc != alsocrc)
            {
                error_packets += 1;
                return ERR_CHECKSUM_FAIL;
            }

            if (frameData & 0b000100000000 || (~frameData & 0b111100000000) == 0b111100000000)
            {
                if(packetType)
                    *packetType = TELEM_TYPE_ERPM;
                *value = erpmToRpm(decode_eRPM_telemetry_value(frameData), dshot_config.num_motor_poles);
            }
            else
            {
                if(packetType)
                    *packetType = (extended_telem_type_t)((frameData >> 8) & 0b1111);
                *value = (frameData & 0b11111111);
            }
        }
        else
        {
            return ERR_NO_PACKETS;
        }
    }
    else
    {
        return ERR_EMPTY_QUEUE;
    }

    successful_packets += 1;
    return DECODE_SUCCESS;
}

float DShotRMT::convert_packet_to_volts(uint8_t value)
{
    return (float)value * 0.25;
}

float DShotRMT::get_telem_success_rate()
{
    return (error_packets && successful_packets) ? 
           (float)successful_packets / (float)(error_packets + successful_packets) : 0.0;
}

void DShotRMT::handle_error(esp_err_t err_code) {
    if (err_code != ESP_OK) {
        ESP_LOGE(TAG, "error: %s", esp_err_to_name(err_code));
    }
}