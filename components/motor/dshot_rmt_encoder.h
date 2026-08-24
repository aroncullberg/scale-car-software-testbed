#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <driver/rmt_encoder.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        uint16_t throttle;
        bool telemetry_req;
    } dshot_rmt_throttle_t;

    typedef struct
    {
        uint32_t resolution;
        uint32_t baud_rate;
        bool bidirectional;
        uint32_t post_delay_us;
    } dshot_rmt_encoder_config_t;

    esp_err_t rmt_new_dshot_esc_encoder(const dshot_rmt_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder);

#ifdef __cplusplus
}
#endif
