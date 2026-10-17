//
// UBX protocol helpers for u-blox M10 configuration.
//

#pragma once

#include <cstddef>
#include <cstdint>

namespace proto::ubx
{
    constexpr uint8_t SYNC1 = 0xB5;
    constexpr uint8_t SYNC2 = 0x62;

    constexpr uint8_t CLASS_ACK = 0x05;
    constexpr uint8_t ID_ACK_NAK = 0x00;
    constexpr uint8_t ID_ACK_ACK = 0x01;

    constexpr uint8_t CLASS_CFG = 0x06;
    constexpr uint8_t ID_CFG_VALSET = 0x8A;

    // VALSET target layers, may be OR'ed together.
    constexpr uint8_t LAYER_RAM = 0x01;
    constexpr uint8_t LAYER_BBR = 0x02;
    constexpr uint8_t LAYER_FLASH = 0x04;

    // Configuration keys, M10 interface description (34.xx).
    constexpr uint32_t KEY_RATE_MEAS = 0x30210001;
    constexpr uint32_t KEY_RATE_NAV = 0x30210002;
    constexpr uint32_t KEY_UART1_BAUDRATE = 0x40520001;

    constexpr uint32_t KEY_MSGOUT_NMEA_DTM_UART1 = 0x209100a7;
    constexpr uint32_t KEY_MSGOUT_NMEA_GBS_UART1 = 0x209100de;
    constexpr uint32_t KEY_MSGOUT_NMEA_GGA_UART1 = 0x209100bb;
    constexpr uint32_t KEY_MSGOUT_NMEA_GLL_UART1 = 0x209100ca;
    constexpr uint32_t KEY_MSGOUT_NMEA_GNS_UART1 = 0x209100b6;
    constexpr uint32_t KEY_MSGOUT_NMEA_GRS_UART1 = 0x209100cf;
    constexpr uint32_t KEY_MSGOUT_NMEA_GSA_UART1 = 0x209100c0;
    constexpr uint32_t KEY_MSGOUT_NMEA_GST_UART1 = 0x209100d4;
    constexpr uint32_t KEY_MSGOUT_NMEA_GSV_UART1 = 0x209100c5;
    constexpr uint32_t KEY_MSGOUT_NMEA_RMC_UART1 = 0x209100ac;
    constexpr uint32_t KEY_MSGOUT_NMEA_VLW_UART1 = 0x209100e8;
    constexpr uint32_t KEY_MSGOUT_NMEA_VTG_UART1 = 0x209100b1;
    constexpr uint32_t KEY_MSGOUT_NMEA_ZDA_UART1 = 0x209100d9;

    struct KeyValue {
        uint32_t key;
        uint32_t value;
    };

    // Value width in bytes encoded in bits 28..30 of the key.
    size_t valueSize(uint32_t key);

    // Serializes a CFG-VALSET frame into out. Returns frame length, or 0 if it does not fit.
    size_t buildValset(uint8_t layers, const KeyValue *kv, size_t count, uint8_t *out, size_t out_cap);

    // Incremental parser looking for an ACK-ACK/ACK-NAK matching a request.
    class AckScanner {
    public:
        AckScanner(uint8_t req_class, uint8_t req_id) : req_class_(req_class), req_id_(req_id) {}

        enum class Result { Pending, Ack, Nak };

        Result feed(uint8_t byte);

    private:
        uint8_t req_class_;
        uint8_t req_id_;
        uint8_t state_{0};
        uint8_t msg_id_{0};
        uint8_t payload_[2]{};
        uint8_t payload_len_{0};
    };
}
