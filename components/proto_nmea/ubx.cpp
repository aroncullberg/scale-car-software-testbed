//
// UBX protocol helpers for u-blox M10 configuration.
//

#include "ubx.h"

namespace proto::ubx
{
    namespace
    {
        void checksum(const uint8_t *data, size_t len, uint8_t &ck_a, uint8_t &ck_b) {
            ck_a = 0;
            ck_b = 0;
            for (size_t i = 0; i < len; ++i) {
                ck_a = static_cast<uint8_t>(ck_a + data[i]);
                ck_b = static_cast<uint8_t>(ck_b + ck_a);
            }
        }
    }

    size_t valueSize(uint32_t key) {
        switch ((key >> 28) & 0x7) {
            case 1: // one bit, stored as one byte
            case 2: return 1;
            case 3: return 2;
            case 4: return 4;
            case 5: return 8;
            default: return 0;
        }
    }

    size_t buildValset(uint8_t layers, const KeyValue *kv, size_t count, uint8_t *out, size_t out_cap) {
        size_t payload_len = 4;
        for (size_t i = 0; i < count; ++i) {
            const size_t width = valueSize(kv[i].key);
            if (width == 0 || width > 4) return 0;
            payload_len += 4 + width;
        }

        const size_t frame_len = 6 + payload_len + 2;
        if (frame_len > out_cap || payload_len > 0xFFFF) return 0;

        out[0] = SYNC1;
        out[1] = SYNC2;
        out[2] = CLASS_CFG;
        out[3] = ID_CFG_VALSET;
        out[4] = static_cast<uint8_t>(payload_len & 0xFF);
        out[5] = static_cast<uint8_t>(payload_len >> 8);

        size_t at = 6;
        out[at++] = 0x00; // version
        out[at++] = layers;
        out[at++] = 0x00; // reserved
        out[at++] = 0x00;

        for (size_t i = 0; i < count; ++i) {
            const uint32_t key = kv[i].key;
            out[at++] = static_cast<uint8_t>(key & 0xFF);
            out[at++] = static_cast<uint8_t>((key >> 8) & 0xFF);
            out[at++] = static_cast<uint8_t>((key >> 16) & 0xFF);
            out[at++] = static_cast<uint8_t>((key >> 24) & 0xFF);

            const size_t width = valueSize(key);
            uint32_t value = kv[i].value;
            for (size_t b = 0; b < width; ++b) {
                out[at++] = static_cast<uint8_t>(value & 0xFF);
                value >>= 8;
            }
        }

        uint8_t ck_a = 0, ck_b = 0;
        checksum(out + 2, at - 2, ck_a, ck_b);
        out[at++] = ck_a;
        out[at++] = ck_b;
        return at;
    }

    AckScanner::Result AckScanner::feed(uint8_t byte) {
        switch (state_) {
            case 0:
                if (byte == SYNC1) state_ = 1;
                break;
            case 1:
                state_ = (byte == SYNC2) ? 2 : (byte == SYNC1 ? 1 : 0);
                break;
            case 2:
                state_ = (byte == CLASS_ACK) ? 3 : 0;
                break;
            case 3:
                if (byte != ID_ACK_ACK && byte != ID_ACK_NAK) {
                    state_ = 0;
                    break;
                }
                msg_id_ = byte;
                state_ = 4;
                break;
            case 4:
                state_ = (byte == 0x02) ? 5 : 0; // ACK payloads are two bytes
                break;
            case 5:
                payload_len_ = 0;
                state_ = (byte == 0x00) ? 6 : 0;
                break;
            case 6:
            case 7:
                payload_[payload_len_++] = byte;
                state_++;
                if (state_ == 8) {
                    state_ = 0;
                    const bool match = payload_[0] == req_class_ && payload_[1] == req_id_;
                    payload_len_ = 0;
                    if (match) return msg_id_ == ID_ACK_ACK ? Result::Ack : Result::Nak;
                }
                break;
            default:
                state_ = 0;
                break;
        }
        return Result::Pending;
    }
}
