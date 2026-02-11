//
// Created by aron on 2025-04-26.
//

#pragma once
#include <cstdint>

namespace rclink
{
/**
* @brief Enum class representing the channel indices (1..16)
*/
enum class ChannelIndex: std::uint8_t
{
    CH1 = 0, CH2, CH3, CH4, CH5, CH6, CH7, CH8,
    CH9, CH10, CH11, CH12, CH13, CH14, CH15, CH16,
    COUNT
};

inline constexpr std::size_t k_channel_count = static_cast<std::size_t>(ChannelIndex::COUNT);

/**
* @brief Scaled channel value that is expected to be within the range 0 - 2000.
*/
using channel_value_t = uint16_t;

/**
* @brief Raw and scaled channel data pair
*/
struct ChannelPair {
    uint16_t raw;           // Raw protocol value (varies by protocol)
    channel_value_t scaled; // Scaled 0-2000 value
};

/**
* @brief All RC channels with .ch1.raw/.ch1.scaled access pattern
*/
struct RcChannels {
    ChannelPair ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8,
                ch9, ch10, ch11, ch12, ch13, ch14, ch15, ch16;
};

}
