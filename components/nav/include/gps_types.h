//
// Created by aron on 2025-04-27.
//

#pragma once
#include <cstdint>


namespace nav
{

    enum class FixType : uint8_t {
        NO_FIX      = 0,    // Receiver has no valid position
        FIX_2D      = 1,    // Receiver has a 2D fix (latitude and longitude)
        FIX_3D      = 2,    // Receiver has a 3D fix (latitude, longitude, and altitude)
        DGPS        = 3,    // Receiver has a differential GPS fix
        RTK_FLOAT   = 4,    // Receiver has a float RTK fix
        RTK_FIXED   = 5,    // Receiver has a fixed RTK fix
    };


    struct Location {
        bool isValid{false};  ///< True if the location is valid
        uint32_t age{};
        int32_t lat_e7{0};    ///< Latitude in degrees × 1e7
        int32_t lon_e7{0};    ///< Longitude in degrees × 1e7
    };

    // TODO: maybe change to signed integers for sensible init values (?)
    struct Date {
        bool isValid{false};
        uint32_t age{};
        uint16_t year{};
        uint8_t month{};
        uint8_t day{};
    };

    struct Time {
        bool isValid{false};
        uint32_t age{};
        uint8_t hour{};
        uint8_t minute{};
        uint8_t second{};
        uint8_t centisecond{};
    };

    struct Speed {
        int32_t speed_mmps{};
        int32_t speed_kmph{};
        int32_t knots{};
    };

    struct Course {
        int32_t course_cd{};
    };

    struct Altitude {
        int32_t altitude_m{};
    };

    struct Satellite {
        bool isValid{false};
        uint32_t age{};
        uint32_t satellites{};
    };

    using HDOP = uint16_t;
}
