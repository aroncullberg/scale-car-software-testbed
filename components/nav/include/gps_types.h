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

    enum GpsField : uint16_t {
        GPS_LOCATION = 1 << 0, GPS_DATE = 1 << 1, GPS_TIME = 1 << 2,
        GPS_SPEED = 1 << 3, GPS_COURSE = 1 << 4, GPS_ALTITUDE = 1 << 5,
        GPS_SATELLITES = 1 << 6, GPS_HDOP = 1 << 7,
    };

    struct GpsSample {
        uint64_t timestamp_us = 0;
        int32_t lat_e7 = 0, lon_e7 = 0, altitude_mm = 0, speed_mmps = 0, course_cd = 0;
        uint32_t satellites = 0, location_age_ms = UINT32_MAX;
        uint16_t hdop_centi = 0, year = 0;
        uint8_t month = 0, day = 0, hour = 0, minute = 0, second = 0, centisecond = 0;
        uint16_t updated_fields = 0, valid_fields = 0;
    };
}
