//
// Created by aron on 2025-04-27.
//

#pragma once

#include <atomic>

#include "gps_types.h"

namespace proto { class NmeaDriver; };

namespace nav
{
    class Gps {
    public:
        static Gps &instance();

        // TODO: Decide if we should change Location => Position
        Location getLocation() const;
        Date getDate() const;
        Time getTime() const;
        Speed getSpeed() const;
        Course getCourse() const;
        Altitude getAltitude() const;
        Satellite getSatellite() const;
        HDOP getHDOP() const;

        Gps(const Gps &) = delete;
        Gps &operator=(const Gps &) = delete;

    private:
        friend class proto::NmeaDriver;
        Gps() = default;

        void setLocation(const Location &l);
        void setDate(const Date &d);
        void setTime(const Time &t);
        void setSpeed(const Speed &s);
        void setCourse(const Course &c);
        void setAltitude(const Altitude &a);
        void setSatellite(const Satellite &s);
        void setHDOP(const HDOP &h);

        std::atomic<Location> location_{Location()};
        std::atomic<Date> date_{Date()};
        std::atomic<Time> time_{Time()};
        std::atomic<Speed> speed_{Speed()};
        std::atomic<Course> course_{Course()};
        std::atomic<Altitude> altitude_{Altitude()};
        std::atomic<Satellite> satellite_{Satellite()};
        std::atomic<HDOP> hdop_{HDOP()};
    };
}
