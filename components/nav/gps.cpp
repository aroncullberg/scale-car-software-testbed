//
// Created by cullb on 2025-04-27.
//

#include "gps.h"


namespace nav
{
    Gps &Gps::instance() {
        static Gps instance;
        return instance;
    }

    Location Gps::getLocation() const {
        return location_.load(std::memory_order_relaxed);
    }

    Date Gps::getDate() const {
        return date_.load(std::memory_order_relaxed);
    }

    Time Gps::getTime() const {
        return time_.load(std::memory_order_relaxed);
    }

    Speed Gps::getSpeed() const {
        return speed_.load(std::memory_order_relaxed);
    }

    Course Gps::getCourse() const {
        return course_.load(std::memory_order_relaxed);
    }

    Altitude Gps::getAltitude() const {
        return altitude_.load(std::memory_order_relaxed);
    }

    Satellite Gps::getSatellite() const {
        return satellite_.load(std::memory_order_relaxed);
    }

    HDOP Gps::getHDOP() const {
        return hdop_.load(std::memory_order_relaxed);
    }

    void Gps::setLocation(const Location &l) {
        location_.store(l, std::memory_order_relaxed);
    }

    void Gps::setDate(const Date &d) {
        date_.store(d, std::memory_order_relaxed);
    }

    void Gps::setTime(const Time &t) {
        time_.store(t, std::memory_order_relaxed);
    }

    void Gps::setSpeed(const Speed &s) {
        speed_.store(s, std::memory_order_relaxed);
    }

    void Gps::setCourse(const Course &c) {
        course_.store(c, std::memory_order_relaxed);
    }

    void Gps::setAltitude(const Altitude &a) {
        altitude_.store(a, std::memory_order_relaxed);
    }

    void Gps::setSatellite(const Satellite &s) {
        satellite_.store(s, std::memory_order_relaxed);
    }

    void Gps::setHDOP(const HDOP &h) {
        hdop_.store(h, std::memory_order_relaxed);
    }
}
