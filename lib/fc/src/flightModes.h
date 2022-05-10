/**
 * @file flightModes.h
 * @author Timo Lehnertz
 * @brief 
 * @version 0.1
 * @date 2022-01-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once

namespace FlightMode {
    enum FlightMode_t {
        none = 0,
        rate,
        turtle,
        level,
        altitudeHold,
        gpsHold,
        wayPoint,
        dreaming, // used for sensors-error handling that are not flight critical

        FlightModeSize,
    };
}