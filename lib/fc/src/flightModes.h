#pragma once

namespace FlightMode {
    enum FlightMode_t {
        none = 0,
        rate = 1,
        level = 2,
        altitudeHold = 3,
        gpsHold = 4,
        wayPoint = 5,
        dreaming = 6, // used for sensors-error handling that are not flight critical

        FlightModeSize,
    };
}