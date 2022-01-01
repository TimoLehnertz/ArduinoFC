/**
 * @file error.h
 * @author Timo Lehnertz
 * @brief 
 * @version 0.1
 * @date 2022-01-01
 * 
 * @copyright Copyright (c) 2022
 * 
 * Principle:
 *      Anything that is required for ratemode to be operational has to be at least in warning state.
 *      If anything required for rate mode fails the aircraft will try to disarm imidiatly to prevent damage
 *
 *      Anything that is not required for rate mode is optional stuff. So if a sensor would be to fail mid flight the aircraft should continue in the best flight mode it can hold up
 */
#pragma once

/**
 * Error Enum used for sensors and more.
 * NO_ERROR:        indicates a perfectly fine operating part
 * WARNING          indicates that something needs to be fixed / calibrated but the drone should still be able to lift off
 * CRITICAL_ERROR:  Critical error. If this part can not be bypassed than we have a problem
 */
namespace Error {
    enum Error_t {
        NO_ERROR = 0,
        WARNING = 1,
        CRITICAL_ERROR = 2
    };
}

/**
 * Enum used to indicate if something is flight critical or could be bypassed
 */
namespace Critical {
    enum Critical_t {
        OPTIONAL = 0,
        FLIGHT_CRITICAL = 1
    };
}