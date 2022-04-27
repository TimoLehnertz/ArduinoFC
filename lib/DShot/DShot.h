/**
 * DShot150 implementation for Teensy 4.0 / 4.1
 * using one IntervalTimer
 * 
 * https://brushlesswhoop.com/dshot-and-bidirectional-dshot/
 * https://www.rcgroups.com/forums/showthread.php?2756129-Dshot-testing-a-new-digital-parallel-ESC-throttle-signal
 * 
 * Frame Structure:
 *  SSSSSSSSSSSTCCCC
 *  
 */
#pragma once
#include <Arduino.h>

#define MAX_DSHOT_COUNT 10

#define MASK_CRC            0b0000000000001111
#define MASK_MSB            0b1000000000000000

#define DIGITAL_CMD_MOTOR_STOP 0
#define DIGITAL_CMD_BEEP1 1
#define DIGITAL_CMD_BEEP2 2
#define DIGITAL_CMD_BEEP3 3
#define DIGITAL_CMD_BEEP4 4
#define DIGITAL_CMD_BEEP5 5
#define DIGITAL_CMD_ESC_INFO 6
#define DIGITAL_CMD_SPIN_DIRECTION_1 7
#define DIGITAL_CMD_SPIN_DIRECTION_2 8
#define DIGITAL_CMD_3D_MODE_OFF 9
#define DIGITAL_CMD_3D_MODE_ON 10
#define DIGITAL_CMD_SETTINGS_REQUEST 12
#define DIGITAL_CMD_SAVE_SETTINGS 13

#define DIGITAL_CMD_SPIN_DIRECTION_NORMAL 20
#define DIGITAL_CMD_SPIN_DIRECTION_REVERSED 21
#define DIGITAL_CMD_LED0_ON 22
#define DIGITAL_CMD_LED1_ON 23
#define DIGITAL_CMD_LED2_ON 24
#define DIGITAL_CMD_LED3_ON 25
#define DIGITAL_CMD_LED0_OFF 26
#define DIGITAL_CMD_LED1_OFF 27
#define DIGITAL_CMD_LED2_OFF 28
#define DIGITAL_CMD_LED3_OFF 29

#define DIGITAL_CMD_SIGNAL_LINE_TELEMETRY_DISABLE 32
#define DIGITAL_CMD_SIGNAL_LINE_TELEMETRY_ENABLE 33
#define DIGITAL_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY 34
#define DIGITAL_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_PERIOD_TELEMETRY 35

#define DIGITAL_CMD_SIGNAL_LINE_TEMPERATURE_TELEMETRY 42
#define DIGITAL_CMD_SIGNAL_LINE_VOLTAGE_TELEMETRY 43
#define DIGITAL_CMD_SIGNAL_LINE_CURRENT_TELEMETRY 44
#define DIGITAL_CMD_SIGNAL_LINE_CONSUMPTION_TELEMETRY 45
#define DIGITAL_CMD_SIGNAL_LINE_ERPM_TELEMETRY 46
#define DIGITAL_CMD_SIGNAL_LINE_ERPM_PERIOD_TELEMETRY 47

class DShot {
public:
    DShot() {}

    bool begin(uint8_t pin);

    void end();

    /**
     * @param throttle float 0-1
     */
    void writeThrottle(float throttle);

    /**
     * Spend next @cycles frames sending @cmd
     */
    void sendCommand(uint8_t cmd, uint8_t cycles = 1);

    /**
     * Has begin() function been called
     */
    bool isBegun() { return begun; }

    /**
     * Arm Motor
     */
    void arm();

    /**
     * Disarm Motor
     */
    void disarm();

    bool isArmed() { return armed; }

    /**
     * get digital pin
     */
    uint8_t getPin() { return pin; }

    void setPin(uint8_t pin);

private:
    // Basic
    uint8_t pin;
    uint16_t throttle = 0;
    bool begun = false;
    uint16_t payload;
    bool armed = false;

    // Comands
    uint8_t cmd;
    volatile uint8_t cmdCycles = 0;

    // Timing
    static IntervalTimer timer;
    volatile static uint8_t phase; // = 0

    // Datastructure
    static DShot* dShots[MAX_DSHOT_COUNT];
    static uint8_t dShotCount; // = 0

    void calcPayload();
    static void timerFunc();
};