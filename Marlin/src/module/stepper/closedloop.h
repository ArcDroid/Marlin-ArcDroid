/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#pragma once

/**
 * stepper/closedloop.h
 * Stepper driver indirection for smart closedloop drivers like BTT S42B
 */

#include "../../inc/MarlinConfig.h"

#include "../../MarlinCore.h"

#if (__cplusplus == 201703L) && defined(__has_include)
	#define SW_CAPABLE_PLATFORM __has_include(<SoftwareSerial.h>)
#elif defined(__AVR__) || defined(TARGET_LPC1768) || defined(ARDUINO_ARCH_STM32)
	#define SW_CAPABLE_PLATFORM true
#else
	#define SW_CAPABLE_PLATFORM false
#endif

#if SW_CAPABLE_PLATFORM
	#include <SoftwareSerial.h>
#endif

#include "../planner.h"

class S42BClosedLoop {
public:
    S42BClosedLoop(Stream * SerialPort);
#if SW_CAPABLE_PLATFORM
    S42BClosedLoop(uint16_t SW_RX_pin, uint16_t SW_TX_pin);
    void beginSerial(uint32_t baudrate) __attribute__((weak));
#else
    void beginSerial(uint32_t) = delete; // Your platform does not currently support Software Serial
#endif

    int32_t readPosition(bool onetry = false);

    bool userCommand(uint8_t function, uint16_t data);

    static int32_t positionIsError(int32_t position) {
        if (position > 0x7f000000)
            return position - 0x7f000000;
        else
            return 0;
    }


    uint16_t bytesWritten = 0;
    bool CRCerror = false;
    int16_t last_error = 0;

protected:
    Stream * HWSerial = nullptr;
#if SW_CAPABLE_PLATFORM
    SoftwareSerial * SWSerial = nullptr;
#endif

    int available();
    void preCommunication();
    int16_t serial_read();
    uint8_t serial_write(const uint8_t data);
    void postCommunication();
    int16_t serial_read_timed(uint32_t millis_deadline);

    static uint8_t calcChecksum(uint8_t buffer[], uint8_t start, uint8_t len);
    static uint8_t calcChecksum(uint8_t l_val, uint8_t f_val, uint8_t buffer[], uint8_t start, uint8_t len);

    void sendCommandBase(const uint8_t function, const uint16_t data);
    int16_t sendCommandBinary(const uint8_t function, const uint16_t data, uint8_t respBuffer[], const uint8_t rBufSize, uint16_t timeout);

    static constexpr uint8_t replyDelay = 2;
    static constexpr uint8_t abort_window = 5;
	static constexpr uint8_t max_retries = 2;
};

template<char AXIS_LETTER, char DRIVER_ID, AxisEnum AXIS_ID>
class ClosedLoopMarlin : public S42BClosedLoop {
  public:
    ClosedLoopMarlin(Stream * SerialPort) : S42BClosedLoop(SerialPort) {

    }
    ClosedLoopMarlin(uint16_t SW_RX_pin, uint16_t SW_TX_pin) : S42BClosedLoop(SW_RX_pin, SW_TX_pin) {

    }

    float encoder_counts_per_step;
    int32_t encoder_counts_per_rev;
    int32_t encoder_offset;
    int32_t home_pulse;
    bool homed;
    void check_comms();

    inline float encoder_counts_per_unit() {
        return planner.settings.axis_steps_per_mm[AXIS_ID] * encoder_counts_per_step;
    }

    bool touch_off_encoder(float new_position, bool calibrate_home) {
        int32_t pos = new_position * encoder_counts_per_unit();
        int32_t raw_read = readPosition();
        int32_t error = S42BClosedLoop::positionIsError(raw_read);
        if (error != 0) {
            homed = false;
            SERIAL_ERROR_MSG("touch_off_encoder Axis:", ((const char[]){ AXIS_LETTER, '\0'})
                ,"Error:", error
                );
            return false;
        }

        // measured offset from switch position to expected encoder count
        int32_t switch_offset = pos - raw_read;
        int32_t switch_revolutions = switch_offset / encoder_counts_per_rev;

        // expected offset for home
        int32_t expected_offset = home_pulse + switch_revolutions * encoder_counts_per_rev;

        // if calibrating, we'll use the new offset as home
        if (calibrate_home) {
            expected_offset = switch_offset;
        }

        int32_t delta = switch_offset - expected_offset;

        int32_t half = encoder_counts_per_rev / 2;

        // fix up any encoder rollover
        if (delta >= half) {
            delta -= encoder_counts_per_rev;
            expected_offset += encoder_counts_per_rev;
        }
        else if (delta < -half) {
            delta += encoder_counts_per_rev;
            expected_offset -= encoder_counts_per_rev;
        }

        #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) {
            DEBUG_ECHO("touch_off_encoder axis:");
            DEBUG_CHAR(AXIS_LETTER);
            DEBUG_ECHOPAIR(" delta: ", delta);
            DEBUG_ECHOPAIR(" expected_offset: ", expected_offset);
            DEBUG_ECHOPAIR(" switch_offset: ", switch_offset);
            DEBUG_EOL();
        }
        #endif

        int32_t margin = encoder_counts_per_rev / 64;
        if (delta > margin || delta < -margin) {
            SERIAL_ECHO("warn: touch_off_encoder encoder position doesn't match expected axis:");
            SERIAL_CHAR(AXIS_LETTER);
            SERIAL_ECHO(" error: ");
            SERIAL_ECHOLN(delta);
        }

        encoder_offset = expected_offset;
        homed = true;

        if (calibrate_home) {
            // remove any extra revolutions
            expected_offset = expected_offset % encoder_counts_per_rev;
            // fix negative
            if (expected_offset < 0) {
                expected_offset += encoder_counts_per_rev;
            }
            // save new offset
            home_pulse = expected_offset;

            SERIAL_ECHO("debug: touch_off_encoder calibrated ");
            SERIAL_CHAR(AXIS_LETTER);
            SERIAL_ECHO(" offset: ");
            SERIAL_ECHOLN(home_pulse);
        }
        return true;
    }

    float to_mm(int32_t enc_count_noscale) {
        return (enc_count_noscale) / encoder_counts_per_unit();
    }

    float read_encoder() {
        int32_t raw_read = readPosition();
        int32_t error = S42BClosedLoop::positionIsError(raw_read);
        if (error != 0) {
            return NAN;
        }
        return (raw_read + encoder_offset) / encoder_counts_per_unit();
    }

    int32_t read_encoder_noscale() {
        int32_t raw_read = readPosition(true);
        int32_t error = S42BClosedLoop::positionIsError(raw_read);
        if (error != 0) {
            return raw_read;
        }
        return (raw_read + encoder_offset);
    }

    int32_t get_home_offset() {
        int32_t offset = encoder_offset;
        int32_t half = (encoder_counts_per_rev>>1);

        offset = offset % encoder_counts_per_rev;
        if (offset > half) {
            offset -= encoder_counts_per_rev;
        } else if (offset <= -half) {
            offset += encoder_counts_per_rev;
        }
        return offset;
    }

};


#define CLOSEDLOOP_X_LABEL 'X', '0'
#define CLOSEDLOOP_Y_LABEL 'Y', '0'
#define CLOSEDLOOP_Z_LABEL 'Z', '0'

#define CLOSEDLOOP_X2_LABEL 'X', '2'
#define CLOSEDLOOP_Y2_LABEL 'Y', '2'
#define CLOSEDLOOP_Z2_LABEL 'Z', '2'
#define CLOSEDLOOP_Z3_LABEL 'Z', '3'
#define CLOSEDLOOP_Z4_LABEL 'Z', '4'

#define CLOSEDLOOP_E0_LABEL 'E', '0'
#define CLOSEDLOOP_E1_LABEL 'E', '1'
#define CLOSEDLOOP_E2_LABEL 'E', '2'
#define CLOSEDLOOP_E3_LABEL 'E', '3'
#define CLOSEDLOOP_E4_LABEL 'E', '4'
#define CLOSEDLOOP_E5_LABEL 'E', '5'
#define CLOSEDLOOP_E6_LABEL 'E', '6'
#define CLOSEDLOOP_E7_LABEL 'E', '7'

#define __CLOSEDLOOP_CLASS(L, I, A) ClosedLoopMarlin<L, I, A>
#define _CLOSEDLOOP_CLASS(LandI, A) __CLOSEDLOOP_CLASS(LandI, A)
#define CLOSEDLOOP_CLASS(ST, A) _CLOSEDLOOP_CLASS(CLOSEDLOOP_##ST##_LABEL, A##_AXIS)
#if ENABLED(DISTINCT_E_FACTORS)
  #define CLOSEDLOOP_CLASS_E(N) CLOSEDLOOP_CLASS(E##N, E##N)
#else
  #define CLOSEDLOOP_CLASS_E(N) CLOSEDLOOP_CLASS(E##N, E)
#endif

void closedloop_serial_begin();

void restore_closedloop_drivers();

void closedloop_home_encoders(AxisEnum axis, abce_pos_t motor_pos, bool calibrate_home);
bool closedloop_has_aligned();
bool closedloop_need_restore();
bool closedloop_restore_position(abce_pos_t *motor_pos, bool enable, bool force_read);
void closedloop_unhome(AxisEnum axis);


void closedloop_reset_pps();
void closedloop_set_pps(abc_float_t pps);
abc_float_t closedloop_get_pps();

void closedloop_reset_home_pulse();
void closedloop_set_home_pulse(abc_long_t home);
abc_long_t closedloop_get_home_pulse();

// X Stepper
#if AXIS_IS_CLOSEDLOOP(X)
  extern CLOSEDLOOP_CLASS(X, X) encoderX;
  #if ENABLED(SOFTWARE_DRIVER_ENABLE)
    #define X_ENABLE_INIT() NOOP
    #define X_ENABLE_WRITE(STATE) encoderX.toff((STATE)==X_ENABLE_ON ? chopper_timing.toff : 0)
    #define X_ENABLE_READ() encoderX.isEnabled()
  #endif
#endif

// Y Stepper
#if AXIS_IS_CLOSEDLOOP(Y)
  extern CLOSEDLOOP_CLASS(Y, Y) encoderY;
  #if ENABLED(SOFTWARE_DRIVER_ENABLE)
    #define Y_ENABLE_INIT() NOOP
    #define Y_ENABLE_WRITE(STATE) encoderY.toff((STATE)==Y_ENABLE_ON ? chopper_timing.toff : 0)
    #define Y_ENABLE_READ() encoderY.isEnabled()
  #endif
#endif
