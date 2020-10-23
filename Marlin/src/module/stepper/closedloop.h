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

class S42BClosedLoop {
public:
    S42BClosedLoop(Stream * SerialPort);
#if SW_CAPABLE_PLATFORM
    S42BClosedLoop(uint16_t SW_RX_pin, uint16_t SW_TX_pin);
    void beginSerial(uint32_t baudrate) __attribute__((weak));
#else
    void beginSerial(uint32_t) = delete; // Your platform does not currently support Software Serial
#endif

    int32_t readPosition();


    uint16_t bytesWritten = 0;
    bool CRCerror = false;

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

    int16_t sendCommand(const uint8_t function, const uint16_t data, uint8_t respBuffer[], const uint8_t rBufSize, uint16_t timeout);

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

    float encoder_counts_per_unit;
    float encoder_offset;
    bool homed;
    void check_comms();

    void touch_off_encoder(float new_position) {
        int32_t pos = new_position * encoder_counts_per_unit;
        int32_t raw_read = readPosition();
        encoder_offset = pos - raw_read;
        homed = true;
    }

    float to_mm(int32_t enc_count) {
        return (enc_count + encoder_offset) / encoder_counts_per_unit;
    }

    float read_encoder() {
        return (readPosition() + encoder_offset) / encoder_counts_per_unit;
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

void closedloop_home_encoders(abce_pos_t motor_pos);
void closedloop_restore_position(abce_pos_t *motor_pos);

// X Stepper
#if AXIS_IS_CLOSEDLOOP(X)
  extern CLOSEDLOOP_CLASS(X, X) stepperX;
  #if ENABLED(SOFTWARE_DRIVER_ENABLE)
    #define X_ENABLE_INIT() NOOP
    #define X_ENABLE_WRITE(STATE) stepperX.toff((STATE)==X_ENABLE_ON ? chopper_timing.toff : 0)
    #define X_ENABLE_READ() stepperX.isEnabled()
  #endif
#endif

// Y Stepper
#if AXIS_IS_CLOSEDLOOP(Y)
  extern CLOSEDLOOP_CLASS(Y, Y) stepperY;
  #if ENABLED(SOFTWARE_DRIVER_ENABLE)
    #define Y_ENABLE_INIT() NOOP
    #define Y_ENABLE_WRITE(STATE) stepperY.toff((STATE)==Y_ENABLE_ON ? chopper_timing.toff : 0)
    #define Y_ENABLE_READ() stepperY.isEnabled()
  #endif
#endif

// Z Stepper
#if AXIS_IS_CLOSEDLOOP(Z)
  extern CLOSEDLOOP_CLASS(Z, Z) stepperZ;
  #if ENABLED(SOFTWARE_DRIVER_ENABLE)
    #define Z_ENABLE_INIT() NOOP
    #define Z_ENABLE_WRITE(STATE) stepperZ.toff((STATE)==Z_ENABLE_ON ? chopper_timing.toff : 0)
    #define Z_ENABLE_READ() stepperZ.isEnabled()
  #endif
#endif

// X2 Stepper
#if HAS_X2_ENABLE && AXIS_IS_CLOSEDLOOP(X2)
  extern CLOSEDLOOP_CLASS(X2, X) stepperX2;
  #if ENABLED(SOFTWARE_DRIVER_ENABLE)
    #define X2_ENABLE_INIT() NOOP
    #define X2_ENABLE_WRITE(STATE) stepperX2.toff((STATE)==X_ENABLE_ON ? chopper_timing.toff : 0)
    #define X2_ENABLE_READ() stepperX2.isEnabled()
  #endif
#endif

// Y2 Stepper
#if HAS_Y2_ENABLE && AXIS_IS_CLOSEDLOOP(Y2)
  extern CLOSEDLOOP_CLASS(Y2, Y) stepperY2;
  #if ENABLED(SOFTWARE_DRIVER_ENABLE)
    #define Y2_ENABLE_INIT() NOOP
    #define Y2_ENABLE_WRITE(STATE) stepperY2.toff((STATE)==Y_ENABLE_ON ? chopper_timing.toff : 0)
    #define Y2_ENABLE_READ() stepperY2.isEnabled()
  #endif
#endif

// Z2 Stepper
#if HAS_Z2_ENABLE && AXIS_IS_CLOSEDLOOP(Z2)
  extern CLOSEDLOOP_CLASS(Z2, Z) stepperZ2;
  #if ENABLED(SOFTWARE_DRIVER_ENABLE) && AXIS_IS_CLOSEDLOOP(Z2)
    #define Z2_ENABLE_INIT() NOOP
    #define Z2_ENABLE_WRITE(STATE) stepperZ2.toff((STATE)==Z_ENABLE_ON ? chopper_timing.toff : 0)
    #define Z2_ENABLE_READ() stepperZ2.isEnabled()
  #endif
#endif

// Z3 Stepper
#if HAS_Z3_ENABLE && AXIS_IS_CLOSEDLOOP(Z3)
  extern CLOSEDLOOP_CLASS(Z3, Z) stepperZ3;
  #if ENABLED(SOFTWARE_DRIVER_ENABLE)
    #define Z3_ENABLE_INIT() NOOP
    #define Z3_ENABLE_WRITE(STATE) stepperZ3.toff((STATE)==Z_ENABLE_ON ? chopper_timing.toff : 0)
    #define Z3_ENABLE_READ() stepperZ3.isEnabled()
  #endif
#endif

// Z4 Stepper
#if HAS_Z4_ENABLE && AXIS_IS_CLOSEDLOOP(Z4)
  extern CLOSEDLOOP_CLASS(Z4, Z) stepperZ4;
  #if ENABLED(SOFTWARE_DRIVER_ENABLE)
    #define Z4_ENABLE_INIT() NOOP
    #define Z4_ENABLE_WRITE(STATE) stepperZ4.toff((STATE)==Z_ENABLE_ON ? chopper_timing.toff : 0)
    #define Z4_ENABLE_READ() stepperZ4.isEnabled()
  #endif
#endif

// E0 Stepper
#if AXIS_IS_CLOSEDLOOP(E0)
  extern CLOSEDLOOP_CLASS_E(0) stepperE0;
  #if ENABLED(SOFTWARE_DRIVER_ENABLE) && AXIS_IS_CLOSEDLOOP(E0)
    #define E0_ENABLE_INIT() NOOP
    #define E0_ENABLE_WRITE(STATE) stepperE0.toff((STATE)==E_ENABLE_ON ? chopper_timing.toff : 0)
    #define E0_ENABLE_READ() stepperE0.isEnabled()
  #endif
#endif

// E1 Stepper
#if AXIS_IS_CLOSEDLOOP(E1)
  extern CLOSEDLOOP_CLASS_E(1) stepperE1;
  #if ENABLED(SOFTWARE_DRIVER_ENABLE) && AXIS_IS_CLOSEDLOOP(E1)
    #define E1_ENABLE_INIT() NOOP
    #define E1_ENABLE_WRITE(STATE) stepperE1.toff((STATE)==E_ENABLE_ON ? chopper_timing.toff : 0)
    #define E1_ENABLE_READ() stepperE1.isEnabled()
  #endif
#endif

// E2 Stepper
#if AXIS_IS_CLOSEDLOOP(E2)
  extern CLOSEDLOOP_CLASS_E(2) stepperE2;
  #if ENABLED(SOFTWARE_DRIVER_ENABLE) && AXIS_IS_CLOSEDLOOP(E2)
    #define E2_ENABLE_INIT() NOOP
    #define E2_ENABLE_WRITE(STATE) stepperE2.toff((STATE)==E_ENABLE_ON ? chopper_timing.toff : 0)
    #define E2_ENABLE_READ() stepperE2.isEnabled()
  #endif
#endif

// E3 Stepper
#if AXIS_IS_CLOSEDLOOP(E3)
  extern CLOSEDLOOP_CLASS_E(3) stepperE3;
  #if ENABLED(SOFTWARE_DRIVER_ENABLE) && AXIS_IS_CLOSEDLOOP(E3)
    #define E3_ENABLE_INIT() NOOP
    #define E3_ENABLE_WRITE(STATE) stepperE3.toff((STATE)==E_ENABLE_ON ? chopper_timing.toff : 0)
    #define E3_ENABLE_READ() stepperE3.isEnabled()
  #endif
#endif

// E4 Stepper
#if AXIS_IS_CLOSEDLOOP(E4)
  extern CLOSEDLOOP_CLASS_E(4) stepperE4;
  #if ENABLED(SOFTWARE_DRIVER_ENABLE) && AXIS_IS_CLOSEDLOOP(E4)
    #define E4_ENABLE_INIT() NOOP
    #define E4_ENABLE_WRITE(STATE) stepperE4.toff((STATE)==E_ENABLE_ON ? chopper_timing.toff : 0)
    #define E4_ENABLE_READ() stepperE4.isEnabled()
  #endif
#endif

// E5 Stepper
#if AXIS_IS_CLOSEDLOOP(E5)
  extern CLOSEDLOOP_CLASS_E(5) stepperE5;
  #if ENABLED(SOFTWARE_DRIVER_ENABLE) && AXIS_IS_CLOSEDLOOP(E5)
    #define E5_ENABLE_INIT() NOOP
    #define E5_ENABLE_WRITE(STATE) stepperE5.toff((STATE)==E_ENABLE_ON ? chopper_timing.toff : 0)
    #define E5_ENABLE_READ() stepperE5.isEnabled()
  #endif
#endif

// E6 Stepper
#if AXIS_IS_CLOSEDLOOP(E6)
  extern CLOSEDLOOP_CLASS_E(6) stepperE6;
  #if ENABLED(SOFTWARE_DRIVER_ENABLE) && AXIS_IS_CLOSEDLOOP(E6)
    #define E6_ENABLE_INIT() NOOP
    #define E6_ENABLE_WRITE(STATE) stepperE6.toff((STATE)==E_ENABLE_ON ? chopper_timing.toff : 0)
    #define E6_ENABLE_READ() stepperE6.isEnabled()
  #endif
#endif

// E7 Stepper
#if AXIS_IS_CLOSEDLOOP(E7)
  extern CLOSEDLOOP_CLASS_E(7) stepperE7;
  #if ENABLED(SOFTWARE_DRIVER_ENABLE) && AXIS_IS_CLOSEDLOOP(E7)
    #define E7_ENABLE_INIT() NOOP
    #define E7_ENABLE_WRITE(STATE) stepperE7.toff((STATE)==E_ENABLE_ON ? chopper_timing.toff : 0)
    #define E7_ENABLE_READ() stepperE7.isEnabled()
  #endif
#endif
