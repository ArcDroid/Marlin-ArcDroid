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

/**
 * stepper/trinamic.cpp
 * Stepper driver indirection for smart closedloop drivers like BTT S42B
 */

#include "../../inc/MarlinConfig.h"

#if HAS_CLOSEDLOOP_CONFIG

#include "closedloop.h"
#include "../stepper.h"


#include <HardwareSerial.h>

#define CLOSEDLOOP_INIT(ST) closedloop_init(encoder##ST, ST##_ENCODER_PPS, ST##_ENCODER_PPR)

#define CLOSEDLOOP_UART_HW_DEFINE(ST, L, AI) ClosedLoopMarlin<L, AI> encoder##ST(&ST##_ENCODER_HARDWARE_SERIAL)
#define CLOSEDLOOP_UART_SW_DEFINE(ST, L, AI) ClosedLoopMarlin<L, AI> encoder##ST(ST##_ENCODER_SERIAL_RX_PIN, ST##_ENCODER_SERIAL_TX_PIN)

#define CLOSEDLOOP_UART_DEFINE(SWHW, ST, AI) CLOSEDLOOP_UART_##SWHW##_DEFINE(ST, CLOSEDLOOP_##ST##_LABEL, AI##_AXIS)

#if DISTINCT_E > 1
  #define CLOSEDLOOP_UART_DEFINE_E(SWHW, AI) CLOSEDLOOP_UART_DEFINE(SWHW, E##AI, E##AI)
#else
  #define CLOSEDLOOP_UART_DEFINE_E(SWHW, AI) CLOSEDLOOP_UART_DEFINE(SWHW, E##AI, E)
#endif


#ifndef CLOSEDLOOP_BAUD_RATE
  #define CLOSEDLOOP_BAUD_RATE 115200
#endif


#if HAS_CLOSEDLOOP_CONFIG
  #if X_ENCODER_TYPE == _CL_S42B
    #ifdef X_ENCODER_HARDWARE_SERIAL
      CLOSEDLOOP_UART_DEFINE(HW, X, X);
    #else
      CLOSEDLOOP_UART_DEFINE(SW, X, X);
    #endif
  #endif
  #if Y_ENCODER_TYPE == _CL_S42B
    #ifdef Y_ENCODER_HARDWARE_SERIAL
      CLOSEDLOOP_UART_DEFINE(HW, Y, Y);
    #else
      CLOSEDLOOP_UART_DEFINE(SW, Y, Y);
    #endif
  #endif

  enum CLOSEDLOOPAxis : uint8_t { X, Y, Z, X2, Y2, Z2, Z3, Z4, E0, E1, E2, E3, E4, E5, E6, E7, TOTAL };

  void closedloop_serial_begin() {
    #if HAS_CLOSEDLOOP_HW_SERIAL
      struct {
        const void *ptr[CLOSEDLOOPAxis::TOTAL];
        bool began(const CLOSEDLOOPAxis a, const void * const p) {
          LOOP_L_N(i, a) if (p == ptr[i]) return true;
          ptr[a] = p; return false;
        };
      } sp_helper;

      #define HW_SERIAL_BEGIN(A) do{ if (!sp_helper.began(CLOSEDLOOPAxis::A, &A##_ENCODER_HARDWARE_SERIAL)) \
                                          A##_ENCODER_HARDWARE_SERIAL.begin(CLOSEDLOOP_BAUD_RATE); }while(0)
    #endif

    #if AXIS_IS_CLOSEDLOOP(X)
      #ifdef X_ENCODER_HARDWARE_SERIAL
        HW_SERIAL_BEGIN(X);
      #else
        encoderX.beginSerial(CLOSEDLOOP_BAUD_RATE);
      #endif
    #endif
    #if AXIS_IS_CLOSEDLOOP(Y)
      #ifdef Y_ENCODER_HARDWARE_SERIAL
        HW_SERIAL_BEGIN(Y);
      #else
        encoderY.beginSerial(CLOSEDLOOP_BAUD_RATE);
      #endif
    #endif
  }
#endif

#if HAS_CLOSEDLOOP_CONFIG
  template<char AXIS_LETTER, char DRIVER_ID, AxisEnum AXIS_ID>
  void closedloop_init(ClosedLoopMarlin<AXIS_LETTER, DRIVER_ID, AXIS_ID> &st, float encoder_counts_per_step, int32_t encoder_counts_per_rev) {
    st.encoder_counts_per_step = encoder_counts_per_step;
    st.encoder_counts_per_rev = encoder_counts_per_rev;
    st.homed = false;
    st.check_comms();
    delay(200);
  }
#endif // CLOSEDLOOP

void restore_closedloop_drivers() {

  // mark angle offset as inaccurate here until homing?
  #if AXIS_IS_CLOSEDLOOP(X)
    CLOSEDLOOP_INIT(X);
  #endif
  #if AXIS_IS_CLOSEDLOOP(Y)
    CLOSEDLOOP_INIT(Y);
  #endif

  stepper.set_directions();
}

void closedloop_home_encoders(AxisEnum axis, abce_pos_t motor_pos, bool calibrate_home) {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
        DEBUG_ECHOPAIR("closedloop_home_encoders axis: ", axis);
        DEBUG_EOL();
    }
    #endif
    #if AXIS_IS_CLOSEDLOOP(X)
      if (axis && _BV(X_AXIS)) {
        encoderX.touch_off_encoder(motor_pos.x, calibrate_home);
        // set_position_from_encoders_force will pick it back up
        set_axis_untrusted(X_AXIS);
      }
    #endif
    #if AXIS_IS_CLOSEDLOOP(Y)
      if (axis && _BV(Y_AXIS)) {
        encoderY.touch_off_encoder(motor_pos.y, calibrate_home);
        set_axis_untrusted(Y_AXIS);
      }
    #endif

    set_position_from_encoders_force(false);
}

bool closedloop_has_aligned() {
    return true
    #if AXIS_IS_CLOSEDLOOP(X)
        && encoderX.homed
    #endif
    #if AXIS_IS_CLOSEDLOOP(Y)
        && encoderY.homed
    #endif
    ;
}

bool closedloop_need_restore() {
    return false
    #if AXIS_IS_CLOSEDLOOP(X)
        || (!axis_is_trusted(X_AXIS) && encoderX.homed)
    #endif
    #if AXIS_IS_CLOSEDLOOP(Y)
        || (!axis_is_trusted(Y_AXIS) && encoderY.homed)
    #endif
    ;
}

void closedloop_unhome(AxisEnum axis) {
    #if AXIS_IS_CLOSEDLOOP(X)
        if (axis && _BV(X_AXIS))
            encoderX.homed = false;
    #endif
    #if AXIS_IS_CLOSEDLOOP(Y)
        if (axis && _BV(Y_AXIS))
            encoderY.homed = false;
    #endif
}

bool closedloop_restore_position(abce_pos_t *motor_pos, bool enable) {
    bool enabled_any = false;
    bool valid_position = true;
    #if AXIS_IS_CLOSEDLOOP(X)
        if (!axis_is_trusted(X_AXIS) && encoderX.homed) {
            motor_pos->x = encoderX.read_encoder();
            if (isnan(motor_pos->x)) {
                valid_position = false;
                encoderX.homed = false;
                set_axis_never_homed(X_AXIS);
            }
            else if (enable) {
                ENABLE_STEPPER_X();
                set_axis_trusted(X_AXIS);
                enabled_any = true;
            }
        }
    #endif
    #if AXIS_IS_CLOSEDLOOP(Y)
        if (!axis_is_trusted(Y_AXIS) && encoderY.homed) {
            motor_pos->y = encoderY.read_encoder();
            if (isnan(motor_pos->y)) {
                valid_position = false;
                encoderY.homed = false;
                set_axis_never_homed(Y_AXIS);
            }
            else if (enable) {
                ENABLE_STEPPER_Y();
                set_axis_trusted(Y_AXIS);
                enabled_any = true;
            }
        }
    #endif
    if (enabled_any) {
        stepper.set_directions();
        delayMicroseconds(100);
    }

    if (!valid_position) {
        SERIAL_ERROR_MSG("encoder ", "error "
        #if AXIS_IS_CLOSEDLOOP(X)
            ,"X:", encoderX.last_error
        #endif
        #if AXIS_IS_CLOSEDLOOP(Y)
            ,"Y:", encoderY.last_error
        #endif
            );
    }

    return valid_position;
}

void closedloop_reset_pps() {
    #if AXIS_IS_CLOSEDLOOP(X)
		encoderX.encoder_counts_per_step = X_ENCODER_PPS;
	#endif
    #if AXIS_IS_CLOSEDLOOP(Y)
		encoderY.encoder_counts_per_step = Y_ENCODER_PPS;
	#endif
}

void closedloop_set_pps(abce_float_t pps) {
  DEBUG_ECHO_MSG("closedloop_set_pps x:", pps.x, " y:", pps.y, "\n");
    #if AXIS_IS_CLOSEDLOOP(X)
		encoderX.encoder_counts_per_step = pps.x;
	#endif
    #if AXIS_IS_CLOSEDLOOP(Y)
		encoderY.encoder_counts_per_step = pps.y;
	#endif
}

abce_float_t closedloop_get_pps() {
	abce_float_t res;
    #if AXIS_IS_CLOSEDLOOP(X)
		res.x = encoderX.encoder_counts_per_step;
	#endif
    #if AXIS_IS_CLOSEDLOOP(Y)
		res.y = encoderY.encoder_counts_per_step;
	#endif

  DEBUG_ECHO_MSG("closedloop_get_pps x:", res.x, " y:", res.y, "\n");
	return res;
}

void closedloop_reset_home_pulse() {
    #if AXIS_IS_CLOSEDLOOP(X)
		encoderX.home_pulse = -1;
	#endif
    #if AXIS_IS_CLOSEDLOOP(Y)
		encoderY.home_pulse = -1;
	#endif
}

void closedloop_set_home_pulse(abc_long_t home) {
    #if AXIS_IS_CLOSEDLOOP(X)
		encoderX.home_pulse = home.x;
	#endif
    #if AXIS_IS_CLOSEDLOOP(Y)
		encoderY.home_pulse = home.y;
	#endif
}

abc_long_t closedloop_get_home_pulse() {
	abc_long_t res;
    #if AXIS_IS_CLOSEDLOOP(X)
		res.x = encoderX.home_pulse;
	#endif
    #if AXIS_IS_CLOSEDLOOP(Y)
		res.y = encoderY.home_pulse;
	#endif

	return res;
}

S42BClosedLoop::S42BClosedLoop(Stream * SerialPort) {
    HWSerial = SerialPort;
}

#if SW_CAPABLE_PLATFORM

    S42BClosedLoop::S42BClosedLoop(uint16_t SW_RX_pin, uint16_t SW_TX_pin)
    {
        SoftwareSerial *SWSerialObj = new SoftwareSerial(SW_RX_pin, SW_TX_pin);
        SWSerial = SWSerialObj;
    }

    void S42BClosedLoop::beginSerial(uint32_t baudrate) {
		if (SWSerial != nullptr)
		{
			SWSerial->begin(baudrate);
			SWSerial->stopListening();
		}
	}
#endif

template<char AXIS_LETTER, char DRIVER_ID, AxisEnum AXIS_ID>
void ClosedLoopMarlin<AXIS_LETTER, DRIVER_ID, AXIS_ID>::check_comms() {

}


__attribute__((weak))
int S42BClosedLoop::available() {
	int out = 0;
	#if SW_CAPABLE_PLATFORM
		if (SWSerial != nullptr) {
			out = SWSerial->available();
		} else
	#endif
		if (HWSerial != nullptr) {
			out = HWSerial->available();
		}

	return out;
}

__attribute__((weak))
void S42BClosedLoop::preCommunication() {
	#if SW_CAPABLE_PLATFORM
		if (SWSerial != nullptr) {
			SWSerial->listen();
		}
	#endif
}

__attribute__((weak))
int16_t S42BClosedLoop::serial_read() {
	int16_t out = 0;
	#if SW_CAPABLE_PLATFORM
		if (SWSerial != nullptr) {
			out = SWSerial->read();
		} else
	#endif
		if (HWSerial != nullptr) {
			out = HWSerial->read();
		}

	return out;
}

__attribute__((weak))
uint8_t S42BClosedLoop::serial_write(const uint8_t data) {
	int out = 0;;
	#if SW_CAPABLE_PLATFORM
		if (SWSerial != nullptr) {
			return SWSerial->write(data);
		} else
	#endif
		if (HWSerial != nullptr) {
			return HWSerial->write(data);
		}

	return out;
}

__attribute__((weak))
void S42BClosedLoop::postCommunication() {
	#if SW_CAPABLE_PLATFORM
		if (SWSerial != nullptr) {
			SWSerial->stopListening();
		}
	#endif
}

uint8_t S42BClosedLoop::calcChecksum(uint8_t l_val, uint8_t f_val, uint8_t buffer[], uint8_t start, uint8_t len) {
    return calcChecksum(buffer, start, len) + l_val + f_val;
}

uint8_t S42BClosedLoop::calcChecksum(uint8_t buffer[], uint8_t start, uint8_t len) {
	uint8_t crc = 0;
	for (uint8_t i = start; i < start + len; i++) {
		uint8_t currentByte = buffer[i];
		crc += currentByte;
	}
	return crc;
}

int16_t S42BClosedLoop::serial_read_timed(uint32_t millis_deadline) {
    do {
        uint32_t time = millis();
		if (time > millis_deadline)
		    return -1; // timeout waiting for response

		int16_t res = serial_read();
		if (res < 0) {
            ////SERIAL_ECHOPAIR("res=", res, "time", time, "millis_deadline", millis_deadline);
            idle();
            continue; // nothing in serial buffer
        }

		return res;
    } while (true);
}

void S42BClosedLoop::sendCommandBase(const uint8_t function, const uint16_t data) {
    uint8_t dh = data >> 8;
    uint8_t dl = data & 0xFF;
	uint8_t command[] = {0xfe, 0xfe, 0x05, function, dh, dl, 0, 0x16};
    uint8_t len = sizeof(command);

	command[6] = calcChecksum(command, 2, 5);

	while (available() > 0) serial_read(); // Flush

	for(uint8_t i = 0; i < len; i++) {
		bytesWritten += serial_write(command[i]);
	}
	delay(replyDelay);
}

int16_t S42BClosedLoop::sendCommandBinary(const uint8_t function, const uint16_t data, uint8_t respBuffer[], const uint8_t rBufSize, uint16_t timeout) {
	sendCommandBase(function, data);

	// scan for the rx frame and read it
	uint32_t deadline = millis() + timeout;
	uint16_t sync_target = 0xfefe;
	uint16_t sync = 0;

	do {
		int16_t res = serial_read_timed(deadline);
		if (res < 0) return -1; // timeout while waiting for start word

		sync <<= 8;
		sync |= res & 0xFF;

	} while (sync != sync_target);

	deadline = millis() + this->abort_window;

    int16_t res_length = serial_read_timed(deadline);
    int16_t res_function = serial_read_timed(deadline);
    if (res_length < 0 || res_function < 0)
        return -2; // timeout reading response

    if (res_length - 3 != rBufSize)
        return -5; // unexpected response size

    if (res_function != function)
        return -6; // response function doesn't match request

    uint8_t read;
	for(read=0; read<rBufSize;) {
		int16_t res = serial_read_timed(deadline);
		if (res < 0) break; // timed out waiting for next character

		respBuffer[read] = res;
		read++;
	}

    if (read == 0 && rBufSize != 0)
        return -2; // timeout reading response

    int16_t checksum = serial_read_timed(deadline);
    int16_t tail = serial_read_timed(deadline);

    if (checksum < 0 || tail < 0)
        return -2; // timed out reading last bytes

    if (tail != 0x16)
        return -3; // bad tail byte

    uint8_t expected = calcChecksum((uint8_t)res_length, (uint8_t)res_function, respBuffer, 0, read);
    if (checksum != expected)
        return -4; // crc error

    while (available() > 0) serial_read(); // Flush

	return read;
}

int32_t S42BClosedLoop::readPosition(bool onetry) {
    uint8_t buff[4];
    int16_t res = 0;
    preCommunication();
    for (uint8_t retries = 0; retries < (onetry ? 1 : 30); retries++) {
        delay(2);
        res = sendCommandBinary(0x37, 0xaaaa, buff, 4, 40);

        if (res < 0)
            continue;
        postCommunication();

        int32_t position = (buff[0] << 24) | (buff[1] << 16) | (buff[2] << 8) | buff[3];
        last_error = 0;
        return position;
    }
    postCommunication();

    last_error = res;
    return 0x7f000000 - res ; // read failed
}

bool S42BClosedLoop::userCommand(uint8_t function, uint16_t data) {
    preCommunication();
    delay(2);

    sendCommandBase(function, data);
    uint32_t timeout = 50;

	// scan for the rx frame and read it
	uint32_t deadline = millis() + timeout;

	do {
		int16_t res = serial_read_timed(deadline);
		if (res <= 0) {
            break; // timeout while waiting for start word
        }
        else {
            deadline = millis() + timeout;
            SERIAL_CHAR(res);
        }
	} while (true);

    postCommunication();

    return true;
}

#endif // HAS_CLOSEDLOOP_CONFIG
