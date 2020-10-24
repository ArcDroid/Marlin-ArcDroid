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

#define CLOSEDLOOP_INIT(ST) closedloop_init(stepper##ST, ST##_ENCODER_PPU)

#define CLOSEDLOOP_UART_HW_DEFINE(ST, L, AI) ClosedLoopMarlin<L, AI> stepper##ST(&ST##_HARDWARE_SERIAL)
#define CLOSEDLOOP_UART_SW_DEFINE(ST, L, AI) ClosedLoopMarlin<L, AI> stepper##ST(ST##_SERIAL_RX_PIN, ST##_SERIAL_TX_PIN)

#define CLOSEDLOOP_UART_DEFINE(SWHW, ST, AI) CLOSEDLOOP_UART_##SWHW##_DEFINE(ST, CLOSEDLOOP_##ST##_LABEL, AI##_AXIS)

#if DISTINCT_E > 1
  #define CLOSEDLOOP_UART_DEFINE_E(SWHW, AI) CLOSEDLOOP_UART_DEFINE(SWHW, E##AI, E##AI)
#else
  #define CLOSEDLOOP_UART_DEFINE_E(SWHW, AI) CLOSEDLOOP_UART_DEFINE(SWHW, E##AI, E)
#endif


#ifndef CLOSEDLOOP_BAUD_RATE
  #define CLOSEDLOOP_BAUD_RATE 19200
#endif


#if HAS_DRIVER(CLOSEDLOOP)
  #if AXIS_IS_CLOSEDLOOP(X)
    #ifdef X_HARDWARE_SERIAL
      CLOSEDLOOP_UART_DEFINE(HW, X, X);
    #else
      CLOSEDLOOP_UART_DEFINE(SW, X, X);
    #endif
  #endif
  #if AXIS_IS_CLOSEDLOOP(X2)
    #ifdef X2_HARDWARE_SERIAL
      CLOSEDLOOP_UART_DEFINE(HW, X2, X);
    #else
      CLOSEDLOOP_UART_DEFINE(SW, X2, X);
    #endif
  #endif
  #if AXIS_IS_CLOSEDLOOP(Y)
    #ifdef Y_HARDWARE_SERIAL
      CLOSEDLOOP_UART_DEFINE(HW, Y, Y);
    #else
      CLOSEDLOOP_UART_DEFINE(SW, Y, Y);
    #endif
  #endif
  #if AXIS_IS_CLOSEDLOOP(Y2)
    #ifdef Y2_HARDWARE_SERIAL
      CLOSEDLOOP_UART_DEFINE(HW, Y2, Y);
    #else
      CLOSEDLOOP_UART_DEFINE(SW, Y2, Y);
    #endif
  #endif
  #if AXIS_IS_CLOSEDLOOP(Z)
    #ifdef Z_HARDWARE_SERIAL
      CLOSEDLOOP_UART_DEFINE(HW, Z, Z);
    #else
      CLOSEDLOOP_UART_DEFINE(SW, Z, Z);
    #endif
  #endif
  #if AXIS_IS_CLOSEDLOOP(Z2)
    #ifdef Z2_HARDWARE_SERIAL
      CLOSEDLOOP_UART_DEFINE(HW, Z2, Z);
    #else
      CLOSEDLOOP_UART_DEFINE(SW, Z2, Z);
    #endif
  #endif
  #if AXIS_IS_CLOSEDLOOP(Z3)
    #ifdef Z3_HARDWARE_SERIAL
      CLOSEDLOOP_UART_DEFINE(HW, Z3, Z);
    #else
      CLOSEDLOOP_UART_DEFINE(SW, Z3, Z);
    #endif
  #endif
  #if AXIS_IS_CLOSEDLOOP(Z4)
    #ifdef Z4_HARDWARE_SERIAL
      CLOSEDLOOP_UART_DEFINE(HW, Z4, Z);
    #else
      CLOSEDLOOP_UART_DEFINE(SW, Z4, Z);
    #endif
  #endif
  #if AXIS_IS_CLOSEDLOOP(E0)
    #ifdef E0_HARDWARE_SERIAL
      CLOSEDLOOP_UART_DEFINE_E(HW, 0);
    #else
      CLOSEDLOOP_UART_DEFINE_E(SW, 0);
    #endif
  #endif
  #if AXIS_IS_CLOSEDLOOP(E1)
    #ifdef E1_HARDWARE_SERIAL
      CLOSEDLOOP_UART_DEFINE_E(HW, 1);
    #else
      CLOSEDLOOP_UART_DEFINE_E(SW, 1);
    #endif
  #endif
  #if AXIS_IS_CLOSEDLOOP(E2)
    #ifdef E2_HARDWARE_SERIAL
      CLOSEDLOOP_UART_DEFINE_E(HW, 2);
    #else
      CLOSEDLOOP_UART_DEFINE_E(SW, 2);
    #endif
  #endif
  #if AXIS_IS_CLOSEDLOOP(E3)
    #ifdef E3_HARDWARE_SERIAL
      CLOSEDLOOP_UART_DEFINE_E(HW, 3);
    #else
      CLOSEDLOOP_UART_DEFINE_E(SW, 3);
    #endif
  #endif
  #if AXIS_IS_CLOSEDLOOP(E4)
    #ifdef E4_HARDWARE_SERIAL
      CLOSEDLOOP_UART_DEFINE_E(HW, 4);
    #else
      CLOSEDLOOP_UART_DEFINE_E(SW, 4);
    #endif
  #endif
  #if AXIS_IS_CLOSEDLOOP(E5)
    #ifdef E5_HARDWARE_SERIAL
      CLOSEDLOOP_UART_DEFINE_E(HW, 5);
    #else
      CLOSEDLOOP_UART_DEFINE_E(SW, 5);
    #endif
  #endif
  #if AXIS_IS_CLOSEDLOOP(E6)
    #ifdef E6_HARDWARE_SERIAL
      CLOSEDLOOP_UART_DEFINE_E(HW, 6);
    #else
      CLOSEDLOOP_UART_DEFINE_E(SW, 6);
    #endif
  #endif
  #if AXIS_IS_CLOSEDLOOP(E7)
    #ifdef E7_HARDWARE_SERIAL
      CLOSEDLOOP_UART_DEFINE_E(HW, 7);
    #else
      CLOSEDLOOP_UART_DEFINE_E(SW, 7);
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

      #define HW_SERIAL_BEGIN(A) do{ if (!sp_helper.began(CLOSEDLOOPAxis::A, &A##_HARDWARE_SERIAL)) \
                                          A##_HARDWARE_SERIAL.begin(CLOSEDLOOP_BAUD_RATE); }while(0)
    #endif

    #if AXIS_IS_CLOSEDLOOP(X)
      #ifdef X_HARDWARE_SERIAL
        HW_SERIAL_BEGIN(X);
      #else
        stepperX.beginSerial(CLOSEDLOOP_BAUD_RATE);
      #endif
    #endif
    #if AXIS_IS_CLOSEDLOOP(X2)
      #ifdef X2_HARDWARE_SERIAL
        HW_SERIAL_BEGIN(X2);
      #else
        stepperX2.beginSerial(CLOSEDLOOP_BAUD_RATE);
      #endif
    #endif
    #if AXIS_IS_CLOSEDLOOP(Y)
      #ifdef Y_HARDWARE_SERIAL
        HW_SERIAL_BEGIN(Y);
      #else
        stepperY.beginSerial(CLOSEDLOOP_BAUD_RATE);
      #endif
    #endif
    #if AXIS_IS_CLOSEDLOOP(Y2)
      #ifdef Y2_HARDWARE_SERIAL
        HW_SERIAL_BEGIN(Y2);
      #else
        stepperY2.beginSerial(CLOSEDLOOP_BAUD_RATE);
      #endif
    #endif
    #if AXIS_IS_CLOSEDLOOP(Z)
      #ifdef Z_HARDWARE_SERIAL
        HW_SERIAL_BEGIN(Z);
      #else
        stepperZ.beginSerial(CLOSEDLOOP_BAUD_RATE);
      #endif
    #endif
    #if AXIS_IS_CLOSEDLOOP(Z2)
      #ifdef Z2_HARDWARE_SERIAL
        HW_SERIAL_BEGIN(Z2);
      #else
        stepperZ2.beginSerial(CLOSEDLOOP_BAUD_RATE);
      #endif
    #endif
    #if AXIS_IS_CLOSEDLOOP(Z3)
      #ifdef Z3_HARDWARE_SERIAL
        HW_SERIAL_BEGIN(Z3);
      #else
        stepperZ3.beginSerial(CLOSEDLOOP_BAUD_RATE);
      #endif
    #endif
    #if AXIS_IS_CLOSEDLOOP(Z4)
      #ifdef Z4_HARDWARE_SERIAL
        HW_SERIAL_BEGIN(Z4);
      #else
        stepperZ4.beginSerial(CLOSEDLOOP_BAUD_RATE);
      #endif
    #endif
    #if AXIS_IS_CLOSEDLOOP(E0)
      #ifdef E0_HARDWARE_SERIAL
        HW_SERIAL_BEGIN(E0);
      #else
        stepperE0.beginSerial(CLOSEDLOOP_BAUD_RATE);
      #endif
    #endif
    #if AXIS_IS_CLOSEDLOOP(E1)
      #ifdef E1_HARDWARE_SERIAL
        HW_SERIAL_BEGIN(E1);
      #else
        stepperE1.beginSerial(CLOSEDLOOP_BAUD_RATE);
      #endif
    #endif
    #if AXIS_IS_CLOSEDLOOP(E2)
      #ifdef E2_HARDWARE_SERIAL
        HW_SERIAL_BEGIN(E2);
      #else
        stepperE2.beginSerial(CLOSEDLOOP_BAUD_RATE);
      #endif
    #endif
    #if AXIS_IS_CLOSEDLOOP(E3)
      #ifdef E3_HARDWARE_SERIAL
        HW_SERIAL_BEGIN(E3);
      #else
        stepperE3.beginSerial(CLOSEDLOOP_BAUD_RATE);
      #endif
    #endif
    #if AXIS_IS_CLOSEDLOOP(E4)
      #ifdef E4_HARDWARE_SERIAL
        HW_SERIAL_BEGIN(E4);
      #else
        stepperE4.beginSerial(CLOSEDLOOP_BAUD_RATE);
      #endif
    #endif
    #if AXIS_IS_CLOSEDLOOP(E5)
      #ifdef E5_HARDWARE_SERIAL
        HW_SERIAL_BEGIN(E5);
      #else
        stepperE5.beginSerial(CLOSEDLOOP_BAUD_RATE);
      #endif
    #endif
    #if AXIS_IS_CLOSEDLOOP(E6)
      #ifdef E6_HARDWARE_SERIAL
        HW_SERIAL_BEGIN(E6);
      #else
        stepperE6.beginSerial(CLOSEDLOOP_BAUD_RATE);
      #endif
    #endif
    #if AXIS_IS_CLOSEDLOOP(E7)
      #ifdef E7_HARDWARE_SERIAL
        HW_SERIAL_BEGIN(E7);
      #else
        stepperE7.beginSerial(CLOSEDLOOP_BAUD_RATE);
      #endif
    #endif
  }
#endif

#if HAS_DRIVER(CLOSEDLOOP)
  template<char AXIS_LETTER, char DRIVER_ID, AxisEnum AXIS_ID>
  void closedloop_init(ClosedLoopMarlin<AXIS_LETTER, DRIVER_ID, AXIS_ID> &st, float encoder_counts_per_unit) {
    st.encoder_counts_per_unit = encoder_counts_per_unit;
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
  #if AXIS_IS_CLOSEDLOOP(X2)
    CLOSEDLOOP_INIT(X2);
  #endif
  #if AXIS_IS_CLOSEDLOOP(Y)
    CLOSEDLOOP_INIT(Y);
  #endif
  #if AXIS_IS_CLOSEDLOOP(Y2)
    CLOSEDLOOP_INIT(Y2);
  #endif
  #if AXIS_IS_CLOSEDLOOP(Z)
    CLOSEDLOOP_INIT(Z);
  #endif
  #if AXIS_IS_CLOSEDLOOP(Z2)
    CLOSEDLOOP_INIT(Z2);
  #endif
  #if AXIS_IS_CLOSEDLOOP(Z3)
    CLOSEDLOOP_INIT(Z3);
  #endif
  #if AXIS_IS_CLOSEDLOOP(Z4)
    CLOSEDLOOP_INIT(Z4);
  #endif
  #if AXIS_IS_CLOSEDLOOP(E0)
    CLOSEDLOOP_INIT(E0);
  #endif
  #if AXIS_IS_CLOSEDLOOP(E1)
    CLOSEDLOOP_INIT(E1);
  #endif
  #if AXIS_IS_CLOSEDLOOP(E2)
    CLOSEDLOOP_INIT(E2);
  #endif
  #if AXIS_IS_CLOSEDLOOP(E3)
    CLOSEDLOOP_INIT(E3);
  #endif
  #if AXIS_IS_CLOSEDLOOP(E4)
    CLOSEDLOOP_INIT(E4);
  #endif
  #if AXIS_IS_CLOSEDLOOP(E5)
    CLOSEDLOOP_INIT(E5);
  #endif
  #if AXIS_IS_CLOSEDLOOP(E6)
    CLOSEDLOOP_INIT(E6);
  #endif
  #if AXIS_IS_CLOSEDLOOP(E7)
    CLOSEDLOOP_INIT(E7);
  #endif

  stepper.set_directions();
}

void closedloop_home_encoders(abce_pos_t motor_pos) {
    #if AXIS_IS_CLOSEDLOOP(X)
        stepperX.touch_off_encoder(motor_pos.x);
    #endif
    #if AXIS_IS_CLOSEDLOOP(Y)
        stepperY.touch_off_encoder(motor_pos.y);
    #endif
}

bool closedloop_need_restore() {
    return false
    #if AXIS_IS_CLOSEDLOOP(X)
        || !TEST(axis_known_position, X_AXIS) && stepperX.homed
    #endif
    #if AXIS_IS_CLOSEDLOOP(Y)
        || !TEST(axis_known_position, Y_AXIS) && stepperY.homed
    #endif
    ;
}

void closedloop_restore_position(abce_pos_t *motor_pos, bool enable) {
    bool enabled_any = false;
    #if AXIS_IS_CLOSEDLOOP(X)
        if (!TEST(axis_known_position, X_AXIS) && stepperX.homed) {
            motor_pos->x = stepperX.read_encoder();
            if (enable) {
                ENABLE_STEPPER_X();
                SBI(axis_known_position, X_AXIS);
                enabled_any = true;
            }
        }
    #endif
    #if AXIS_IS_CLOSEDLOOP(Y)
        if (!TEST(axis_known_position, Y_AXIS) && stepperY.homed) {
            motor_pos->y = stepperY.read_encoder();
            if (enable) {
                ENABLE_STEPPER_Y();
                SBI(axis_known_position, Y_AXIS);
                enabled_any = true;
            }
        }
    #endif
    if (enabled_any) {
        stepper.set_directions();
        delayMicroseconds(100);
    }
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
		if (res < 0) continue; // nothing in serial buffer

		return res;
    } while (true);
}

int16_t S42BClosedLoop::sendCommand(const uint8_t function, const uint16_t data, uint8_t respBuffer[], const uint8_t rBufSize, uint16_t timeout) {
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

int32_t S42BClosedLoop::readPosition() {
    uint8_t buff[4];
    int16_t res;
    preCommunication();
    for (uint8_t retries = 0; retries < 5; retries++) {
        delay(2);
        res = sendCommand(0x37, 0xaaaa, buff, 4, 40);

        if (res < 0)
            continue;
        postCommunication();

        int32_t position = (buff[0] << 24) | (buff[1] << 16) | (buff[2] << 8) | buff[3];
        return position;
    }
    postCommunication();

    return 0x7f000000 - res ; // read failed
}

#endif // HAS_CLOSEDLOOP_CONFIG
