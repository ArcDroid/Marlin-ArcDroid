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
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

/***** Z Probe *****/
// Z- / J32
#define Z_MIN_PROBE_PIN PC8

/***** Calibration jig uses extra endstops for probing *****/
// MT_DET1 / J17
#define Y2_MAX_PIN PA4
#define HAS_Y2_MAX 1
// MT_DET2 / J18
#define X2_MIN_PIN PE6
#define HAS_X2_MIN 1
// PW_DET / J19
#define X2_MAX_PIN PD3
#define HAS_X2_MAX 1

#define USB_CONNECT_PIN PD6
#define USB_CONNECT_INVERTING false

#define THC_PIN PC0
#define TH1_PIN PC1

/***** Controller fan used for stepper driver cooling *****/
// FAN2 / J23
#define CONTROLLER_FAN_PIN PB1


/***** Serial ports *****/
// J14 - TFT UART / J16 - TFT RJ45

// J9 - X encoder
#define X_ENCODER_HARDWARE_SERIAL Serial3
#define PIN_SERIAL3_RX          PB11
#define PIN_SERIAL3_TX          PB10

// J10 - Y encoder
#define Y_ENCODER_HARDWARE_SERIAL Serial6
#define PIN_SERIAL6_RX          PC7
#define PIN_SERIAL6_TX          PC6

// or software serials
//#define X_ENCODER_SERIAL_RX_PIN PE14
//#define X_ENCODER_SERIAL_TX_PIN PE15
//#define Y_ENCODER_SERIAL_RX_PIN PD11
//#define Y_ENCODER_SERIAL_TX_PIN PD10

// extra serials
// J11
#define PIN_SERIAL4_RX          PA1
#define PIN_SERIAL4_TX          PA0

/***** Torch output pin *****/
// HEATER1 / J12 3,4 relay
#define SPINDLE_LASER_ENA_PIN PE5
#ifdef MINI_MODEL_2AM
// HEATER2 / J12 1,2 MOSFET
#define SPINDLE_LASER_PWM_PIN PB0
#else
//#define SPINDLE_LASER_PWM_PIN -1
#endif
// T_EN / J16 - TFT RJ45
#define SPINDLE_LASER_INHIBIT_PIN PD13

//
// Include MKS ROBIN NANO V3 pins
//
#include "pins_MKS_ROBIN_NANO_V3.h"

#undef POWER_LOSS_PIN
#undef PW_DET

#undef SD_DETECT_PIN
#define SD_DETECT_PIN PD10

#undef BOARD_INFO_NAME
#define BOARD_INFO_NAME "ArcDroid V1.2-2"
