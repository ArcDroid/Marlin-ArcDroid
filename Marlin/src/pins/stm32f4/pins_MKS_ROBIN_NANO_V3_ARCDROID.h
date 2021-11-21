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
// E0DET connector on SKR 1.4
#define Z_MIN_PROBE_PIN PC8

/***** Calibration jig uses extra endstops for probing *****/
// E1DET connector on SKR 1.4
#define Y2_MAX_PIN PA4
#define HAS_Y2_MAX 1
// PWRDET connector on SKR 1.4
#define X2_MIN_PIN PE6
#define HAS_X2_MIN 1
// P1.20 on EXP1 connector on SKR 1.4
#define X2_MAX_PIN PA13
#define HAS_X2_MAX 1

/***** Controller fan used for stepper driver cooling *****/
#define CONTROLLER_FAN_PIN PB1


/***** Encoder serial ports *****/
#define X_ENCODER_HARDWARE_SERIAL Serial3
#define PIN_SERIAL3_RX          PB11
#define PIN_SERIAL3_TX          PB10

#define Y_ENCODER_HARDWARE_SERIAL Serial6
#define PIN_SERIAL6_RX          PC7
#define PIN_SERIAL6_TX          PC6

// or software serials
//#define X_ENCODER_SERIAL_RX_PIN PE14
//#define X_ENCODER_SERIAL_TX_PIN PE15
//#define Y_ENCODER_SERIAL_RX_PIN PD11
//#define Y_ENCODER_SERIAL_TX_PIN PD10

/***** Torch output pin *****/
#ifdef MINI_MODEL_2AM
#define SPINDLE_LASER_ENA_PIN PB0
#define SPINDLE_LASER_PWM_PIN PA0
#else
#define SPINDLE_LASER_ENA_PIN PA0
//#define SPINDLE_LASER_PWM_PIN 2
#endif

#define SPINDLE_LASER_INHIBIT_PIN PD13

//
// Include MKS ROBIN NANO V3 pins
//
#include "pins_MKS_ROBIN_NANO_V3.h"

#undef POWER_LOSS_PIN
#undef PW_DET

#undef BOARD_INFO_NAME
#define BOARD_INFO_NAME "MKS Robin Nano V3 for ArcDroid"
