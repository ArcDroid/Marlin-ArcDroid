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
#define Z_MIN_PROBE_PIN P1_26

/***** Calibration jig uses extra endstops for probing *****/
// E1DET connector on SKR 1.4
#define Y2_MAX_PIN P1_25
#define HAS_Y2_MAX 1
// PWRDET connector on SKR 1.4
#define X2_MIN_PIN P1_00
#define HAS_X2_MIN 1
// P1.20 on EXP1 connector on SKR 1.4
#define X2_MAX_PIN P1_20
#define HAS_X2_MAX 1

#define THC_PIN P0_25_A2
#define TH1_PIN P0_23_A0

/***** Controller fan used for stepper driver cooling *****/
#define CONTROLLER_FAN_PIN P2_03


/***** Encoder serial ports *****/
// EXP2 0.15 = TXD1, 0.16 = RXD1
#define X_ENCODER_HARDWARE_SERIAL Serial1
// Serial2 0.11 = RXD2 on E0-CLS, 0.10 = TXD2 on PROBE
#define Y_ENCODER_HARDWARE_SERIAL Serial2
// Serial3 WiFi 4.28 = TXD3, 4.29 = RXD3
//#define LPC_PINCFG_UART3_P4_28
//#define Y_ENCODER_HARDWARE_SERIAL Serial3

// or software serials
//#define X_ENCODER_SERIAL_RX_PIN P1_23
//#define X_ENCODER_SERIAL_TX_PIN P1_22
//#define Y_ENCODER_SERIAL_RX_PIN P1_21
//#define Y_ENCODER_SERIAL_TX_PIN P1_20

/***** Torch output pin *****/
#ifdef MINI_MODEL_2AM
#define SPINDLE_LASER_ENA_PIN P1_31
#define SPINDLE_LASER_PWM_PIN P2_05
#else
#define SPINDLE_LASER_ENA_PIN P2_05
//#define SPINDLE_LASER_PWM_PIN 2
#endif

#define SPINDLE_LASER_INHIBIT_PIN P1_22

//
// Include SKR 1.4 TURBO pins
//
#define REQUIRE_LPC1769
#include "pins_BTT_SKR_V1_4_TURBO.h"

#undef BOARD_INFO_NAME
#define BOARD_INFO_NAME "BTT SKR V1.4 TURBO for ArcDroid"
