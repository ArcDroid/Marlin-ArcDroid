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

#include "../../../inc/MarlinConfig.h"

#if HAS_CLOSEDLOOP_CONFIG

#include "../../gcode.h"
#include "../../../feature/tmc_util.h"
#include "../../../module/stepper/indirection.h"

void send_axis_commands(uint8_t function, uint16_t data) {
  LOOP_LINEAR_AXES(i) if (parser.seen(axis_codes[i])) {
    switch (i) {
      case X_AXIS:
        #if AXIS_IS_CLOSEDLOOP(X)
          SERIAL_ECHOPGM("ClosedLoop X:\n");
          encoderX.userCommand(function, data);
        #endif
        break;
      case Y_AXIS:
        #if AXIS_IS_CLOSEDLOOP(Y)
          SERIAL_ECHOPGM("ClosedLoop Y:\n");
          encoderY.userCommand(function, data);
        #endif
        break;
    }
  }
}

/**
 * M927: Send CL_S42B raw command
 *
 *   XYZE to target an axis
 *   P command
 *   Q value
 */
void GcodeSuite::M927() {
  uint8_t function = 0;
  uint16_t data = 0;

  if (parser.seen('P')) {
    function = parser.intval('P', 0xB0);
  }

  if (parser.seen('Q')) {
    data = parser.intval('Q');
  }

  if (function == 0)
    return;

  send_axis_commands(function, data);
}

/**
 * M926: Start CL_S42B calibration
 *
 *   XYZE to target an axis
 */
void GcodeSuite::M926() {
  send_axis_commands(0xaf, 0x11);
}

/**
 * M925: Configure CL_S42B driver
 *
 *   XYZE to target an axis
 *   M[1|0] set mode to Closed/Open loop
 *   R set cuRrent
 *   P set PID P
 *   I set PID I
 *   D set PID D
 *   F set PID FF
 *   U set PID I Unwinding
 *   S set Microstep resolution
 *   T set direcTion polarity
 */
void GcodeSuite::M925() {
  uint8_t keys[10] = {0};
  uint16_t values[10];
  byte set_value = 0;
  uint16_t val = 0;

  if (parser.seen('M')) {
    keys[set_value] = 0xA9;
    values[set_value++] = parser.value_bool() ? 1 : 0;
  }

  if (parser.seen('F')) {
    keys[set_value] = 0xA7;
    values[set_value++] = parser.intval('F', 0);
  }

  if (parser.seen('U')) {
    keys[set_value] = 0xA8;
    values[set_value++] = parser.value_bool() ? 1 : 0;
  }

  if (parser.seen('R')) {
    keys[set_value] = 0xA3;
    val = parser.intval('R', 100);
    if (val > 3100) {
        val = 3100;
    }
    values[set_value++] = val;
  }

  if (parser.seen('P')) {
    keys[set_value] = 0xA0;
    values[set_value++] = parser.intval('P', 30);
  }

  if (parser.seen('I')) {
    keys[set_value] = 0xA1;
    values[set_value++] = parser.intval('I', 10);
  }

  if (parser.seen('D')) {
    keys[set_value] = 0xA2;
    values[set_value++] = parser.intval('D', 250);
  }

  if (parser.seen('S')) {
    keys[set_value] = 0xA4;
    val = parser.intval('S', 16);
    switch (val)
    {
    case 2:
    case 4:
    case 8:
    case 16:
    case 32:
        break;
    default:
        // invalid value
        val = 16;
        break;
    }
    values[set_value++] = val;
  }

  if (parser.seen('T')) {
    keys[set_value] = 0xA6;
    values[set_value++] = parser.value_bool() ? 34 : 17;
  }

  if (set_value == 0) {
      // read parameters
      send_axis_commands(0xb0, 0xaaaa);
      return;
  }

  // write parameters
  LOOP_LINEAR_AXES(i) if (parser.seen(axis_codes[i])) {
    switch (i) {
      case X_AXIS:
        #if AXIS_IS_CLOSEDLOOP(X)
          SERIAL_ECHOPGM("ClosedLoop X:\n");
          for (byte index = 0; index < set_value; index++) {
              encoderX.userCommand(keys[index], values[index]);
          }
        #endif
        break;
      case Y_AXIS:
        #if AXIS_IS_CLOSEDLOOP(Y)
          SERIAL_ECHOPGM("ClosedLoop Y:\n");
          for (byte index = 0; index < set_value; index++) {
              encoderY.userCommand(keys[index], values[index]);
          }
        #endif
        break;
    }
  }
}

/**
 * M924: Set CL_S42B encoder pulses per step count
 *
 *   XYZ pulses per count on axis
 *   IJK encoder home position
 */
void GcodeSuite::M924() {
  // XYZ are steps/mm
  LOOP_LINEAR_AXES(i) if (parser.seenval(axis_codes[i])) {
    float pps = parser.value_float();
    switch (i) {
      case X_AXIS:
        #if AXIS_IS_CLOSEDLOOP(X)
          encoderX.encoder_counts_per_step = pps;
        #endif
        break;
      case Y_AXIS:
        #if AXIS_IS_CLOSEDLOOP(Y)
          encoderY.encoder_counts_per_step = pps;
        #endif
        break;
    }
  }

  // encoder homing positions
  LOOP_LINEAR_AXES(i) if (parser.seen('I' + i)) {
    // can be zero
    int offset = parser.longval('I' + i, 0);
    switch (i) {
      case X_AXIS:
        #if AXIS_IS_CLOSEDLOOP(X)
          encoderX.home_pulse = offset;
        #endif
        break;
      case Y_AXIS:
        #if AXIS_IS_CLOSEDLOOP(Y)
          encoderY.home_pulse = offset;
        #endif
        break;
    }
  }

  SERIAL_ECHOLNPAIR_P(
    PSTR(" M924"), ""
    #if AXIS_IS_CLOSEDLOOP(X)
      , SP_X_STR,
      encoderX.encoder_counts_per_step
    #endif
    #if AXIS_IS_CLOSEDLOOP(Y)
      , SP_Y_STR,
      encoderY.encoder_counts_per_step
    #endif
    #if AXIS_IS_CLOSEDLOOP(X)
      , " I",
      encoderX.home_pulse
    #endif
    #if AXIS_IS_CLOSEDLOOP(Y)
      , " J",
      encoderY.home_pulse
    #endif
  );

}

#endif // HAS_STEALTHCHOP
