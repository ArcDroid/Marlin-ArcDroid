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

#include "../gcode.h"
#include "../../module/motion.h"

#if ENABLED(ENCODER_SLED)

#include "../../module/stepper.h"
#include "../../module/temperature.h"


/**
 * M922: apply external offset to machine coordinates
 */
void GcodeSuite::M922() {
  // M922.1 zero external offset
  if (parser.subcode == 1) {
    external_shift_zero = external_shift;
    LOOP_LINEAR_AXES(i) {
      update_workspace_offset((AxisEnum)i);
    }
    return;
  }

  // only update when motors are off
  if(parser.subcode == 0) {

    LOOP_LINEAR_AXES(i) {
      if (parser.seenval(axis_codes[i])) {
        const float v = parser.value_float();       // external offset in mm(!)
        external_shift_next[i] = v;
        external_shift_set |= (1<<i);
      }
    }
    if (external_shift_set) {
      // reduce traffic on the scale port by disabling auto-reporting on it
      auto mask = position_auto_reporter.report_port_mask;
      mask = position_auto_reporter.report_port_mask.combineMinus(multiSerial.portMask);
      position_auto_reporter.report_port_mask = mask;
      thermalManager.auto_reporter.report_port_mask = mask;
    }
  }
}

#endif // CNC_COORDINATE_SYSTEMS
