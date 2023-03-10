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


/**
 * M922: apply external offset to machine coordinates
 */
void GcodeSuite::M922() {

  bool sync_XYZE = false;

  LOOP_LINEAR_AXES(i) {
    if (parser.seenval(axis_codes[i])) {
      const float v = parser.value_float();       // external offset in mm(!)
        external_shift[i] = v;
        update_workspace_offset((AxisEnum)i);
    }
  }

  if   (sync_XYZE) sync_plan_position();
}

#endif // CNC_COORDINATE_SYSTEMS
