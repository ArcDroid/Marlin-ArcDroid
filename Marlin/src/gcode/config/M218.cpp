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

#include "../../inc/MarlinConfig.h"

#if HAS_HOTEND_OFFSET

#include "../gcode.h"
#include "../../module/motion.h"

#if ENABLED(DELTA)
  #include "../../module/planner.h"
#endif

/**
 * M218 - set hotend offset (in linear units)
 *
 *   T<tool>
 *   X<xoffset>
 *   Y<yoffset>
 *   Z<zoffset>
 */
void GcodeSuite::M218() {

  const int8_t target_extruder = get_target_extruder_from_command();
  if (target_extruder < 0) return;

  // tool 0 always has zero offset
  if (target_extruder >= 0) {
    if (parser.seenval('X')) hotend_offset[target_extruder].x = parser.value_linear_units();
    if (parser.seenval('Y')) hotend_offset[target_extruder].y = parser.value_linear_units();
    if (parser.seenval('Z')) hotend_offset[target_extruder].z = parser.value_linear_units();
  }

  if (!parser.seen("XYZ") || target_extruder == 0) {
    SERIAL_ECHO_START();
    SERIAL_ECHOPGM(STR_HOTEND_OFFSET);
    HOTEND_LOOP() {
      SERIAL_CHAR(' ');
      SERIAL_ECHO(hotend_offset[e].x);
      SERIAL_CHAR(',');
      SERIAL_ECHO(hotend_offset[e].y);
      SERIAL_CHAR(',');
      SERIAL_ECHO_F(hotend_offset[e].z, 3);
    }
    SERIAL_EOL();
  }

  #if ENABLED(DELTA)
    if (target_extruder == active_extruder)
      do_blocking_move_to_xy(current_position, planner.settings.max_feedrate_mm_s[X_AXIS]);
  #elif defined(KINEMATIC_TOOL_OFFSET)
    if (target_extruder == active_extruder)
      kinematics_apply_tool_offset(active_extruder, active_extruder);
  #endif
}

#endif // HAS_HOTEND_OFFSET
