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
#include "../../module/stepper.h"

#if ENABLED(I2C_POSITION_ENCODERS)
  #include "../../feature/encoder_i2c.h"
#endif

/**
 * G92: Set current position to given X Y Z E
 */
void GcodeSuite::G92() {

  bool sync_E = false, sync_XYZ = false;

  #if ENABLED(USE_GCODE_SUBCODES)
    const uint8_t subcode_G92 = parser.subcode;
  #else
    constexpr uint8_t subcode_G92 = 0;
  #endif

  switch (subcode_G92) {
    default: break;
    #if ENABLED(CNC_COORDINATE_SYSTEMS)
      case 1: {
        // Zero the G92 values and restore current position
        LOOP_XYZ(i) if (position_shift[i]) {
          position_shift[i] = 0;
          offset_rotation = OFFSET_ROTATION_0;
          update_workspace_offset((AxisEnum)i);
        }
      } break;
    #endif
    #if ENABLED(POWER_LOSS_RECOVERY)
      case 9: {
        LOOP_XYZE(i) {
          if (parser.seenval(axis_codes[i])) {
            current_position[i] = parser.value_axis_units((AxisEnum)i);
            if (i == E_AXIS) sync_E = true; else sync_XYZ = true;
          }
        }
      } break;
    #endif
    case 0: {
      #if HAS_CLOSEDLOOP_CONFIG
        set_position_from_encoders_if_lost(false);
      #endif
      LOOP_XYZE(i) {
        if (parser.seenval(axis_codes[i])) {
          const float l = parser.value_axis_units((AxisEnum)i),
                      v = i == E_AXIS ? l : LOGICAL_TO_NATIVE(l, i),
                      d = v - current_position[i];
          if (!NEAR_ZERO(d)) {
            //// FIXME
            #if !HAS_POSITION_SHIFT
              if (i == E_AXIS) sync_E = true; else sync_XYZ = true;
              current_position[i] = v;        // Without workspaces revert to Marlin 1.0 behavior
            #else
              if (i == E_AXIS) {
                sync_E = true;
                current_position.e = v;       // When using coordinate spaces, only E is set directly
              }
              else {
                position_shift[i] += d;       // Other axes simply offset the coordinate space
                update_workspace_offset((AxisEnum)i);
              }
            #endif
          }
        }
      }
        #if HAS_POSITION_SHIFT
          if (parser.seenval('R')) {
            bool rotate = parser.value_bool();
            offset_rotation = rotate ? OFFSET_ROTATION_180 : OFFSET_ROTATION_0;
          }
        #endif
    } break;
  }

  #if ENABLED(CNC_COORDINATE_SYSTEMS)
    // Apply workspace offset to the active coordinate system
    if (WITHIN(active_coordinate_system, 0, MAX_COORDINATE_SYSTEMS - 1)) {
      coordinate_system[active_coordinate_system].offset = position_shift;
      coordinate_system[active_coordinate_system].rotation = offset_rotation;
    }
  #endif

  if    (sync_XYZ) sync_plan_position();
  else if (sync_E) sync_plan_position_e();

  #if DISABLED(DIRECT_STEPPING)
    report_current_position();
  #endif
}
