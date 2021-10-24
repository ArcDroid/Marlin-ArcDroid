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

#if IS_KINEMATIC

#include "../gcode.h"
#include "../../module/motion.h"
#include "../../module/planner.h"
#include "../../module/stepper/indirection.h"

#if ENABLED(DELTA)

  #include "../../module/delta.h"
  /**
   * M665: Set delta configurations
   *
   *    H = delta height
   *    L = diagonal rod
   *    R = delta radius
   *    S = segments per second
   *    X = Alpha (Tower 1) angle trim
   *    Y = Beta  (Tower 2) angle trim
   *    Z = Gamma (Tower 3) angle trim
   *    A = Alpha (Tower 1) diagonal rod trim
   *    B = Beta  (Tower 2) diagonal rod trim
   *    C = Gamma (Tower 3) diagonal rod trim
   */
  void GcodeSuite::M665() {
    if (parser.seenval('H')) delta_height              = parser.value_linear_units();
    if (parser.seenval('L')) delta_diagonal_rod        = parser.value_linear_units();
    if (parser.seenval('R')) delta_radius              = parser.value_linear_units();
    if (parser.seenval('S')) segments_per_second       = parser.value_float();
    if (parser.seenval('X')) delta_tower_angle_trim.a  = parser.value_float();
    if (parser.seenval('Y')) delta_tower_angle_trim.b  = parser.value_float();
    if (parser.seenval('Z')) delta_tower_angle_trim.c  = parser.value_float();
    if (parser.seenval('A')) delta_diagonal_rod_trim.a = parser.value_float();
    if (parser.seenval('B')) delta_diagonal_rod_trim.b = parser.value_float();
    if (parser.seenval('C')) delta_diagonal_rod_trim.c = parser.value_float();
    recalc_delta_settings();
  }

#elif IS_SCARA

  #include "../../module/scara.h"

  /**
   * M665: Set SCARA settings
   *
   * Parameters:
   *
   *   S[segments-per-second] - Segments-per-second
   *   P[theta-psi-offset]    - Theta-Psi offset, added to the shoulder (A/X) angle
   *   T[theta-offset]        - Theta     offset, added to the elbow    (B/Y) angle
   *   Z[z-offset]            - Z offset, added to Z
   *
   *   A, P, and X are all aliases for the shoulder angle
   *   B, T, and Y are all aliases for the elbow angle
   */
  void GcodeSuite::M665() {
    if (parser.seenval('S')) segments_per_second = parser.value_float();

    abc_pos_t offset_new;

    #if HAS_SCARA_OFFSET

      if (parser.seenval('Z')) offset_new.z = parser.value_linear_units();

      const bool hasA = parser.seenval('A'), hasP = parser.seenval('P'), hasX = parser.seenval('X');
      const uint8_t sumAPX = hasA + hasP + hasX;
      if (sumAPX) {
        if (sumAPX == 1)
          offset_new.a = parser.value_float();
        else {
          SERIAL_ERROR_MSG("Only one of A, P, or X is allowed.");
          return;
        }
      }

      const bool hasB = parser.seenval('B'), hasT = parser.seenval('T'), hasY = parser.seenval('Y');
      const uint8_t sumBTY = hasB + hasT + hasY;
      if (sumBTY) {
        if (sumBTY == 1)
          offset_new.b = parser.value_float();
        else {
          SERIAL_ERROR_MSG("Only one of B, T, or Y is allowed.");
          return;
        }
      }

    #endif // HAS_SCARA_OFFSET

    // reset cartesian position to match offset motor position
    #ifdef MOVE_ON_M665
        xyze_pos_t pos = current_position;
        abce_pos_t target = planner.get_axis_positions_mm();
        target = target - scara_home_offset + offset_new;
    #endif
    #ifdef HAS_CLOSEDLOOP_CONFIG
        abc_pos_t change = offset_new - scara_home_offset;
        #if AXIS_IS_CLOSEDLOOP(X)
            encoderX.offset_encoder_home(change.a);
        #endif
        #if AXIS_IS_CLOSEDLOOP(Y)
            encoderY.offset_encoder_home(change.b);
        #endif
    #endif
        scara_home_offset = offset_new;
    #ifdef MOVE_ON_M665
        planner.set_machine_position_mm(target);
        set_current_from_steppers_for_axis(ALL_AXES_ENUM);
        do_blocking_move_to(pos, homing_feedrate(X_AXIS));
        report_current_position();
    #endif

  }

#endif

#endif // IS_KINEMATIC
