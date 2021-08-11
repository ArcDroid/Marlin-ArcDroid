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

#if ENABLED(MORGAN_SCARA)

#include "../gcode.h"
#include "../../module/scara.h"
#include "../../module/motion.h"
#include "../../module/planner.h"
#include "../../MarlinCore.h" // for IsRunning()

bool SCARA_move_to_cal(const uint8_t delta_a, const uint8_t delta_b) {
  if (IsRunning()) {

    abce_pos_t target = planner.get_axis_positions_mm();
    target.a = delta_a;
    target.b = delta_b;
    planner.buffer_segment(target.a, target.b, target.c, target.e, homing_feedrate(X_AXIS), active_extruder);
    planner.synchronize();
    return true;
  }
  return false;
}

/**
 * M360: SCARA calibration: Move to cal-position ThetaA (0 deg calibration)
 */
bool GcodeSuite::M360() {

  bool setAny = false;
  abce_pos_t target = planner.get_axis_positions_mm();
  const bool hasA = parser.seenval('A'), hasP = parser.seenval('P'), hasX = parser.seenval('X');
  const uint8_t sumAPX = hasA + hasP + hasX;
  if (sumAPX) {
    if (sumAPX == 1) {
      target.a = parser.value_float();
      setAny = true;
    }
    else {
      SERIAL_ERROR_MSG("Only one of A, P, or X is allowed.");
      return false;
    }
  }

  const bool hasB = parser.seenval('B'), hasT = parser.seenval('T'), hasY = parser.seenval('Y');
  const uint8_t sumBTY = hasB + hasT + hasY;
  if (sumBTY) {
    if (sumBTY == 1) {
      target.b = parser.value_float();
      setAny = true;
    }
    else {
      SERIAL_ERROR_MSG("Only one of B, T, or Y is allowed.");
      return false;
    }
  }

  if (setAny) {
    return SCARA_move_to_cal(target.a, target.b);
  }
  else {
    SERIAL_ECHOLNPGM(" Cal: Theta 0");
    return SCARA_move_to_cal(0, 120);
  }
}

/**
 * M361: SCARA calibration: Move to cal-position ThetaB (90 deg calibration - steps per degree)
 */
bool GcodeSuite::M361() {
  SERIAL_ECHOLNPGM(" Cal: Theta 90");
  return SCARA_move_to_cal(90, 130);
}

/**
 * M362: SCARA calibration: Move to cal-position PsiA (0 deg calibration)
 */
bool GcodeSuite::M362() {
  SERIAL_ECHOLNPGM(" Cal: Psi 0");
  return SCARA_move_to_cal(60, 180);
}

/**
 * M363: SCARA calibration: Move to cal-position PsiB (90 deg calibration - steps per degree)
 */
bool GcodeSuite::M363() {
  SERIAL_ECHOLNPGM(" Cal: Psi 90");
  return SCARA_move_to_cal(50, 90);
}

/**
 * M364: SCARA calibration: Move to cal-position PsiC (90 deg to Theta calibration position)
 */
bool GcodeSuite::M364() {
  SERIAL_ECHOLNPGM(" Cal: Theta-Psi 90");
  return SCARA_move_to_cal(45, 135);
}

/**
 * M365: SCARA calibration: set arm lengths (X=L1, Y=L2)
 */
void GcodeSuite::M365() {

  const bool hasA = parser.seenval('A'), hasP = parser.seenval('P'), hasX = parser.seenval('X');
  const uint8_t sumAPX = hasA + hasP + hasX;
  float l1 = scara_L1_base, l2 = scara_L2_base;
  bool setAny = false;
  if (sumAPX) {
    if (sumAPX == 1) {
      l1 = parser.value_float();
      setAny = true;
    }
    else {
      SERIAL_ERROR_MSG("Only one of A, P, or X is allowed.");
      return;
    }
  }

  const bool hasB = parser.seenval('B'), hasT = parser.seenval('T'), hasY = parser.seenval('Y');
  const uint8_t sumBTY = hasB + hasT + hasY;
  if (sumBTY) {
    if (sumBTY == 1) {
      l2 = parser.value_float();
      setAny = true;
    }
    else {
      SERIAL_ERROR_MSG("Only one of B, T, or Y is allowed.");
      return;
    }
  }

  if (setAny) {
    scara_set_arm_length(l1, l2);
  }

  SERIAL_ECHOLNPAIR("SCARA L1:", scara_L1_base, " L2:", scara_L2_base);
}

#endif // MORGAN_SCARA
