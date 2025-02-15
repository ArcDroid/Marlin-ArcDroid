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

#if HAS_BED_PROBE

#include "../gcode.h"
#include "../../module/motion.h"
#include "../../module/probe.h"
#include "../../feature/bedlevel/bedlevel.h"

#if ENABLED(ARBITRARY_LEVEL_POINTS)
  #include "../../module/planner.h"
#endif

/**
 * G30: Do a single Z probe at the current XY
 *
 * Parameters:
 *
 *   X   Probe X position (default current X)
 *   Y   Probe Y position (default current Y)
 *   E   Engage the probe for each probe (default 1)
 */
void GcodeSuite::G30() {
  #if HAS_CLOSEDLOOP_CONFIG
    set_position_from_encoders_if_lost(true);
  #endif

  const xy_pos_t pos = { parser.linearval('X', current_position.x + probe.offset_xy.x),
                         parser.linearval('Y', current_position.y + probe.offset_xy.y) };

  if (!probe.can_reach(pos)) return;

  // Disable leveling so the planner won't mess with us
  #if HAS_LEVELING
    bool old_leveling = get_bed_leveling_enabled();
    set_bed_leveling_enabled(false);
  #endif

  remember_feedrate_scaling_off();

  const ProbePtRaise raise_after = parser.boolval('E', true) ? PROBE_PT_STOW : PROBE_PT_NONE;
  const float measured_z = probe.probe_at_point(pos, raise_after, 1, true, false);
  if (!isnan(measured_z))
    SERIAL_ECHOLNPAIR("Bed X: ", pos.x, " Y: ", pos.y, " Z: ", measured_z);

  restore_feedrate_and_scaling();

  if (raise_after == PROBE_PT_STOW)
    probe.move_z_after_probing();

  report_current_position();

  #if HAS_LEVELING
    if (old_leveling) {
      #if ENABLED(ARBITRARY_LEVEL_POINTS)
        planner.set_level_fulcrum(current_position);
      #endif
      set_bed_leveling_enabled(old_leveling);
    }
  #endif
}

#endif // HAS_BED_PROBE
