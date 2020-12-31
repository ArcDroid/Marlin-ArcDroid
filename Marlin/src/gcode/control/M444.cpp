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

#include "../../inc/MarlinConfig.h"

#if HAS_CUTTER

#include "../gcode.h"
#include "../../feature/spindle_laser.h"
/**
 * M444 - Set cutter start(S) and stop(P) delay in seconds (Requies HAS_CUTTER)
 */
void GcodeSuite::M444() {
  millis_t dwell_ms = 0;

  if (parser.seenval('S')) {
    dwell_ms = parser.value_millis_from_seconds();
    cutter.powerup_delay = dwell_ms;
  }
  if (parser.seenval('P')) {
    dwell_ms = parser.value_millis_from_seconds();
    cutter.powerdown_delay = dwell_ms;
  }

  SERIAL_ECHOLNPAIR_P(PSTR("Cutter delay start:"), cutter.powerup_delay / 1000.0f,
    PSTR(" stop:"), cutter.powerdown_delay / 1000.0f);

}

#endif // HAS_CUTTER
