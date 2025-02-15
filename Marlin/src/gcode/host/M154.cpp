/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2021 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#include "../../inc/MarlinConfigPre.h"

#if ENABLED(AUTO_REPORT_POSITION)

#include "../gcode.h"
#include "../../module/motion.h"

/**
 * M154: Set position auto-report interval. M154 S<seconds>
 */
void GcodeSuite::M154() {

  if (parser.seenval('S'))
    position_auto_reporter.set_interval(parser.value_byte());

  if (parser.seenval('P'))
    position_auto_reporter.set_intervalms(parser.value_ushort());
}

#endif // AUTO_REPORT_POSITION
