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
#pragma once

#include "../inc/MarlinConfig.h"

template <typename Helper>
struct AutoReporter {
  millis_t next_report_ms;
  uint8_t report_interval;
  #if HAS_MULTI_SERIAL
    SerialMask report_port_mask;
    AutoReporter() : report_port_mask(SerialMask::All) {}
  #endif

  inline void set_intervalms(uint32_t ms, const uint32_t limit=60000) {
    report_interval = _MIN(ms, limit);
    next_report_ms = millis() + ms;
  }

  inline void set_interval(uint8_t seconds, const uint8_t limit=60) {
    set_intervalms(SEC_TO_MS(seconds), SEC_TO_MS(limit));
  }

  inline void tick() {
    if (!report_interval) return;
    const millis_t ms = millis();
    if (ELAPSED(ms, next_report_ms)) {
      next_report_ms = ms + report_interval;
      PORT_REDIRECT(report_port_mask);
      Helper::report();
      //PORT_RESTORE();
    }
  }
};
