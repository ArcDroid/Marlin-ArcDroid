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
#include "../module/planner.h"
// #include "../module/thermistor/thermistors.h"

class TorchHeightControl {
public:
  static bool enabled;              // THC enabled
  static uint32_t acc;              // ADC accumulator
  static uint16_t raw;              // Measured filament diameter - one extruder only

  TorchHeightControl() { init(); }
  static void init();

  static inline void enable(const bool ena) { enabled = ena; }

  inline void reset() { acc = 0; }
  inline void sample(const uint16_t s) { acc += s; }
  inline void update() { raw = acc; }

  static void update_height() {
    //TODO
  }

  static void update_measured_units() {
    //TODO
  }

};

extern TorchHeightControl thc;
#if ENABLED(TORCH_HEIGHT_TH1)
extern struct TempInfo thc_th1;
#endif
