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

#include "../feature/kalman.h"

void M783_report(const bool forReplay);

struct THCControlStruct {
  bool is_turn_on;
  bool is_turn_off;
};

typedef struct _THCSettings {
  // kalman filter
  // "sensor nosie covariance";
  float sigma_R;
  // "model nosie covariance";
  float sigma_Q;

  float variance;
  float setpoint_actual;
  float z_steps_per_mm;

  int32_t delay_on;
  float __pv_limit;

  float pid_p;
  float setpoint_fixed;
  float slope_limit;
} THCSettings;

class TorchHeightControl : EKFModel<2, 1, THCControlStruct> {
public:
  static bool enabled;
  static uint32_t acc;
  static uint16_t raw;
  static float filtered;
  static float filtered_dt;
  static float correction;
  static float correction_remainder;

  static float variance;
  static float last_target_v;

  static THCSettings settings;

  // control input
  static uint32_t turned_on_time;
#if ENABLED(TORCH_HEIGHT_CONTROL_TRAPEZOID)
  static uint8_t vel_gain;
  static float nominal_speed_sqr;
  static float nominal_speed;
#endif

  // kalman state
  static bool beam_on;
  static float voltage;
  static float voltage_rate;

  static float last_measurement;

  static float target_v;
  static float accum_i;
  static float accept_factor;

  static ExtendedKalman<2, 1, THCControlStruct> kalman;

  TorchHeightControl();
  static void init();

  static void reset_settings();

  static inline void enable(const bool ena) { enabled = ena; }

  inline void reset() { acc = 0; }
  // called by ACCUMULATE_ADC
  inline void sample(const uint16_t s) { acc += s; }
  void update();

  static void update_height();

  static void update_measured_units();

  static void update_beam(bool on);

  #if ENABLED(TORCH_HEIGHT_CONTROL_TRAPEZOID)
    static void update_vel_gain(uint8_t v, float nominal_speed_sqr);
  #endif

  // EKFModel
  Matrix<1, 1> getR();
  Matrix<2, 2> getQ(const float dT);
  Matrix<2, 1> f(const Matrix<2, 1> x_prev, const float dT, THCControlStruct* u);
  Matrix<2, 2> getF(const Matrix<2, 1> x_prev, const float dT, THCControlStruct* u);
  Matrix<1, 1> h(const Matrix<2, 1> x);
  Matrix<1, 2> getH(const Matrix<2, 1> x);

  bool updateSensorNoise(const float measurement, const float dT, const bool is_turn_on);

  Estimate<2, 1, THCControlStruct> step(const Matrix<1, 1> z, const float deltaT, THCControlStruct* u, const bool skipUpdate);
};

extern TorchHeightControl thc;
#if ENABLED(TORCH_HEIGHT_TH1)
extern struct TempInfo thc_th1;
#endif
