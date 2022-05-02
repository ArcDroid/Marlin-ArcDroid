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

#include "../inc/MarlinConfig.h"

#if ENABLED(TORCH_HEIGHT_CONTROL)

#include "thc.h"
#include "../module/temperature.h"
#include "babystep.h"

#if !ENABLED(BABYSTEPPING)
  #error "TORCH_HEIGHT_CONTROL requires a BABYSTEPPING."
#endif


Matrix<1, 1> inverse(Matrix<1, 1>a) {
  Matrix<1, 1> ret;
  ret.x[0][0] = 1.0f / a.x[0][0];
  return ret;
};


TorchHeightControl thc;

#if ENABLED(TORCH_HEIGHT_TH1)
struct TempInfo thc_th1;
#endif

bool TorchHeightControl::enabled; // = false;                          // Torch height control on/off
uint32_t TorchHeightControl::acc; // = 0                             // ADC accumulator
uint16_t TorchHeightControl::raw; // = 0                               // Raw ADC torch voltage measurement
float TorchHeightControl::filtered;
float TorchHeightControl::filtered_dt;
float TorchHeightControl::correction;

THCSettings TorchHeightControl::settings;

uint32_t TorchHeightControl::turned_on_time;

bool TorchHeightControl::beam_on;
float TorchHeightControl::voltage;
float TorchHeightControl::voltage_rate;
float TorchHeightControl::sigma_R;

float TorchHeightControl::last_measurement;
float TorchHeightControl::last_rate;

float TorchHeightControl::target_v;
float TorchHeightControl::accum_i;

ExtendedKalman<2, 1, THCControlStruct> TorchHeightControl::kalman;

TorchHeightControl::TorchHeightControl() {
  init();
}

void TorchHeightControl::init() {
  turned_on_time = 0;

  voltage = 0;
  voltage_rate = 0;
  sigma_R = settings.sigma_R_min * 20;

  last_measurement = NAN;
  last_rate = NAN;

  target_v = NAN;
  accum_i = 0;
}

void TorchHeightControl::reset_settings() {
  // kalman filter
  settings.sigma_R_min = 60.0f;
  settings.sigma_Q = 1E-06f;
  settings.sigma_R_decay_time = 1000.0f;
  settings.sensor_rate_scale = 0.2f;
  settings.sensor_rate_rate_scale = 0.1f/50;

  settings.delay_on = 2000;
  settings.pv_limit = 40.0f;

  settings.pid_p = .3f;
  settings.pid_i = 0;
  settings.pid_d = 0;
}

void TorchHeightControl::update_beam(bool on) {
  beam_on = on;
  if (on) {
    turned_on_time = getCurrentMillis();
  } else {
    correction = 0;
  }
}

void TorchHeightControl::update() {
  raw = acc;

  if (__isnanf(last_measurement)) {

    Estimate<2, 1, THCControlStruct> est = {0};
    est.P.x[0][0] = .1;
    est.P.x[0][1] = 0;
    est.P.x[1][0] = 0;
    est.P.x[1][1] = .1;

    est.x.x[0][0] = raw;

    kalman.initialize(est, this);
  }

  static uint32_t time_last = 0;
  uint32_t time = getCurrentMillis();

  int32_t dt = (int32_t)(time - turned_on_time);

  bool is_turn_on = beam_on && dt > 0 && dt < settings.delay_on;

  // update kalman filter
  Matrix<1,1> m;
  m.x[0][0] = raw;
  float dT = (float) (int32_t) (time - time_last);
  //if (dT < 100) {
    //SERIAL_ECHOLNPAIR(" updateSensorNoise(", raw, ", ", dT, ", ", is_turn_on, ")");
    bool accept = updateSensorNoise(raw, dT, is_turn_on);

    //SERIAL_ECHOLNPAIR(" step(", raw, ", ", dT, ")");
    Estimate<2, 1, THCControlStruct> e = step(m, dT, NULL, !accept);

    filtered = e.x.x[0][0];
    filtered_dt = e.x.x[1][0];
  //}

  if (is_turn_on) {
    // beam turning on
    target_v = filtered;
    accum_i = 0;
  }

  if (beam_on && enabled && dt > settings.delay_on) {
    // babystep Z here
    float error = filtered - target_v;
    float p_part = error * settings.pid_p;
    if (fabs(p_part) < settings.pv_limit && fabs(error) > 0.00001) {
      accum_i += error * dT * settings.pid_i;
    } else {
      accum_i = 0;
    }
    float d_part = filtered_dt * settings.pid_d; // kalman already includes step time in velocity calculation
    correction = -(p_part + accum_i + d_part);
    int16_t correction_i = (int16_t) round(correction);
    int16_t existing = babystep.steps[BS_AXIS_IND(Z_AXIS)];

    if (correction_i - existing) {
      babystep.add_steps(Z_AXIS, correction_i - existing);
    }
  }

  time_last = time;
}

bool TorchHeightControl::updateSensorNoise(const float measurement, const float dT, const bool is_turn_on)
{
  // trust sensor readings more over time
  float weight = MIN(1.0f, dT / settings.sigma_R_decay_time);
  sigma_R = sigma_R * (1.0f - weight) + settings.sigma_R_min * weight;
  if (__isnanf(last_measurement))
  {
    // no last measurement to compare to, accept measuremnt
    last_measurement = measurement;
    return true;
  }

  // accept sensor readings by default
  bool accept = true;
  // check if measurement rate of change is too fast for Math.Max actual rate
  // start increasing measurement error for kalman filter if so
  float rate = (measurement - last_measurement) / dT;

  //var flow_min = draining ? this.config.flow_min_open : this.config.flow_min_closed;
  float diff = 0;
  //if (rate < flow_min)
  //{
  //	diff = rate / flow_min * this.config.sensor_rate_scale;
  //}
  //else if (rate > this.config.flow_max)
  //{
  //	diff = rate / this.config.flow_max * this.config.sensor_rate_scale;
  //}

  // high rate of rate indicates repeating noise spikes
  // adjust measurement error along with it as well
  if (!__isnanf(last_rate))
  {
    float rateRate = powf(rate - last_rate, 2.0f) / dT;
    diff = MAX(diff, rateRate / settings.sensor_rate_rate_scale);
  }

  if (sigma_R < diff && !is_turn_on)
  {
    // adjust measurement error
    sigma_R = diff;
    // tell caller not to pass this reading to filter, use model only
    accept = false;
  }
  else if (is_turn_on)
  {
    sigma_R = settings.sigma_R_min;
  }
  last_rate = rate;
  last_measurement = measurement;
  return accept;
}

Estimate<2, 1, THCControlStruct> TorchHeightControl::step(const Matrix<1, 1> z, const float deltaT, THCControlStruct* u, const bool skipUpdate) {
  //#  predict
    // estimated state
    Matrix<2, 1> xPriori = kalman.model->f(kalman.state.x, deltaT, u);
    // estimated error covariance
    Matrix<2, 2> F = kalman.model->getF(kalman.state.x, deltaT, u);
    //## Ppriori = F.times(self.state.P).times(F.transpose()).plus(self.model->getQ(deltaT))
    Matrix<2, 2> Ppriori = plus(times(times(F, kalman.state.P), transpose(F)), kalman.model->getQ(deltaT));

    // xPriori.debugPrint("xPriori");
    // F.debugPrint("F");
    // Ppriori.debugPrint("Ppriori");
    kalman.state.x = xPriori;
    kalman.state.P = Ppriori;
    kalman.state.dt = deltaT;
    kalman.state.u = u;
    if (!skipUpdate)
    {
      //# update
      // difference between expected observation and actual
      Matrix<1, 1> y = minus(z, kalman.model->h(xPriori));
      // innovation covariance
      Matrix<1, 2> H = kalman.model->getH(xPriori);
      Matrix<1, 1> S = plus(times(times(H, Ppriori), transpose(H)), kalman.model->getR());
      // kalman gain
      Matrix<2, 1> K = times(times(Ppriori, transpose(H)), inverse(S));
      // state estimate updated with measurement
      Matrix<2, 1> x = plus(xPriori, times(K, y));
      // estimated covariance
      Matrix<2, 2> P = times(minus(identity<2>(), times(K, H)), Ppriori);
      // y.debugPrint("y");
      // H.debugPrint("H");
      // S.debugPrint("S");
      // K.debugPrint("K");
      // x.debugPrint("x");
      // P.debugPrint("P");
      kalman.state.x = x;
      kalman.state.P = P;
    }
    ////kalman.state = new Estimate(xPriori, Ppriori, kalman.state.u, deltaT);
    return kalman.state;
}

void TorchHeightControl::update_height() {

}

void TorchHeightControl::update_measured_units() {

}


Matrix<1, 1> TorchHeightControl::getR() {
  Matrix<1,1> ret;
  ret.x[0][0] = sigma_R;
  return ret;
}
Matrix<2, 2> TorchHeightControl::getQ(float dT) {
  Matrix<2,2> ret;
  ret.x[0][0] = dT*dT / 2.0f * settings.sigma_Q;
  ret.x[0][1] = 0.0;
  ret.x[1][0] = 0;
  ret.x[1][1] = dT * settings.sigma_Q;
  return ret;
}
Matrix<2, 1> TorchHeightControl::f(const Matrix<2, 1> x_prev, const float dT, THCControlStruct* u) {
  Matrix<2,1> ret;
  // xNext.voltage = x.voltage + dT * x.voltageRate;
  ret.x[0][0] = x_prev.x[0][0] + dT * x_prev.x[1][0];
  // voltageRate should slowly decrease to 0 over time
  ret.x[1][0] = x_prev.x[1][0] * MAX(0.0f, MIN(1.0f, 1.0f - dT / 300.0f));
  return ret;
}
Matrix<2, 2> TorchHeightControl::getF(const Matrix<2, 1> x_prev, const float dT, THCControlStruct* u) {
  Matrix<2, 2> ret;
  ret.x[0][0] = 1;
  ret.x[0][1] = dT;
  ret.x[1][0] = 0;
  ret.x[1][1] = 1;
  return ret;
}
Matrix<1, 1> TorchHeightControl::h(const Matrix<2, 1> x) {
  Matrix<1, 1> ret;
  ret.x[0][0] = x.x[0][0];
  return ret;
}
Matrix<1, 2> TorchHeightControl::getH(const Matrix<2, 1> x) {
  Matrix<1, 2> ret;
  ret.x[0][0] = 1;
  ret.x[0][1] = 0;
  return ret;
}

#endif // TORCH_HEIGHT_CONTROL
