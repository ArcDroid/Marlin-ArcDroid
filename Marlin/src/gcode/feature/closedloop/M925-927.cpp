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

#include "../../../inc/MarlinConfig.h"

#if HAS_CLOSEDLOOP_CONFIG

#include "../../gcode.h"
#include "../../../feature/tmc_util.h"
#include "../../../module/stepper/indirection.h"

#include "../../../module/endstops.h"

void send_axis_commands(uint8_t function, uint16_t data) {
  LOOP_LINEAR_AXES(i) if (parser.seen(axis_codes[i])) {
    switch (i) {
      case X_AXIS:
        #if AXIS_IS_CLOSEDLOOP(X)
          SERIAL_ECHOPGM("ClosedLoop X:\n");
          encoderX.userCommand(function, data);
        #endif
        break;
      case Y_AXIS:
        #if AXIS_IS_CLOSEDLOOP(Y)
          SERIAL_ECHOPGM("ClosedLoop Y:\n");
          encoderY.userCommand(function, data);
        #endif
        break;
    }
  }
}

/**
 * M927: Send CL_S42B raw command
 *
 *   XYZE to target an axis
 *   P command
 *   Q value
 */
void GcodeSuite::M927() {
  uint8_t function = 0;
  uint16_t data = 0;

  if (parser.seen('P')) {
    function = parser.intval('P', 0xB0);
  }

  if (parser.seen('Q')) {
    data = parser.intval('Q');
  }

  if (function == 0)
    return;

  send_axis_commands(function, data);
}

/**
 * M926: Start CL_S42B calibration
 *
 *   XYZE to target an axis
 */
void GcodeSuite::M926() {
  send_axis_commands(0xaf, 0x11);
}

/**
 * M925: Configure CL_S42B driver
 *
 *   XYZE to target an axis
 *   M[1|0] set mode to Closed/Open loop
 *   R set cuRrent
 *   P set PID P
 *   I set PID I
 *   D set PID D
 *   F set PID FF
 *   U set PID I Unwinding
 *   S set Microstep resolution
 *   T set direcTion polarity
 */
void GcodeSuite::M925() {
  uint8_t keys[10] = {0};
  uint16_t values[10];
  byte set_value = 0;
  uint16_t val = 0;

  if (parser.seen('M')) {
    keys[set_value] = 0xA9;
    values[set_value++] = parser.value_bool() ? 1 : 0;
  }

  if (parser.seen('F')) {
    keys[set_value] = 0xA7;
    values[set_value++] = parser.intval('F', 0);
  }

  if (parser.seen('U')) {
    keys[set_value] = 0xA8;
    values[set_value++] = parser.value_bool() ? 1 : 0;
  }

  if (parser.seen('R')) {
    keys[set_value] = 0xA3;
    val = parser.intval('R', 100);
    if (val > 3100) {
        val = 3100;
    }
    values[set_value++] = val;
  }

  if (parser.seen('P')) {
    keys[set_value] = 0xA0;
    values[set_value++] = parser.intval('P', 30);
  }

  if (parser.seen('I')) {
    keys[set_value] = 0xA1;
    values[set_value++] = parser.intval('I', 10);
  }

  if (parser.seen('D')) {
    keys[set_value] = 0xA2;
    values[set_value++] = parser.intval('D', 250);
  }

  if (parser.seen('S')) {
    keys[set_value] = 0xA4;
    val = parser.intval('S', 16);
    switch (val)
    {
    case 2:
    case 4:
    case 8:
    case 16:
    case 32:
        break;
    default:
        // invalid value
        val = 16;
        break;
    }
    values[set_value++] = val;
  }

  if (parser.seen('T')) {
    keys[set_value] = 0xA6;
    values[set_value++] = parser.value_bool() ? 34 : 17;
  }

  if (set_value == 0) {
      // read parameters
      send_axis_commands(0xb0, 0xaaaa);
      return;
  }

  // write parameters
  LOOP_LINEAR_AXES(i) if (parser.seen(axis_codes[i])) {
    switch (i) {
      case X_AXIS:
        #if AXIS_IS_CLOSEDLOOP(X)
          SERIAL_ECHOPGM("ClosedLoop X:\n");
          for (byte index = 0; index < set_value; index++) {
              encoderX.userCommand(keys[index], values[index]);
          }
        #endif
        break;
      case Y_AXIS:
        #if AXIS_IS_CLOSEDLOOP(Y)
          SERIAL_ECHOPGM("ClosedLoop Y:\n");
          for (byte index = 0; index < set_value; index++) {
              encoderY.userCommand(keys[index], values[index]);
          }
        #endif
        break;
    }
  }
}

/**
 * M924: Set CL_S42B encoder pulses per step count
 *
 *   XYZ pulses per count on axis
 *   IJK encoder home position
 */
void GcodeSuite::M924() {
  // XYZ are steps/mm
  LOOP_LINEAR_AXES(i) if (parser.seenval(axis_codes[i])) {
    float pps = parser.value_float();
    switch (i) {
      case X_AXIS:
        #if AXIS_IS_CLOSEDLOOP(X)
          encoderX.encoder_counts_per_step = pps;
        #endif
        break;
      case Y_AXIS:
        #if AXIS_IS_CLOSEDLOOP(Y)
          encoderY.encoder_counts_per_step = pps;
        #endif
        break;
    }
  }

  // encoder homing positions
  LOOP_LINEAR_AXES(i) if (parser.seen('I' + i)) {
    // can be zero
    int offset = parser.longval('I' + i, 0);
    switch (i) {
      case X_AXIS:
        #if AXIS_IS_CLOSEDLOOP(X)
          encoderX.home_pulse = offset;
        #endif
        break;
      case Y_AXIS:
        #if AXIS_IS_CLOSEDLOOP(Y)
          encoderY.home_pulse = offset;
        #endif
        break;
    }
  }

  SERIAL_ECHOLNPAIR_P(
    PSTR(" M924"), ""
    #if AXIS_IS_CLOSEDLOOP(X)
      , SP_X_STR,
      encoderX.encoder_counts_per_step
    #endif
    #if AXIS_IS_CLOSEDLOOP(Y)
      , SP_Y_STR,
      encoderY.encoder_counts_per_step
    #endif
    #if AXIS_IS_CLOSEDLOOP(X)
      , " I",
      encoderX.home_pulse
    #endif
    #if AXIS_IS_CLOSEDLOOP(Y)
      , " J",
      encoderY.home_pulse
    #endif
  );

}



bool SCARA_move_relative_start(const float rel_a, const float rel_b, const float rel_z) {
  abce_pos_t target = planner.get_axis_positions_mm();
  target.a += rel_a;
  target.b += rel_b;
  target.z += rel_z;
  return planner.buffer_segment(target.a, target.b, target.c, target.e, homing_feedrate(X_AXIS), active_extruder);
}

struct PollResult {
  float min_value;
  float max_value;
  float trigger_pos;
  int samples;
};

bool poll_stepper_result(AxisEnum axis, float threshold, struct PollResult& output) {
  int force = 10;
  while (planner.has_blocks_queued() || force > 0) {
    force--;
    abce_pos_t pos = planner.get_axis_positions_mm();
    float enc_pos;
    switch (axis) {
      case X_AXIS:
        enc_pos = encoderX.read_encoder(true);
        break;
      case Y_AXIS:
        enc_pos = encoderY.read_encoder(true);
        break;
      default:
        enc_pos = NAN;
    }
    float pos_error = enc_pos - pos.pos[axis];

    output.samples++;

    //SERIAL_ECHOLNPAIR("; E", pos_error, " S", pos.pos[axis], " M", enc_pos);

    if (pos_error < output.min_value)
      output.min_value = pos_error;
    if (pos_error > output.max_value)
      output.max_value = pos_error;

    if ((threshold > 0 && pos_error > threshold) ||
        (threshold < 0 && pos_error < threshold)) {

      //SERIAL_ECHOLNPAIR("; zE", pos_error, " zS", pos.pos[axis], " zM", enc_pos, " zT", threshold, " zF", force);
      planner.quick_stop();
      output.trigger_pos = enc_pos;
      return true;
    }

    idle_no_sleep();
  }
  return false;
}

/**
 * M930: check belt tension
 *
 *   XYZE to target an axis
 *   P move distance
 *   T TMC SG_RESULT threshold
 *   R repeat cycles
 *   Q test current
 */
void GcodeSuite::M930() {

  float D = 15;
  float T = 0.1f;
  int R = 3;
  int Q = 800;

  float min_backlash = 0;
  float max_backlash = 0;

  if (parser.seen('D')) {
    D = parser.value_float();
  }

  if (parser.seen('T')) {
    T = parser.value_float();
  }

  if (parser.seen('R')) {
    R = parser.value_int();
  }

  if (parser.seen('Q')) {
    Q = parser.value_int();
  }

  int axis = NO_AXIS_ENUM;

  LOOP_LINEAR_AXES(i) if (parser.seen(axis_codes[i])) {
    axis = i;
    break;
  }

  if (axis < A_AXIS || axis > Z_AXIS)
    return;

  endstops.enable_globally(false);
  char axis_string[2] = { axis_codes[axis], 0};

  // SERIAL_ECHOPAIR("; test start: ");
  // SERIAL_ECHOPAIR("D", D);
  // SERIAL_ECHOPAIR_F("T", T);
  // SERIAL_ECHOPAIR("R", R);
  // SERIAL_ECHOPAIR("Axis: ", axis_string);

  planner.finish_and_disable();
  delay(20);

  int old_current;
  switch (axis) {
    case X_AXIS:
      old_current = stepperX.rms_current();
      stepperX.rms_current(Q);
      break;
    case Y_AXIS:
      old_current = stepperY.rms_current();
      stepperY.rms_current(Q);
      break;
    default:
      old_current = 0;
  }

  struct PollResult res_range[R*2 + 1];


  #if HAS_CLOSEDLOOP_CONFIG
    set_position_from_encoders_if_lost(true);
  #endif
  enable_all_steppers();
  #if HAS_CLOSEDLOOP_CONFIG
    set_position_from_encoders_if_lost(true);
  #endif

  // complete all motion
  planner.synchronize();
  delay(200);

  for (int i = 0; i < R; i++) {

    for (int d = 0; d < 2; d++) {
      res_range[i*2 + d].max_value = -123;
      res_range[i*2 + d].min_value = 123;
      res_range[i*2 + d].trigger_pos = NAN;
      res_range[i*2 + d].samples = 0;


      float delta = d == 0 ? D : -D;
      abce_pos_t start_pos = planner.get_axis_positions_mm();

      while (!SCARA_move_relative_start(
        axis == A_AXIS ? delta : 0,
        axis == B_AXIS ? delta : 0,
        axis == Z_AXIS ? delta : 0
      )){};

      poll_stepper_result((AxisEnum)axis, d == 0 ? -T : T, res_range[i*2 + d]);

      if (__isnanf(res_range[i*2 + d].trigger_pos)) {
        SERIAL_ERROR_MSG("M930 did not reach error threshold, arm not held or belt slipping?");
        goto exit_M930;
      }

      res_range[i*2 + d].trigger_pos -= start_pos.pos[axis];

      planner.synchronize();
      disable_all_steppers();

      delay(200);
    }

      // SERIAL_ECHOPAIR("; pass result: ", i + 1);
      // SERIAL_ECHOPAIR(  "  Fs:", res_range[i*2 + 0].samples);
      // SERIAL_ECHOPAIR_F( " Fm:", res_range[i*2 + 0].min_value);
      // SERIAL_ECHOPAIR_F( " FM:", res_range[i*2 + 0].max_value);
      // SERIAL_ECHOPAIR_F( " FR:", res_range[i*2 + 0].trigger_pos);
      // SERIAL_ECHOPAIR(  "  Bs:", res_range[i*2 + 1].samples);
      // SERIAL_ECHOPAIR_F( " Bm:", res_range[i*2 + 1].min_value);
      // SERIAL_ECHOPAIR_F( " BM:", res_range[i*2 + 1].max_value);
      // SERIAL_ECHOPAIR_F( " BR:", res_range[i*2 + 1].trigger_pos);
      // SERIAL_EOL();
  }

  for (int i = 0; i < R; i++) {
    max_backlash += res_range[i*2 + 0].trigger_pos;
    min_backlash += res_range[i*2 + 1].trigger_pos;
  }
  max_backlash /= R;
  max_backlash /= R;

  SERIAL_ECHO_MSG("M930 done T", T, " D", D, " R", R, " Q", Q, " Axis:", axis_string, " Backlash:", max_backlash - min_backlash);

exit_M930:

  switch (axis) {
    case X_AXIS:
      stepperX.rms_current(old_current);
      break;
    case Y_AXIS:
      stepperY.rms_current(old_current);
      break;
    default:
      ;
  }

  endstops.enable_globally(true);

}

#endif // HAS_STEALTHCHOP
