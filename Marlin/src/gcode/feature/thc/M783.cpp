
#include "../../../inc/MarlinConfigPre.h"

#if ENABLED(TORCH_HEIGHT_CONTROL)

#include "../../gcode.h"
#include "../../../feature/thc.h"


void M783_report(const bool forReplay) {
  #ifdef DISABLE_THC
    return;
  #endif

  if (!forReplay) { SERIAL_ECHO_MSG("; Torch Height Control"); SERIAL_ECHO_START(); }
  SERIAL_ECHOLNPAIR("  M783"
    " E", int(thc.enabled),
    " Q", (__isnanf(thc.settings.setpoint_fixed) ? 0.0f : thc.settings.setpoint_fixed),
    " L", thc.settings.delay_on / 1000.0f,
    " R", thc.settings.sigma_R,
    " P", thc.settings.pid_p,
    " V", thc.settings.slope_limit,
    " T", thc.variance,
    " A", thc.last_target_v,
  );
}

/**
 * M783: Set Torch Height Control settings
 *  E         : Enable (1/0)
 *  Q         : Target value (THC ADC value/4096.0)
 *
 *  L         : Delay after firing before activating control (decimal seconds)
 *  M         : Kalman sigma_R_min
 *  S         : sensor_rate_scale
 *  T         : sensor_rate_rate_scale
 *  B         : pv_limit
 *
 *  P         : PID P gain
 *  V         : Velocity Comp
 */
void GcodeSuite::M783() {
  #ifdef DISABLE_THC
    return;
  #endif

  const bool seenE = parser.seenval('E');
  if (seenE) thc.enabled = parser.value_bool();

  const bool seenQ = parser.seenval('Q');
  if (seenQ) {
    float q = parser.value_float();
    if (q == 0.0f) {
      thc.settings.setpoint_fixed = NAN;
    }
    else {
      thc.settings.setpoint_fixed = q;
    }
  }

  const bool seenL = parser.seenval('L');
  if (seenL) thc.settings.delay_on = (int32_t) parser.value_millis_from_seconds();

  const bool seenM = parser.seenval('R');
  if (seenM) thc.settings.sigma_R = parser.value_float();

  const bool seenP = parser.seenval('P');
  if (seenP) thc.settings.pid_p = parser.value_float();

  const bool seenV = parser.seenval('V');
  if (seenV) thc.settings.slope_limit = parser.value_float();

  //if (!(seenE || seenL || seenM || seenS || seenT || seenB || seenP))
    M783_report(false);
}


#endif
