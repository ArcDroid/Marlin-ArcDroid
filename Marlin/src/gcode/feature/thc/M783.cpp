
#include "../../../inc/MarlinConfigPre.h"

#if ENABLED(TORCH_HEIGHT_CONTROL)

#include "../../gcode.h"
#include "../../../feature/thc.h"


void M783_report(bool forReplay) {
  if (!forReplay) { SERIAL_ECHOLNPGM("; Torch Height Control"); SERIAL_ECHO_START(); }
  SERIAL_ECHOLNPAIR("  M783"
    " E", int(thc.enabled),
    " D", thc.delay_on,
    " M", thc.sigma_R_min,
    " S", thc.sensor_rate_scale,
    " T", thc.sensor_rate_rate_scale,
    " B", thc.rate_toggle,
    " G", thc.pid_p
  );
}

/**
 * M783: Set Torch Height Control settings
 *  E         : Enable (1/0)
// *  P         : Target value (THC ADC value/4096.0)
 *
 *  D         : Delay after firing before activating control (decimal seconds)
 *  M         : Kalman sigma_R_min
 *  S         : sensor_rate_scale
 *  T         : sensor_rate_rate_scale
 *  B         : rate_toggle
 *
 *  G         : PID P gain
 */
void GcodeSuite::M783() {

  const bool seenE = parser.seenval('E');
  if (seenE) thc.enabled = parser.value_bool();

  const bool seenD = parser.seenval('D');
  if (seenD) thc.delay_on = (int32_t) parser.value_millis_from_seconds();

  const bool seenM = parser.seenval('M');
  if (seenM) thc.sigma_R_min = parser.value_float();

  const bool seenS = parser.seenval('S');
  if (seenS) thc.sensor_rate_scale = parser.value_float();

  const bool seenT = parser.seenval('T');
  if (seenT) thc.sensor_rate_rate_scale = parser.value_float();

  const bool seenB = parser.seenval('B');
  if (seenB) thc.rate_toggle = parser.value_float();

  const bool seenG = parser.seenval('G');
  if (seenG) thc.pid_p = parser.value_float();

  //if (!(seenE || seenD || seenM || seenS || seenT || seenB || seenG))
    M783_report(false);
}


#endif
