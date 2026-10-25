#!/usr/bin/env bash
# Build a PX4 SITL rootfs for CarlaAir/AirSim next to an existing PX4 build.
#
# Usage: make_px4_rootfs.sh PX4_ETC_DIR OUTPUT_DIR [AIRFRAME_FILE...]
#
# PX4_ETC_DIR is the "etc" directory of a PX4 SITL build (the one passed as
# the last argument to the px4 binary). OUTPUT_DIR becomes a symlinked copy
# of it with these changes:
# - px4-rc.simulator no longer switches to one EKF per IMU (EKF2_MULTI_IMU 3,
#   SENS_IMU_MODE 0). That suits Gazebo's several IMUs; with AirSim's single
#   IMU it leaves no selectable estimator. PX4's defaults (single EKF) remain.
# - simulator_mavlink starts in the foreground, so the rest of the startup
#   waits until AirSim is connected. AirSim stamps its sensor data with Unix
#   time; PX4's lockstep clock jumps there on the first sample, and modules
#   that scheduled timers before the jump (manual_control, battery and power
#   simulators, ...) never run again: no manual control, so no POSCTL.
#   Start AirSim before PX4.
# - px4-rc.mavlink also starts the control link AirSim expects
#   (settings.json ControlPortRemote 14581 -> ControlPortLocal 14541, plus the
#   PX4 instance). Without it AirSim never learns PX4's home position, and any
#   AirSim API control (CarlaAir enables it on the first WASD key in its window)
#   ends in an uncaught "no valid GPS home location" exception that kills the
#   engine. Port 14540 stays free for MAVROS.
# - 22180_airsim_quad and any further AIRFRAME_FILEs are added to
#   init.d-posix/airframes; select one with PX4_SIM_MODEL=<name after the id>.
#
# Start PX4 with: PX4_SIM_MODEL=airsim_quad px4 -d -s OUTPUT_DIR/init.d-posix/rcS OUTPUT_DIR
set -euo pipefail

[[ $# -ge 2 ]] || { sed -n '4p' "$0" | cut -c3-; exit 2; }
px4_etc="$(cd -- "$1" && pwd)"
output="$2"
shift 2
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
simulator_rc="${px4_etc}/init.d-posix/px4-rc.simulator"
[[ -f "${simulator_rc}" ]] || { echo "ERROR: not a PX4 SITL etc directory: ${px4_etc}" >&2; exit 2; }

rm -rf "${output}"
mkdir -p "$(dirname -- "${output}")"
cp -rs "${px4_etc}" "${output}"

rm "${output}/init.d-posix/px4-rc.simulator"
grep -vE '^[[:space:]]*param set-default (EKF2_MULTI_IMU 3|SENS_IMU_MODE 0)[[:space:]]*$' \
  "${simulator_rc}" | sed -E 's/^([[:space:]]*simulator_mavlink start .*[^[:space:]])[[:space:]]*&[[:space:]]*$/\1/' \
  > "${output}/init.d-posix/px4-rc.simulator"
grep -qE '^[[:space:]]*simulator_mavlink start .*&[[:space:]]*$' "${output}/init.d-posix/px4-rc.simulator" && {
  echo "ERROR: could not start simulator_mavlink in the foreground" >&2
  exit 1
}
removed=$(( $(wc -l < "${simulator_rc}") - $(wc -l < "${output}/init.d-posix/px4-rc.simulator") ))
[[ "${removed}" == 2 ]] || {
  echo "ERROR: expected to remove 2 multi-IMU lines from px4-rc.simulator, removed ${removed}" >&2
  exit 1
}

mavlink_rc="${output}/init.d-posix/px4-rc.mavlink"
rm "${mavlink_rc}"
cp "${px4_etc}/init.d-posix/px4-rc.mavlink" "${mavlink_rc}"
cat >>"${mavlink_rc}" <<'RC'

# CarlaAir/AirSim control link (settings.json ControlPortRemote/ControlPortLocal).
mavlink start -x -u $((14581+px4_instance)) -r 4000000 -m onboard -o $((14541+px4_instance))
RC

for airframe in "${here}/22180_airsim_quad" "$@"; do
  install -m 0644 "${airframe}" "${output}/init.d-posix/airframes/"
done
