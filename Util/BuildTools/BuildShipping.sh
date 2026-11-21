#!/usr/bin/env bash
# Build the native project against a prepared, unmodified CARLA UE 4.26 engine.
# Usage: UE4_ROOT=/path/to/UnrealEngine bash Util/BuildTools/BuildShipping.sh
set -Eeuo pipefail

CARLA_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
UE4_ROOT="${UE4_ROOT:-${CARLA_ROOT}/../UnrealEngine}"
UE4_ROOT="$(cd -- "${UE4_ROOT}" && pwd)"
JOBS="${JOBS:-22}"
UBT="${UE4_ROOT}/Engine/Binaries/DotNET/UnrealBuildTool.exe"
[[ -f "${UBT}" ]] || { echo "Build the engine first: ${UBT} is missing (see SOS_NAV_BUILD.md)." >&2; exit 1; }
[[ -f "${CARLA_ROOT}/Unreal/CarlaUE4/Plugins/AirSim/Source/AirLib/lib/libAirLib.a" ]] \
  || { echo "Unpack carlaair_rebuild_inputs.tar first (see SOS_NAV_BUILD.md)." >&2; exit 1; }

# Exclude CPU 6 only, retaining any affinity already set by the caller.
build_cpus="$(python3 - <<'PY'
import os
cpus = sorted(os.sched_getaffinity(0) - {6})
if not cpus:
    raise SystemExit('No build CPUs remain after excluding CPU 6.')
print(','.join(map(str, cpus)))
PY
)"
cd "${UE4_ROOT}"
set +u
source Engine/Build/BatchFiles/Linux/SetupEnvironment.sh -mono Engine/Build/BatchFiles/Linux
set -u
echo "Building CarlaUE4 Shipping: ${JOBS} workers, CPUs ${build_cpus}"
exec taskset -c "${build_cpus}" "${UE_MONO_DIR}/bin/mono" "${UBT}" CarlaUE4 Linux Shipping \
  "-project=${CARLA_ROOT}/Unreal/CarlaUE4/CarlaUE4.uproject" \
  "-MaxParallelActions=${JOBS}" -NoHotReloadFromIDE
