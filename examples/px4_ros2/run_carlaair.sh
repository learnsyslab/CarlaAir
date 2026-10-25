#!/usr/bin/env bash
# Start a packaged CarlaAir release in the foreground with a given AirSim
# settings file.
#
# Usage: run_carlaair.sh --settings FILE [--release DIR] [--state-dir DIR]
#                        [--map MAP] [--res WxH] [--quality LEVEL] [--offscreen]
#
#   --settings   AirSim settings.json to use (required)
#   --release    unpacked CarlaAir release (default: $CARLAAIR_RELEASE or
#                ~/workspace_sim/CarlaAir-v0.1.7)
#   --state-dir  private home for this instance (default: a temporary dir)
#   --map        CARLA map (default: Town10HD)
#   --res        window size (default: 960x540)
#   --quality    Low, Medium, High or Epic (default: Low)
#   --offscreen  render without a window. CarlaAir starts its own drone control
#                on the first WASD/Space/Shift key in its window; with a PX4
#                vehicle that fights PX4's flight modes and can abort the engine.
#
# CarlaAir ignores AirSim's -settings= argument and always reads
# ~/Documents/AirSim/settings.json. To leave the user's own settings alone,
# the engine runs with HOME set to a private state directory that holds the
# requested settings. The engine ignores SIGTERM; stop it with SIGKILL.
set -euo pipefail

release="${CARLAAIR_RELEASE:-${HOME}/workspace_sim/CarlaAir-v0.1.7}"
settings=""
state_dir=""
map="Town10HD"
resolution="960x540"
quality="Low"
offscreen=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --settings) settings="$2"; shift 2 ;;
    --release) release="$2"; shift 2 ;;
    --state-dir) state_dir="$2"; shift 2 ;;
    --map) map="$2"; shift 2 ;;
    --res) resolution="$2"; shift 2 ;;
    --quality) quality="$2"; shift 2 ;;
    --offscreen) offscreen=1; shift ;;
    -h|--help) sed -n '2,22p' "$0" | cut -c3-; exit 0 ;;
    *) echo "ERROR: unknown option $1" >&2; exit 2 ;;
  esac
done
binary="${release}/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping"
[[ -n "${settings}" && -f "${settings}" ]] || { echo "ERROR: --settings FILE is required" >&2; exit 2; }
[[ -x "${binary}" ]] || { echo "ERROR: CarlaAir binary not found: ${binary}" >&2; exit 2; }
[[ "${resolution}" =~ ^([0-9]+)x([0-9]+)$ ]] || { echo "ERROR: --res must be WxH" >&2; exit 2; }
width="${BASH_REMATCH[1]}"
height="${BASH_REMATCH[2]}"

state_dir="${state_dir:-$(mktemp -d -t carlaair-state.XXXXXX)}"
mkdir -p "${state_dir}/Documents/AirSim"
cp "${settings}" "${state_dir}/Documents/AirSim/settings.json"

# Same renderer settings as CarlaAir.sh (its v0.1.7 "VSync fix"): without the
# VSync and 60 FPS cap the engine renders uncapped, which has ended in Vulkan
# device-lost crashes (NVIDIA Xid 31).
runtime_ini="${state_dir}/.config/Epic/CarlaUE4/Saved/Config/LinuxNoEditor/GameUserSettings.ini"
mkdir -p "$(dirname -- "${runtime_ini}")"
cat >"${runtime_ini}" <<'INI'
[/Script/Engine.GameUserSettings]
bUseVSync=True
FullscreenMode=1
LastConfirmedFullscreenMode=1
FrameRateLimit=60.000000
Version=5
INI

icd_files="$(ls /usr/share/vulkan/icd.d/*.json /etc/vulkan/icd.d/*.json 2>/dev/null | paste -sd: || true)"
[[ -n "${icd_files}" ]] && export VK_ICD_FILENAMES="${icd_files}"
export HOME="${state_dir}"
cd "${release}"
window_args=(-windowed -ResX="${width}" -ResY="${height}")
[[ "${offscreen}" == 1 ]] && window_args=(-RenderOffScreen)
exec "${binary}" CarlaUE4 "${map}" "${window_args[@]}" \
  -carla-rpc-port=2000 -quality-level="${quality}" -TexturePoolSize=2048 \
  -unattended -nosound -UseVSync
