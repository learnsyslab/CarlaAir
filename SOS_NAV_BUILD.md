# Building this CarlaAir fork (branch sos_nav_fx)

This branch supplies fire/smoke actors and environment Pause/Resume. Pause leaves
the drone, cameras and sensor time running. Its implementation is in the CarlaAir
project (`EnvironmentPause.cpp`, `AirSim.Build.cs`, `CarlaEpisode.h`); no Unreal
Engine source patches are required. Updating Python launchers alone does not
update the native `carla.EnvironmentPaused` command.

## Tested versions

| Component | Version |
| --- | --- |
| Native source | `learnsyslab/CarlaAir`, `sos_nav_fx`; Pause/Resume starts at `b518ee32c25182ebad9f5d0f0177c9e37edbf559` |
| Engine | `CarlaUnreal/UnrealEngine`, `carla`, `e9d9e60c85f643e10eeb03f42f61554d18dcb30f` (UE 4.26.2) |
| Engine toolchain | bundled `v17_clang-10.0.1-centos7`, bundled Mono |
| AirSim libraries | `v1.7.0-linux`, `0becf9148d88c19c3ffd69359ebe1f9c3ae43826` |
| StreetMap | `260273d6b7c3f28988cda31fd33441de7e272958` |
| CARLA content / Python API | `20250912_2171890` / fork's CPython 3.10 `carla-0.9.16` wheel |

The project includes private Renderer headers, so use this engine revision when
reproducing the build. The engine requires your own Epic-linked GitHub access.

## Install or update the ready-to-run simulator

Use the package produced by `carlaair_sim/setup/make_package.sh`. From that package,
with the simulator stopped:

```bash
bash setup.sh carlaair
```

This installs or replaces `~/workspace_sim/CarlaAir-sosnav`, including an already
installed executable, and installs the matching wheel. Set `SIM_DIR` for another
location. Update the `carlaair_sim` checkout to its `gofusion` branch containing
the Pause/Resume controls as well. The package's `sources` stage only creates
missing checkouts; it preserves existing checkouts and their local edits.

## Rebuild project C++ using the packaged dependencies

The setup package contains a committed native source bundle and
`carlaair_rebuild_inputs.tar`: the existing AirLib libraries/dependencies,
CarlaDependencies, StreetMap source, OptionalModules.ini, camera materials and
LibCarla client headers/libraries. These are required inputs, not Git changes.
The archive also carries the current build guide, build helper and `.gitmodules`.
It excludes engine source, intermediate files, IDE metadata and personal settings.

Prepare the engine above using its `Setup.sh`, `GenerateProjectFiles.sh` and build
instructions. Then, from the setup package (choose a new source directory):

```bash
git clone --branch sos_nav_fx payload/sources/CarlaAir-sos_nav_fx.bundle \
  ~/workspace_sim/carlaair_src
tar -xf payload/sources/carlaair_rebuild_inputs.tar -C ~/workspace_sim/carlaair_src
cd ~/workspace_sim/carlaair_src
UE4_ROOT=~/workspace_sim/UnrealEngine bash Util/BuildTools/BuildShipping.sh
```

The helper runs UnrealBuildTool with 22 workers and excludes only CPU 6 from the
caller's allowed CPU set. `JOBS` overrides the worker count. It uses the engine's
existing toolchain and Mono, without Docker or another compiler installation.
Ubuntu 24.04 can build the project with the supplied libraries; that does not
imply all third-party dependencies can be rebuilt against Ubuntu 24.04's glibc.

For C++ changes such as Pause/Resume, reuse the matching package's cooked content.
Stop the simulator, then install the new executable:

```bash
install -m 755 Unreal/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping \
  ~/workspace_sim/CarlaAir-sosnav/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping
```

Keep any `.sym`/`.debug` files with their matching executable for debugging; they
are omitted from the runtime package. Do not mix this executable with an
unrelated CARLA release's cooked content. The optional C++ camera bridge can use
`CARLA_DEPENDENCIES=<source>/PythonAPI/carla/dependencies` when building.

## Rebuilding dependencies or cooking changed assets

The packaged inputs avoid rebuilding dependencies for project-only C++ changes.
To regenerate them, the original environments were Ubuntu 22.04 for CARLA
(`make setup`, `make CarlaUE4Editor`, Python 3.10 wheel) and Ubuntu 20.04 with
clang 8 for AirSim `v1.7.0-linux` (`./setup.sh --no-full-poly-car`, `./build.sh`).
Copy AirSim's `AirLib/lib` and `AirLib/deps` into
`Unreal/CarlaUE4/Plugins/AirSim/Source/AirLib`. Mount the engine and source at their
actual paths when using a container. This repository does not supply the old
local Docker images or Dockerfiles.

Cooking requires additional source assets, which are not in the rebuild-input
archive: CARLA content `20250912_2171890` from the location in `Update.sh` and
`Util/ContentVersions.txt` goes in `Unreal/CarlaUE4/Content/Carla`; the matching
engine's `Samples/StarterContent/Content/StarterContent` goes in
`Unreal/CarlaUE4/Content/StarterContent`. Preserve the configured fire/smoke cook
directories in `DefaultGame.ini`. Camera `PostProcessingMaterials` come from
CARLA 0.9.16 and are included in the rebuild-input archive.

Before using `Util/BuildTools/Package.sh --config=Shipping --no-zip`, build the
editor and provide `Docs/python_api.md` from CARLA 0.9.16; the upstream packaging
script requires that file in addition to this fork's `Docs/release_readme.md`.
The runtime package already contains cooked content. A project C++ rebuild and
the runtime installation are checked separately from a complete dependency
rebuild and asset cook on a fresh machine.
