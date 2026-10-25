# Building this CarlaAir fork (branch sos_nav_fx)

Fire and smoke effects as CARLA actors (`static.effect.fire`, `static.effect.smoke`,
`static.effect.particle` with `particle_path`; all take `scale`):
`Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Actor/ParticleEffectFactory.*`, registered in
`Plugins/AirSim/Source/SimWorldGameMode.cpp`.

Build on Ubuntu 24.04 (everything under `~/workspace_sim/carlaair_src`):

1. Unreal Engine 4.26, CARLA fork (`git@github.com:CarlaUnreal/UnrealEngine.git`, branch
   `carla`; needs a GitHub account linked to Epic): `./Setup.sh && ./GenerateProjectFiles.sh && make`
   on the host (its bundled toolchain uses an old-glibc sysroot).
2. CARLA's dependencies (`make setup`) and the editor (`make CarlaUE4Editor`) in Ubuntu 22.04
   (`docker/Dockerfile`, image `carlaair-build:22.04`): Unreal's bundled linker cannot link
   against Ubuntu 24.04's glibc.
3. AirSim's prebuilt libraries: the plugin's AirLib headers are AirSim **v1.7.0-linux**; build
   that release in Ubuntu 20.04 (`docker/airsim/Dockerfile`, clang-8: `./setup.sh
   --no-full-poly-car && ./build.sh`) and copy `AirLib/lib` and `AirLib/deps` into
   `Unreal/CarlaUE4/Plugins/AirSim/Source/AirLib/`.
4. `Unreal/CarlaUE4/Plugins/Streetmap`: clone `carla-simulator/StreetMap` at the commit pinned in
   `Util/BuildTools/BuildUE4Plugins.sh` (an empty folder breaks the script).
5. Content (git-ignored): CARLA's content package `20250912_2171890.tar.gz` (Util/ContentVersions.txt,
   from carla-assets.s3.us-east-005.backblazeb2.com) into `Unreal/CarlaUE4/Content/Carla`, and UE's
   `Samples/StarterContent/Content/StarterContent` into `Unreal/CarlaUE4/Content/StarterContent`
   (P_Fire, P_Smoke; cooked via `Config/DefaultGame.ini`).
6. CARLA's camera post-processing materials (git-ignored in this fork, missing them crashes the
   engine when a CARLA camera spawns): copy `Unreal/CarlaUE4/Plugins/Carla/Content/PostProcessingMaterials`
   from carla-simulator/carla tag 0.9.16 (sparse checkout).
7. `Docs/` must exist (Package.sh copies it into the package; this fork has none).
8. Package in the 22.04 container: `make package ARGS="--config=Shipping --no-zip"`.

Run the docker steps as the user with the tree mounted at the same path, `HOME` inside the tree
and `UE4_ROOT` pointing at the engine.
