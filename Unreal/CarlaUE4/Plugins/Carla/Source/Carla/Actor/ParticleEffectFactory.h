// SOS-Nav addition to CarlaAir: spawn particle effects (fire, smoke) as CARLA
// actors, e.g. world.spawn_actor(library.find("static.effect.fire"), transform).

#pragma once

#include "Carla/Actor/ActorSpawnResult.h"
#include "Carla/Actor/CarlaActorFactory.h"

#include "ParticleEffectFactory.generated.h"

/// Factory of particle-effect actors (UE4 Cascade emitters):
/// "static.effect.fire", "static.effect.smoke" and "static.effect.particle"
/// (any ParticleSystem asset, attribute "particle_path"). All take "scale".
UCLASS()
class CARLA_API AParticleEffectFactory : public ACarlaActorFactory
{
  GENERATED_BODY()

  TArray<FActorDefinition> GetDefinitions() final;

  FActorSpawnResult SpawnActor(
      const FTransform &SpawnAtTransform,
      const FActorDescription &ActorDescription) final;
};
