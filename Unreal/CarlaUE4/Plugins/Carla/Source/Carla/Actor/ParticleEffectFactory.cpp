// SOS-Nav addition to CarlaAir: spawn particle effects (fire, smoke) as CARLA
// actors. The effects are UE4's starter content, cooked from
// /Game/StarterContent/Particles (Config/DefaultGame.ini).

#include "ParticleEffectFactory.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include "Particles/Emitter.h"
#include "Particles/ParticleSystem.h"
#include "Particles/ParticleSystemComponent.h"

namespace
{
  FActorDefinition MakeEffectDefinition(const FString &Id, const FString &DefaultPath)
  {
    using ABFL = UActorBlueprintFunctionLibrary;
    auto Definition = ABFL::MakeGenericDefinition(TEXT("static"), TEXT("effect"), Id);
    Definition.Class = AEmitter::StaticClass();
    Definition.Variations.Emplace(FActorVariation{
        TEXT("particle_path"), EActorAttributeType::String, {DefaultPath}, false});
    Definition.Variations.Emplace(FActorVariation{
        TEXT("scale"), EActorAttributeType::Float, {TEXT("1.0")}, false});
    return Definition;
  }
}

TArray<FActorDefinition> AParticleEffectFactory::GetDefinitions()
{
  return {
      MakeEffectDefinition(TEXT("fire"), TEXT("/Game/StarterContent/Particles/P_Fire.P_Fire")),
      MakeEffectDefinition(TEXT("smoke"), TEXT("/Game/StarterContent/Particles/P_Smoke.P_Smoke")),
      MakeEffectDefinition(TEXT("particle"), TEXT("/Game/StarterContent/Particles/P_Fire.P_Fire"))};
}

FActorSpawnResult AParticleEffectFactory::SpawnActor(
    const FTransform &SpawnAtTransform,
    const FActorDescription &ActorDescription)
{
  using ABFL = UActorBlueprintFunctionLibrary;
  auto *World = GetWorld();
  if (World == nullptr)
  {
    UE_LOG(LogCarla, Error, TEXT("AParticleEffectFactory: cannot spawn into an empty world."));
    return {};
  }
  const FString Path = ActorDescription.Variations.Contains("particle_path")
      ? ABFL::ActorAttributeToString(ActorDescription.Variations["particle_path"], "")
      : FString();
  UParticleSystem *Template = LoadObject<UParticleSystem>(nullptr, *Path);
  if (Template == nullptr)
  {
    UE_LOG(LogCarla, Warning, TEXT("AParticleEffectFactory: no particle system at %s"), *Path);
    FActorSpawnResult Result;
    Result.Status = EActorSpawnResultStatus::InvalidDescription;
    return Result;
  }
  const float Scale = ABFL::RetrieveActorAttributeToFloat("scale", ActorDescription.Variations, 1.0f);
  FTransform ScaledTransform(SpawnAtTransform);
  ScaledTransform.SetScale3D(FVector(Scale));

  FActorSpawnParameters SpawnParameters;
  SpawnParameters.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
  auto *Emitter = World->SpawnActor<AEmitter>(AEmitter::StaticClass(), ScaledTransform, SpawnParameters);
  if (Emitter == nullptr)
  {
    return {};
  }
  Emitter->SetTemplate(Template);
  Emitter->GetParticleSystemComponent()->bAutoDestroy = false;
  Emitter->GetParticleSystemComponent()->ActivateSystem(true);
  return FActorSpawnResult(Emitter);
}
