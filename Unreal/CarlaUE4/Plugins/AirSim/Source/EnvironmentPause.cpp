#include "Engine/World.h"
#include "Engine/Engine.h"
#include "EngineUtils.h"
#include "HAL/IConsoleManager.h"
#include "SceneView.h"
#include "SceneViewExtension.h"
#include "SceneRendering.h"
#include "Components/PrimitiveComponent.h"
#include "GameFramework/Info.h"
#include "GameFramework/Controller.h"
#include "GameFramework/SpectatorPawn.h"
#include "GameFramework/HUD.h"
#include "Camera/PlayerCameraManager.h"
#include "Carla/Sensor/Sensor.h"
#include "Carla/Vehicle/CarlaWheeledVehicle.h"
#include "Vehicles/Multirotor/FlyingPawn.h"
#include "Vehicles/ComputerVision/ComputerVisionPawn.h"
#include "SimMode/SimModeBase.h"
#include "PIPCamera.h"
#include "CameraDirector.h"

namespace
{
TAutoConsoleVariable<float> EnvironmentPauseTime(
    TEXT("r.EnvironmentPauseTime"), -1.0f,
    TEXT("Frozen environment animation time; negative values use the world clock."),
    ECVF_ReadOnly | ECVF_RenderThreadSafe);

TAutoConsoleVariable<float> EnvironmentTimeOffset(
    TEXT("r.EnvironmentTimeOffset"), 0.0f,
    TEXT("Elapsed pause time excluded from environment material animation."),
    ECVF_ReadOnly | ECVF_RenderThreadSafe);

struct FPausedBody
{
    TWeakObjectPtr<UPrimitiveComponent> Component;
    FVector Velocity;
    FVector AngularVelocity;
};

struct FEnvironmentPause : FSceneViewExtensionBase
{
    TWeakObjectPtr<UWorld> World;
    bool Paused = false;
    float Started = 0.0f;
    FSceneInterface* Scene;
    TMap<FSceneViewFamily*, FDelegateHandle> RenderHandles;
    int32 PreviousCaptureExtensions = 0;
    int32 PreviousParticleFreeze = 0;
    TArray<TWeakObjectPtr<AActor>> TickingActors;
    TArray<TWeakObjectPtr<UActorComponent>> TickingComponents;
    TArray<FPausedBody> Bodies;

    FEnvironmentPause(const FAutoRegister& AutoRegister, UWorld* InWorld)
        : FSceneViewExtensionBase(AutoRegister), World(InWorld), Scene(InWorld->Scene)
    {
        auto* CaptureExtensions = IConsoleManager::Get().FindConsoleVariable(TEXT("r.SceneCapture.EnableViewExtensions"));
        PreviousCaptureExtensions = CaptureExtensions->GetInt();
        CaptureExtensions->Set(1, ECVF_SetByCode);
    }

    void PrepareRender(FSceneViewFamily& Family)
    {
        if (Family.Scene != Scene || !Family.EngineShowFlags.Rendering
            || Family.GetFeatureLevel() < ERHIFeatureLevel::SM5 || RenderHandles.Contains(&Family)) return;
        const float FrozenTime = EnvironmentPauseTime.GetValueOnRenderThread();
        const float Offset = EnvironmentTimeOffset.GetValueOnRenderThread();
        if (FrozenTime < 0.0f && Offset == 0.0f) return;

        // This delegate runs after InitViews, before drawing. Family clocks stay
        // live so visibility, texture streaming and particle activation still work.
        RenderHandles.Add(&Family, GEngine->GetPreRenderDelegate().AddLambda([this, &Family, FrozenTime, Offset]()
        {
            for (const FSceneView* BaseView : Family.Views)
                if (!static_cast<const FViewInfo*>(BaseView)->CachedViewUniformShaderParameters) return;
            for (const FSceneView* BaseView : Family.Views)
            {
                auto& View = *static_cast<FViewInfo*>(const_cast<FSceneView*>(BaseView));
                auto& Uniforms = *View.CachedViewUniformShaderParameters;
                Uniforms.GameTime = (FrozenTime >= 0.0f ? FrozenTime : Family.CurrentWorldTime) - Offset;
                Uniforms.DeltaTime = FrozenTime >= 0.0f ? 0.0f : Family.DeltaWorldTime;
                Uniforms.PrevFrameGameTime = Uniforms.GameTime - Uniforms.DeltaTime;
                View.ViewUniformBuffer.UpdateUniformBufferImmediate(Uniforms);
            }
            FinishRender(Family);
        }));
    }

    void FinishRender(FSceneViewFamily& Family)
    {
        if (const auto* Handle = RenderHandles.Find(&Family))
        {
            GEngine->GetPreRenderDelegate().Remove(*Handle);
            RenderHandles.Remove(&Family);
        }
    }

    void SetupViewFamily(FSceneViewFamily& Family) override
    {
        // Scene captures in this engine call SetupViewFamily but skip the
        // PreRenderViewFamily hook used by the window viewport.
        if (Family.Scene != Scene || Family.Views.Num() == 0 || !Family.Views[0]->bIsSceneCapture) return;
        auto Extension = StaticCastSharedRef<FEnvironmentPause>(AsShared());
        ENQUEUE_RENDER_COMMAND(PrepareEnvironmentCapture)([Extension, &Family](FRHICommandListImmediate&)
        {
            Extension->PrepareRender(Family);
        });
    }

    void SetupView(FSceneViewFamily&, FSceneView&) override {}
    void BeginRenderViewFamily(FSceneViewFamily&) override {}
    void PreRenderViewFamily_RenderThread(FRHICommandListImmediate&, FSceneViewFamily& Family) override
    {
        PrepareRender(Family);
    }
    void PreRenderView_RenderThread(FRHICommandListImmediate&, FSceneView&) override {}
    void PostRenderViewFamily_RenderThread(FRHICommandListImmediate&, FSceneViewFamily& Family) override
    {
        FinishRender(Family);
    }

    static bool KeepRunning(AActor* Actor)
    {
        return Actor->IsA<AInfo>() || Actor->IsA<AController>() || Actor->IsA<AHUD>()
            || Actor->IsA<APlayerCameraManager>() || Actor->IsA<ASpectatorPawn>()
            || Actor->IsA<ASensor>() || Actor->IsA<ASimModeBase>()
            || Actor->IsA<AFlyingPawn>() || Actor->IsA<AComputerVisionPawn>()
            || Actor->IsA<APIPCamera>() || Actor->IsA<ACameraDirector>();
    }

    static void SetPhysics(UPrimitiveComponent* Component, bool Enabled)
    {
        auto* Vehicle = Cast<ACarlaWheeledVehicle>(Component->GetOwner());
        if (Vehicle && Vehicle->GetRootComponent() == Component)
            Vehicle->SetSimulatePhysics(Enabled);
        else
            Component->SetSimulatePhysics(Enabled);
    }

    void SetPaused(bool Value)
    {
        if (Paused == Value || !World.IsValid()) return;
        auto* Particles = IConsoleManager::Get().FindConsoleVariable(TEXT("FX.FreezeParticleSimulation"));

        if (Value)
        {
            Started = World->GetTimeSeconds();
            EnvironmentPauseTime->Set(Started, ECVF_SetByCode);
            PreviousParticleFreeze = Particles->GetInt();
            Particles->Set(1, ECVF_SetByCode);
            for (TActorIterator<AActor> It(World.Get()); It; ++It)
            {
                AActor* Actor = *It;
                if (KeepRunning(Actor)) continue;
                if (Actor->IsActorTickEnabled())
                {
                    TickingActors.Add(Actor);
                    Actor->SetActorTickEnabled(false);
                }
                TInlineComponentArray<UActorComponent*> Components(Actor);
                for (auto* Component : Components)
                {
                    if (Component->IsComponentTickEnabled())
                    {
                        TickingComponents.Add(Component);
                        Component->SetComponentTickEnabled(false);
                    }
                    auto* Primitive = Cast<UPrimitiveComponent>(Component);
                    if (Primitive && Primitive->IsSimulatingPhysics())
                    {
                        Bodies.Add({Primitive, Primitive->GetPhysicsLinearVelocity(),
                                    Primitive->GetPhysicsAngularVelocityInDegrees()});
                    }
                }
            }
            for (const auto& Body : Bodies)
                SetPhysics(Body.Component.Get(), false);
        }
        else
        {
            EnvironmentTimeOffset->Set(EnvironmentTimeOffset.GetValueOnGameThread() + World->GetTimeSeconds() - Started, ECVF_SetByCode);
            EnvironmentPauseTime->Set(-1.0f, ECVF_SetByCode);
            Particles->Set(PreviousParticleFreeze, ECVF_SetByCode);
            for (const auto& Body : Bodies)
            {
                if (auto* Component = Body.Component.Get())
                {
                    SetPhysics(Component, true);
                    Component->SetPhysicsLinearVelocity(Body.Velocity);
                    Component->SetPhysicsAngularVelocityInDegrees(Body.AngularVelocity);
                }
            }
            for (const auto& Component : TickingComponents)
                if (Component.IsValid()) Component->SetComponentTickEnabled(true);
            for (const auto& Actor : TickingActors)
                if (Actor.IsValid()) Actor->SetActorTickEnabled(true);
            Bodies.Reset();
            TickingComponents.Reset();
            TickingActors.Reset();
        }
        Paused = Value;
    }
};

TSharedPtr<FEnvironmentPause, ESPMode::ThreadSafe> Environment;
FDelegateHandle CleanupHandle;

void CleanupEnvironment(UWorld* World, bool, bool)
{
    if (!Environment || Environment->World.Get() != World) return;
    Environment->SetPaused(false);
    EnvironmentTimeOffset->Set(0.0f, ECVF_SetByCode);
    IConsoleManager::Get().FindConsoleVariable(TEXT("r.SceneCapture.EnableViewExtensions"))->Set(
        Environment->PreviousCaptureExtensions, ECVF_SetByCode);
    Environment.Reset();
    FWorldDelegates::OnWorldCleanup.Remove(CleanupHandle);
    CleanupHandle.Reset();
}

void SetEnvironmentPaused(const TArray<FString>& Args, UWorld* World)
{
    if (!World) return;
    if (!Environment)
    {
        Environment = FSceneViewExtensions::NewExtension<FEnvironmentPause>(World);
        CleanupHandle = FWorldDelegates::OnWorldCleanup.AddStatic(&CleanupEnvironment);
    }
    if (Args.Num() == 1 && (Args[0] == TEXT("0") || Args[0] == TEXT("1")))
        Environment->SetPaused(Args[0] == TEXT("1"));
    UE_LOG(LogTemp, Display, TEXT("Environment %s; drone and sensors running"),
           Environment->Paused ? TEXT("paused") : TEXT("running"));
}

FAutoConsoleCommandWithWorldAndArgs EnvironmentPauseCommand(
    TEXT("carla.EnvironmentPaused"),
    TEXT("0 resumes the environment; 1 freezes it while the drone and sensors keep running."),
    FConsoleCommandWithWorldAndArgsDelegate::CreateStatic(&SetEnvironmentPaused));
}
