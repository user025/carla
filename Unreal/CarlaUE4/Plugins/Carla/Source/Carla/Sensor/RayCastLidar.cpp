// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <thread>
#include <iostream>
#include <sys/time.h>
#include <sys/syscall.h>

#include "Carla.h"
#include "Carla/Sensor/RayCastLidar.h"

#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"

#include "DrawDebugHelpers.h"
#include "Engine/CollisionProfile.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "Runtime/Engine/Classes/Kismet/KismetMathLibrary.h"
#include "StaticMeshResources.h"
#include <thread>
#include <vector>

FActorDefinition ARayCastLidar::GetSensorDefinition()
{
  return UActorBlueprintFunctionLibrary::MakeLidarDefinition(TEXT("ray_cast"));
}

ARayCastLidar::ARayCastLidar(const FObjectInitializer& ObjectInitializer)
  : Super(ObjectInitializer)
{
  PrimaryActorTick.bCanEverTick = true;
  // PrimaryActorTick.bRunOnAnyThread = true;

  auto MeshComp = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("RootComponent"));
  MeshComp->SetCollisionProfileName(UCollisionProfile::NoCollision_ProfileName);
  MeshComp->bHiddenInGame = true;
  MeshComp->CastShadow = false;
  MeshComp->PostPhysicsComponentTick.bCanEverTick = false;
  RootComponent = MeshComp;
}


void ARayCastLidar::Set(const FActorDescription &ActorDescription)
{
  Super::Set(ActorDescription);
  FLidarDescription LidarDescription;
  UActorBlueprintFunctionLibrary::SetLidar(ActorDescription, LidarDescription);
  Set(LidarDescription);
}

void ARayCastLidar::Set(const FLidarDescription &LidarDescription)
{
  Description = LidarDescription;
  LidarMeasurement = FLidarMeasurement(Description.Channels);
  CreateLasers();
}

void ARayCastLidar::CreateLasers()
{
  const auto NumberOfLasers = Description.Channels;
  check(NumberOfLasers > 0u);
  const float DeltaAngle = NumberOfLasers == 1u ? 0.f :
    (Description.UpperFovLimit - Description.LowerFovLimit) /
    static_cast<float>(NumberOfLasers - 1);
  LaserAngles.Empty(NumberOfLasers);
  for(auto i = 0u; i < NumberOfLasers; ++i)
  {
    const float VerticalAngle =
      Description.UpperFovLimit - static_cast<float>(i) * DeltaAngle;
    LaserAngles.Emplace(VerticalAngle);
  }
}

void ARayCastLidar::Tick(const float DeltaTime)
{
  Super::Tick(DeltaTime);
#ifdef LIDAR_PROFILE
  struct timeval st, ed;
  int st_s, ed_s;
  gettimeofday(&st, NULL);
  st_s = st.tv_sec * 1000 + st.tv_usec / 1000;
  UE_LOG(
          LogCarla, 
          Warning, 
          TEXT("%s: tick data begin, delta time %f, at thread %ld"), 
          *GetName(),
          DeltaTime,
          (long int)syscall(SYS_gettid)
        ) 
#endif
  ReadPoints(DeltaTime);
#ifdef LIDAR_PROFILE
  gettimeofday(&ed, NULL);
  ed_s = ed.tv_sec * 1000 + ed.tv_usec / 1000;
  UE_LOG(
          LogCarla, 
          Warning, 
          TEXT("%s: tick data read end, delta time %f, at thread %ld, used %ld"), 
          *GetName(),
          DeltaTime, 
          (long int)syscall(SYS_gettid),
          ed_s - st_s
          );
#endif
  auto DataStream = GetDataStream(*this);
  DataStream.Send(*this, LidarMeasurement, DataStream.PopBufferFromPool());
}

void ARayCastLidar::ShootOneChannel(const uint32 PointsToScanWithOneLaser, const uint32 Channel, const float StartAngle, const float AngleStep) {
    std::vector<float> Angles;
    Angles.reserve(PointsToScanWithOneLaser);
    for (auto i = 0u; i < PointsToScanWithOneLaser; ++i)
    {
      Angles.emplace_back(StartAngle + AngleStep * i);
    }

    std::vector<FVector> Points;
    Points.reserve(PointsToScanWithOneLaser);
    std::vector<uint32> actor_ids;
    actor_ids.reserve(PointsToScanWithOneLaser);
    
    if (ShootLaser(Channel, Angles, Points, actor_ids)) {
      for (auto Point=Points.begin(); Point!=Points.end(); Point++) {
        LidarMeasurement.WritePoint(Channel, *Point);
      }
      for (auto actor=actor_ids.begin(); actor!=actor_ids.end(); actor++) {
        LidarMeasurement.WriteGroundtruth(*actor);
      }
    }
}

void ARayCastLidar::ReadPoints(const float DeltaTime)
{
  const uint32 ChannelCount = Description.Channels;
  const uint32 PointsToScanWithOneLaser =
    FMath::RoundHalfFromZero(
        Description.PointsPerSecond * 0.1 / float(ChannelCount));

  if (PointsToScanWithOneLaser <= 0)
  {
    UE_LOG(
        LogCarla,
        Warning,
        TEXT("%s: no points requested this frame, try increasing the number of points per second."),
        *GetName());
    return;
  }

  check(ChannelCount == LaserAngles.Num());

  const float CurrentHorizontalAngle = LidarMeasurement.GetHorizontalAngle();
  const float AngleDistanceOfTick = Description.RotationFrequency * 360.0f * 0.1;
  const float AngleDistanceOfLaserMeasure = AngleDistanceOfTick / PointsToScanWithOneLaser;

  LidarMeasurement.Reset(ChannelCount * PointsToScanWithOneLaser);


  for (auto Channel = 0u; Channel < ChannelCount; ++Channel) {
      ShootOneChannel(PointsToScanWithOneLaser, Channel, CurrentHorizontalAngle, AngleDistanceOfLaserMeasure);
  }

  // UE_LOG(LogCarla, Warning, TEXT("%s shoot channel cost %ld"), *GetName(), (ed.tv_sec - st.tv_sec) * 1000 + ed.tv_usec / 1000 - st.tv_usec / 1000)

  // UE_LOG(LogCarla, Warning, TEXT("%s write %d points"), *GetName(), tot_num);
  const float HorizontalAngle = std::fmod(CurrentHorizontalAngle + AngleDistanceOfTick, 360.0f);
  LidarMeasurement.SetHorizontalAngle(HorizontalAngle);
}


bool ARayCastLidar::ShootLaser(const uint32 Channel, const std::vector<float>& HorizontalAngles, std::vector<FVector> &XYZs, std::vector<uint32> &hit_ats)
{
  const float VerticalAngle = LaserAngles[Channel];

  FCollisionQueryParams TraceParams = FCollisionQueryParams(FName(TEXT("Laser_Trace")), true, this);
  TraceParams.bTraceComplex = true;
  TraceParams.bReturnPhysicalMaterial = false;

  int points_to_go = HorizontalAngles.size();
  const auto Range = Description.Range;

  std::vector<FHitResult> HitInfos;
  HitInfos.reserve(points_to_go);
  for (int i=0; i<points_to_go; i++) {
    HitInfos.emplace_back(FHitResult(ForceInit));
  }

  FVector LidarBodyLoc = GetActorLocation();
  FRotator LidarBodyRot = GetActorRotation();

  std::vector<FVector> EndTraces;
  std::vector<FVector> StartTraces;
  EndTraces.reserve(points_to_go);
  StartTraces.reserve(points_to_go);

  for (auto HorizontalAngle=HorizontalAngles.begin(); HorizontalAngle != HorizontalAngles.end(); HorizontalAngle++) {
      FRotator LaserRot(VerticalAngle, *HorizontalAngle, 0);  // float InPitch, float InYaw, float InRoll
      FRotator ResultRot = UKismetMathLibrary::ComposeRotators(
          LaserRot,
          LidarBodyRot
      );
      FVector EndTrace = Range * UKismetMathLibrary::GetForwardVector(ResultRot) + LidarBodyLoc;
      EndTraces.emplace_back(EndTrace);
      StartTraces.emplace_back(LidarBodyLoc);
  }

#ifdef LIDAR_PROFILE
  struct timeval st, ed;
  gettimeofday(&st, NULL);
#endif

  GetWorld()->LineTraceSingleMultiByChannel(
    HitInfos,
    StartTraces,
    EndTraces,
    ECC_MAX,
    TraceParams,
    FCollisionResponseParams::DefaultResponseParam
  );

#ifdef LIDAR_PROFILE
    gettimeofday(&ed, NULL);
    UE_LOG(
          LogCarla, 
          Warning, 
          TEXT("%s: ray cost %d"), 
          *GetName(),
          (ed.tv_sec - st.tv_sec) * 1000 + ed.tv_usec / 1000 - st.tv_usec / 1000
        );
#endif

  FVector XYZ;
  for (auto HitInfo=HitInfos.begin(); HitInfo!=HitInfos.end(); HitInfo++) {
    if (HitInfo->bBlockingHit) {
         auto actor_temp = HitInfo->GetActor();
         hit_ats.emplace_back(GetEpisode().FindActor(actor_temp).GetActorId());
         XYZ = LidarBodyLoc - HitInfo->ImpactPoint;
         XYZ = UKismetMathLibrary::RotateAngleAxis(
           XYZ,
           -LidarBodyRot.Yaw + 90,
           FVector(0, 0, 1)
         );
         XYZs.emplace_back(XYZ);
    }
  }
  return true;
}
