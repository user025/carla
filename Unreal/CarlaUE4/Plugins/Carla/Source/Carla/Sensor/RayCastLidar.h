// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "Carla/Sensor/Sensor.h"

#include "Carla/Actor/ActorDefinition.h"
#include "Carla/Sensor/LidarDescription.h"
#include "PhysicalMaterials/PhysicalMaterial.h"

#include <compiler/disable-ue4-macros.h>
#include <vector>
#include <carla/sensor/s11n/LidarMeasurement.h>
#include <compiler/enable-ue4-macros.h>
#include "RayCastLidar.generated.h"



/// A ray-cast based Lidar sensor.
UCLASS()
class CARLA_API ARayCastLidar : public ASensor
{
  GENERATED_BODY()

  using FLidarMeasurement = carla::sensor::s11n::LidarMeasurement;

public:

  static FActorDefinition GetSensorDefinition();

  ARayCastLidar(const FObjectInitializer &ObjectInitializer);

  void Set(const FActorDescription &Description) override;

  void Set(const FLidarDescription &LidarDescription);

protected:

  virtual void Tick(float DeltaTime) override;

private:

  /// Creates a Laser for each channel.
  void CreateLasers();

  /// run one channel shoot.
  void ShootOneChannel(uint32 PointsToScanWithOneLaser, uint32 Channel, float StartAngle, float AngleStep);

  /// Updates LidarMeasurement with the points read in DeltaTime.
  void ReadPoints(float DeltaTime);

  /// Shoot a laser ray-trace, return whether the laser hit something.
  bool ShootLaser(const uint32 Channel, const std::vector<float>& HorizontalAngles, std::vector<FVector> &XYZs, std::vector<uint32> &hit_ats);


  UPROPERTY(EditAnywhere)
  FLidarDescription Description;

  TArray<float> LaserAngles;

  FLidarMeasurement LidarMeasurement;
};
