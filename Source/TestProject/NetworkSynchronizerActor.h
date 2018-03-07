// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Networking.h"
#include "GameFramework/Actor.h"

class AABoxActor;
struct RemotelySimulatedDartVisual;

#include "net/UdpClient.hpp"

#include "NetworkSynchronizerActor.generated.h"

UCLASS()
class TESTPROJECT_API ANetworkSynchronizerActor : public AActor
{
   GENERATED_BODY()
   
public:	
   // Sets default values for this actor's properties
   ANetworkSynchronizerActor();

protected:
   // Called when the game starts or when spawned
   virtual void BeginPlay() override;
   virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
   void HandleDataReceived(const uint8 *data, const uint32 count);
   void HandleDataReceived_InRenderThread(TArray<RemotelySimulatedDartVisual> visuals);

public:	
   // Called every frame
   virtual void Tick(float DeltaTime) override;

private:
   UStaticMesh * sphereMesh;
   TSharedPtr<UdpClient> udp;
   TArray<AABoxActor*> spawnedActors;
};
