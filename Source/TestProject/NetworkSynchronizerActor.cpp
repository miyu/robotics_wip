// Fill out your copyright notice in the Description page of Project Settings.

#include "NetworkSynchronizerActor.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/World.h"
#include "ConstructorHelpers.h"

#include "net/Deserializer.hpp"
#include "ABoxActor.h"


// Sets default values
ANetworkSynchronizerActor::ANetworkSynchronizerActor()
{
   // Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
   PrimaryActorTick.bCanEverTick = true;

   // Find sphere asset
   static ConstructorHelpers::FObjectFinder<UStaticMesh> SphereVisualAsset(TEXT("/Game/StarterContent/Shapes/Shape_Sphere.Shape_Sphere"));
   UE_LOG(LogTemp, Warning, TEXT("ANetworkSynchronizerActor::ctor Sphere Asset Found? %d"), SphereVisualAsset.Succeeded());
   sphereMesh = SphereVisualAsset.Object;
}

void ANetworkSynchronizerActor::BeginPlay()
{
   Super::BeginPlay();
   UE_LOG(LogTemp, Warning, TEXT("ANetworkSynchronizerActor::BeginPlay Enter"));

   // HandleDataReceived(HARDCODED_PAYLOAD, HARDCODED_PAYLOAD_LENGTH);

   auto endpoint = UdpClient::CreateUdpEndpoint(L"255.255.255.255", 21337);
   udp = MakeShareable(new UdpClient(endpoint));
   udp->OnDataReceived().BindUObject(this, &ANetworkSynchronizerActor::HandleDataReceived);
}

void ANetworkSynchronizerActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
   Super::EndPlay(EndPlayReason);
   udp->Shutdown();
}

void ANetworkSynchronizerActor::Tick(float DeltaTime)
{
   Super::Tick(DeltaTime);
   //UE_LOG(LogTemp, Warning, TEXT("ANetworkSynchronizerActor::Tick Enter"));

   //const char* buff = "Hello!\0";
   //udp->Send(reinterpret_cast<const uint8*>(buff), strlen(buff) + 1);
}

void ANetworkSynchronizerActor::HandleDataReceived(const uint8 *data, const uint32 count)
{
   UE_LOG(LogTemp, Warning, TEXT("ANetworkSynchronizerActor::HandleDataReceived Enter (count = %d)"), count);

   //auto reader = MakeShareable(new FBufferReader((void*)HARDCODED_PAYLOAD, HARDCODED_PAYLOAD_LENGTH, false));
   auto reader = MakeShareable(new FBufferReader((void*)data, count, false));
   Deserializer deserializer{ reader };
   auto res = deserializer.Deserialize();

   UE_LOG(LogTemp, Warning, TEXT("ANetworkSynchronizerActor::HandleDataReceived Entity Count %d"), res.Num());

   AsyncTask(ENamedThreads::GameThread, [=]() {
      this->HandleDataReceived_InRenderThread(res);
   });
}

void ANetworkSynchronizerActor::HandleDataReceived_InRenderThread(TArray<RemotelySimulatedDartVisual> visuals)
{
   UE_LOG(LogTemp, Warning, TEXT("ANetworkSynchronizerActor::HandleDataReceived_InRenderThread Num %d"), visuals.Num());

   for (auto& oldActor : spawnedActors)
   {
      oldActor->Destroy();
   }
   spawnedActors.Empty();

   for (auto& visual : visuals)
   {
      UE_LOG(LogTemp, Warning, TEXT("ANetworkSynchronizerActor::HandleDataReceived_InRenderThread Type %s"), *(visual.shape));

      for (auto i = 0; i < 4; i++)
      {
         UE_LOG(LogTemp, Warning, TEXT("ANetworkSynchronizerActor::HandleDataReceived_InRenderThread %d: %f %f %f %f"), i, visual.matrix.M[i][0], visual.matrix.M[i][1], visual.matrix.M[i][2], visual.matrix.M[i][3]);
      }

      UE_LOG(LogTemp, Warning, TEXT("ANetworkSynchronizerActor::HandleDataReceived_InRenderThread Args: %f %f %f %f"), visual.param0, visual.param1, visual.param2, visual.param3);


      if (visual.shape == TEXT("BoxShape") || visual.shape == TEXT("EllipsoidShape")) {
         FTransform modelTransform{ FQuat::Identity, FVector::ZeroVector, FVector{ visual.param0, visual.param1, visual.param2 } };
         FTransform worldTransform{ visual.matrix.GetTransposed() };
         FTransform embiggen{ FQuat::Identity, FVector::ZeroVector, FVector{ 100, 100, 100 } };

         FActorSpawnParameters spawnInfo;
         spawnInfo.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
         auto actor = GetWorld()->SpawnActor<AABoxActor>(spawnInfo);
         actor->SetActorTransform(modelTransform * worldTransform * embiggen);

         if (visual.shape != TEXT("BoxShape"))
         {
            actor->MeshComp->SetStaticMesh(sphereMesh);
         }

         spawnedActors.Add(actor);
      }
      else if (visual.shape == TEXT("MeshShape")) {
         FTransform modelTransform{ FQuat::Identity, FVector{ visual.param3, visual.param4, visual.param5 }, FVector{ visual.param0, visual.param1, visual.param2 } };
         FTransform worldTransform{ visual.matrix.GetTransposed() };
         FTransform embiggen{ FQuat::Identity, FVector::ZeroVector, FVector{ 100, 100, 100 } };

         FActorSpawnParameters spawnInfo;
         spawnInfo.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
         auto actor = GetWorld()->SpawnActor<AABoxActor>(spawnInfo);
         actor->SetActorTransform(modelTransform * worldTransform * embiggen);
         spawnedActors.Add(actor);
      }
   }
}

