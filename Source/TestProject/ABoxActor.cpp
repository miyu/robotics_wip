// Fill out your copyright notice in the Description page of Project Settings.

#include "ABoxActor.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "ConstructorHelpers.h"


// Sets default values
AABoxActor::AABoxActor()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

   SceneComp = CreateDefaultSubobject<USceneComponent>(TEXT("Scene"));
   SceneComp->SetWorldScale3D(FVector{ 100, 100, 100 });

   MeshComp = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Mesh"));

   static ConstructorHelpers::FObjectFinder<UStaticMesh> CubeVisualAsset(TEXT("/Game/StarterContent/Shapes/Shape_Cube.Shape_Cube"));
   UE_LOG(LogTemp, Warning, TEXT("AABoxActor::ctor Cube Asset Found? %d"), CubeVisualAsset.Succeeded());
   if (CubeVisualAsset.Succeeded())
   {
      MeshComp->SetStaticMesh(CubeVisualAsset.Object);
      
      // Out of the box cube is 100x100x100, so scale down to unit.
      MeshComp->AddRelativeLocation(FVector{ 0, 0, -0.5 });
      MeshComp->SetWorldScale3D(FVector{ 0.01 });

      MeshComp->AttachToComponent(SceneComp, FAttachmentTransformRules::KeepRelativeTransform);
   }

   RootComponent = SceneComp;
}

// Called when the game starts or when spawned
void AABoxActor::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void AABoxActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

