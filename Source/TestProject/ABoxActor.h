// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ABoxActor.generated.h"

UCLASS()
class TESTPROJECT_API AABoxActor : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AABoxActor();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

   UPROPERTY(BlueprintReadOnly, VisibleAnywhere, Category = "Scene")
   USceneComponent *SceneComp;

   UPROPERTY(BlueprintReadOnly, VisibleAnywhere, Category = "Mesh")
   UStaticMeshComponent *MeshComp;
};
