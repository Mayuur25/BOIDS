// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Kismet/GameplayStatics.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Runtime/Core/Public/Async/ParallelFor.h"
#include "Math/UnrealMathUtility.h"
#include "Kismet/KismetMathLibrary.h"
#include "Components/SplineComponent.h"
#include "boidController2.generated.h"

UCLASS()
class BOIDS_API AboidController2 : public AActor
{
	GENERATED_BODY()

public:	
	// Sets default values for this actor's properties
	AboidController2();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller")
		class UStaticMesh* boidsMesh;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller Predator")
		class UStaticMesh* predatorMesh;
	UPROPERTY(EditAnywhere, Instanced, Category = "Path spline")
		USplineComponent* PathSpline;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller Predator")
		float scalePredator = 1.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller Predator")
		float visualRangePredator = 30;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller Predator")
		float speedLimitPredetor = 1;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller Predator")
		float seperationFactoPredator = 0.05f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller Predator")
		float huntFactorPredetor = 0.005f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller Predator")
		float dodgeFactorPredetor = 0.05f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller Colision")
		float collisionDodgeFact = 0.05f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller")
		float boidsRange = 45;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller")
		int boidsNum = 200;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller")
		float scaleBoids = 0.5f;
	// Speed Limit of boids
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller")
		float limit_Speed = 0.5f;
	// Turn Factor of Boids
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller")
		float factorT = 1;
	// Coherence Factors change
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller")
		float factorCoherence = 0.005f;
	// Alignment Factors change
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller")
		float factorAlignment = 0.05f;
	// Sepreation change
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller")
		float separation = 10;
	// Sepreation factor
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller")
		float factorSeperation = 0.05f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller")
		float visualRange = 16;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller")
		bool boidsIsfollowingSpline = false;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller")
		bool boidsActive = false;

	TArray<FTransform> newBoidsTransformsArray;
	TArray<FTransform> newPredatorsTransformsArray;
	TArray<FTransform> predBoidsTransformsArray;
	TArray<FTransform> predPredatorsTransformsArray;
	UInstancedStaticMeshComponent *BoidsMeshInstance;
	UInstancedStaticMeshComponent *PredatorsMeshInstance;

	FVector centerPositon;

	int currentMaxNeigbors = 0;
	FVector centeredBoid;

	TMap<int, TArray<FVector4>> boids;
	TArray<TArray<FVector4>> neighbors;
	TArray<FVector4> predators;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller")
	int alpha = 0;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Controller")
	int alphaMod = 10000;

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	void getNeighbors(int boidIndex, int boidFamily, int boidGlobalIndex, FVector location);

	void keepInBounds(FVector &direction, FVector4 position);

	void separate(int boidIndex, FVector &direction,FVector position);

	void averageAlignment(int boidIndex, FVector &direction, FVector startDirection);

	void cohesion(int boidIndex, FVector &direction, FVector location);

	void moveFromPredators(FVector &direction, FVector location);

	void huntBoids(FVector &direction, FVector location);

	void limitSpeed(FVector &direction);

	void limitPredatorSpeed(FVector &direction);

	FRotator calcNewDirection(FVector direction,FVector loc);

};
