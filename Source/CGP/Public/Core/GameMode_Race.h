// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameMode.h"
#include "GameMode_Race.generated.h"

/**
 * 
 */
UCLASS()
class CGP_API AGameMode_Race : public AGameMode
{
	GENERATED_BODY()
	
public:
	UFUNCTION(BlueprintCallable)
	void StartGame();
};
