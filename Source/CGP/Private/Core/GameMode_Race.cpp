// Fill out your copyright notice in the Description page of Project Settings.


#include "Core/GameMode_Race.h"

#include "OnlineSessionSettings.h"
#include "OnlineSubsystem.h"
#include "Interfaces/OnlinePartyInterface.h"
#include "Interfaces/OnlineSessionInterface.h"

void AGameMode_Race::StartGame(){
    if(auto session =  IOnlineSubsystem::Get()->GetSessionInterface()) {
    }
}
