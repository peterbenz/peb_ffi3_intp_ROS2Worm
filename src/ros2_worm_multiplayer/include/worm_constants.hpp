#pragma once

#ifndef _WORM_CONSTANTS_HPP
#define _WORM_CONSTANTS_HPP

#include <chrono>

/* Namespace for constants used in this project */
namespace WormConstants 
{
  constexpr std::chrono::milliseconds TICK_TIME{100};
  constexpr const int GRID_MESSAGE_QUEUE_LENGTH = 10;
}

/* Namespace for topics used in this project */
namespace WormTopics
{
  constexpr const char* PlayerInput = "PlayerInput";
  constexpr const char* GameStart = "GameStart";
  constexpr const char* BoardInfo = "BoardInfo";
}
#endif