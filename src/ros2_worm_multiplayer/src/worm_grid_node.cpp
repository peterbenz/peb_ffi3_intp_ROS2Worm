// ############################################################################
// INCLUDES
// ############################################################################

#include <cstdio>
#include <ctime>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

#include "worm_constants.hpp"
#include "ros2_worm_multiplayer/msg/board.hpp"
#include "ros2_worm_multiplayer/msg/direction.hpp"
#include "ros2_worm_multiplayer/msg/element.hpp"

extern "C" {
#include <curses.h>
}


// ############################################################################
// GRID NODE CLASS DECLARATION
// ############################################################################

class WormGridNode : public rclcpp::Node {
  public:
    WormGridNode();  // Constructor

    enum GameState {
      INIT,
      LOBBY,
      GAME,
      ENDED
    };

    GameState currentGameState = GameState::INIT;

    const int32_t gameId = std::rand();

  private:
    // publishers
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr gameId_publisher_;
    rclcpp::Publisher<ros2_worm_multiplayer::msg::Board>::SharedPtr boardInfo_publisher_;

    // subscribers
    rclcpp::Subscription<ros2_worm_multiplayer::msg::Direction>::SharedPtr playerInput_subscription_;

    // timer for generating time ticks
    rclcpp::TimerBase::SharedPtr tick_timer_;

    // logical representation of the board
    ros2_worm_multiplayer::msg::Board Board;

    // methods to implement gameplay
    void startLobby();
    void startGame();
    void endGame();

    // callback methods for publishing
    void GameIdPublishCallback();
    void BoardInfoPublishCallback();

    // callback methods for subscribing
    void PlayerInputCallback(const ros2_worm_multiplayer::msg::Direction::SharedPtr direction);

    // method combining all the routines to be run in 1 tick
    void RunTick();
};


// ############################################################################
// NODE METHOD DEFINITIONS
// ############################################################################

/**
 * @brief Construct the Node and initialize instance members.
*/
WormGridNode::WormGridNode() : Node("worm_grid_node") {
  gameId_publisher_ = this->create_publisher<std_msgs::msg::Int32>(WormTopics::GameStart, WormConstants::GRID_MESSAGE_QUEUE_LENGTH);
  boardInfo_publisher_ = this->create_publisher<ros2_worm_multiplayer::msg::Board>(WormTopics::BoardInfo, WormConstants::GRID_MESSAGE_QUEUE_LENGTH);

  tick_timer_ = this->create_wall_timer(
    WormConstants::TICK_TIME,
    std::bind(
      &WormGridNode::RunTick, 
      this
    )
  );

  playerInput_subscription_ = this->create_subscription<ros2_worm_multiplayer::msg::Direction>(
    WormTopics::PlayerInput, 
    WormConstants::GRID_MESSAGE_QUEUE_LENGTH, 
    std::bind(
      &WormGridNode::PlayerInputCallback,
      this,
      std::placeholders::_1
    )
  );

  // initialize board
  Board = ros2_worm_multiplayer::msg::Board();
  
  auto boardVector = std::vector<ros2_worm_multiplayer::msg::Row>();
  auto currentRow = std::vector<ros2_worm_multiplayer::msg::Element>();
  auto currentElement = ros2_worm_multiplayer::msg::Element();
  
  for (int y = 0; y < WormConstants::BOARD_HEIGHT; y++) {

    for (int x = 0; x < WormConstants::BOARD_LENGTH; x++) {
      currentElement.set__color(COLOR_BLACK);
      currentElement.set__zeichen(WormConstants::WormCharacters::EMPTY);
      currentRow.push_back(currentElement);
    }
    boardVector.push_back(ros2_worm_multiplayer::msg::Row().set__row(currentRow));
    currentRow.clear();
  }
  Board.set__board(boardVector);
}

/**
 * @brief Start the lobby for players to wait in.
*/
void WormGridNode::startLobby() {
  currentGameState = GameState::LOBBY;

}

/**
 * @brief Stop the waiting lobby and start the game.
*/
void WormGridNode::startGame() {
  currentGameState = GameState::GAME;

}

/**
 * @brief End the game and stop the grid node.
*/
void WormGridNode::endGame() {
  currentGameState = GameState::ENDED;

}

/**
 * @brief Callback method to send GameId when waiting for players.
*/
void WormGridNode::GameIdPublishCallback() {
  static std_msgs::msg::Int32 message = std_msgs::msg::Int32();
  message.data = gameId;

  if (currentGameState == GameState::LOBBY) {
    gameId_publisher_->publish(message);
  }

}

/**
 * @brief Callback method to send board info each tick.
*/
void WormGridNode::BoardInfoPublishCallback() {
  boardInfo_publisher_->publish(Board);
}

/**
 * @brief Callback method to process player inputs.
*/
void WormGridNode::PlayerInputCallback(ros2_worm_multiplayer::msg::Direction::SharedPtr direction) {

}

/**
 * @brief Run all methods that need to be run in a tick.
*/
void WormGridNode::RunTick() {
  BoardInfoPublishCallback();
  GameIdPublishCallback();
}


// ############################################################################
// MAIN
// ############################################################################

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);

  std::srand(std::time(nullptr));

  rclcpp::spin(std::make_shared<WormGridNode>());
  rclcpp::shutdown();
  
  return 0;
}