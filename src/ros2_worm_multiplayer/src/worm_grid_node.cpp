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

#include "ros2_worm_multiplayer/srv/join_server.hpp"

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

    void handleJoin(
      const std::shared_ptr<ros2_worm_multiplayer::srv::JoinServer::Request> request,
      std::shared_ptr<ros2_worm_multiplayer::srv::JoinServer::Response> response
    );
    std::vector<int32_t> joinedPlayers;

  private:
    // publishers
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr gameId_publisher_;
    rclcpp::Publisher<ros2_worm_multiplayer::msg::Board>::SharedPtr boardInfo_publisher_;

    // subscribers
    rclcpp::Subscription<ros2_worm_multiplayer::msg::Direction>::SharedPtr playerInput_subscription_;

    // services
    rclcpp::Service<ros2_worm_multiplayer::srv::JoinServer>::SharedPtr join_service_;

    // timer for generating time ticks
    rclcpp::TimerBase::SharedPtr tick_timer_;

    // logical representation of the board
    ros2_worm_multiplayer::msg::Board Board;

    // methods to implement gameplay
    void runLobby();
    void runGame();
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

  // initialize tick timer
  tick_timer_ = this->create_wall_timer(
    WormConstants::TICK_TIME,
    std::bind(
      &WormGridNode::RunTick, 
      this
    )
  );

  // initialize player input subscription 
  playerInput_subscription_ = this->create_subscription<ros2_worm_multiplayer::msg::Direction>(
    WormTopics::PlayerInput, 
    WormConstants::GRID_MESSAGE_QUEUE_LENGTH, 
    std::bind(
      &WormGridNode::PlayerInputCallback,
      this,
      std::placeholders::_1
    )
  );

  // initialize join service server
  rclcpp::Service<ros2_worm_multiplayer::srv::JoinServer>::SharedPtr join_service_ = this->create_service<ros2_worm_multiplayer::srv::JoinServer>(
    WormServices::JoinService,
    std::bind(
      &WormGridNode::handleJoin,
      this,
      std::placeholders::_1,
      std::placeholders::_2
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

  // initialize player list
  joinedPlayers = std::vector<int32_t>();

  // send message of server starting to console
  RCLCPP_INFO(this->get_logger(), "Worm Grid Node started! GameId: %d", gameId);

  // start the lobby
  currentGameState = GameState::LOBBY;
}

/**
 * @brief Start the lobby for players to wait in.
*/
void WormGridNode::runLobby() {
  if (joinedPlayers.size() < WormConstants::MAX_PLAYERS) {
    GameIdPublishCallback();
  } else {
    currentGameState = GameState::GAME;
  }
}

/**
 * @brief Stop the waiting lobby and start the game.
*/
void WormGridNode::runGame() {

}

/**
 * @brief End the game and stop the grid node.
*/
void WormGridNode::endGame() {

}

/**
 * @brief Callback method to send GameId when waiting for players.
*/
void WormGridNode::GameIdPublishCallback() {
  static std_msgs::msg::Int32 message = std_msgs::msg::Int32();
  message.data = gameId;

  gameId_publisher_->publish(message);
  RCLCPP_INFO(this->get_logger(), "Publishing GameId %d!", gameId);
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
  
  // run the tick according to GameState
  switch (currentGameState) {
  case GameState::LOBBY:
    runLobby();
    break;

  case GameState::GAME:
    runGame();
    break;
  
  case GameState::ENDED:
    endGame();
    break;
  
  default:
    break;
  }
}

/**
 * @brief Handle joins and disconnects through the JoinServer service.
*/
void WormGridNode::handleJoin(
  const std::shared_ptr<ros2_worm_multiplayer::srv::JoinServer::Request> request,
  std::shared_ptr<ros2_worm_multiplayer::srv::JoinServer::Response> response
) {
  // Handle joining
  if (request->srv_request == WormConstants::ServiceRequests::SRV_JOIN) {
    // Only allow joining when GameState is LOBBY
    if (currentGameState != GameState::LOBBY) {
      response->set__wormid(WormConstants::INVALID_WORM_ID);
      return;
    }

    int32_t newWormId = std::rand();
    while (std::find(joinedPlayers.begin(), joinedPlayers.end(), newWormId) != joinedPlayers.end()) {
      // Increment newWormId until it is unique
      newWormId++;
    }
    RCLCPP_INFO(this->get_logger(), "Player joined. Given ID: %d", newWormId);
    response->set__wormid(newWormId);

  // Handle disconnecting
  } else if (request->srv_request == WormConstants::ServiceRequests::SRV_DISCONNECT) {
    RCLCPP_INFO(this->get_logger(), "Player %d left.", request->wormid);
    
    joinedPlayers.erase(
      std::remove(joinedPlayers.begin(), joinedPlayers.end(), request->wormid),
      joinedPlayers.end()
    );

    response->set__wormid(WormConstants::INVALID_WORM_ID);
  }
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