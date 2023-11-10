#include <cstdio>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

#include "worm_constants.hpp"
#include "ros2_worm_multiplayer/msg/board.hpp"
#include "ros2_worm_multiplayer/msg/direction.hpp"

/*
Topics und Messages des Nodes:

  Publish:
    GameId auf Topic GameStart
    Boarddatenstruktur auf Topic BoardInfo

  Subscribe:
    Spielereingabe auf Topic PlayerInput

  Messages:  (Code von Peter muss noch integriert werden)
    GameId: int
    Borddatenstruktur: {(boardCharacter: char, playerId: int), ... , (boardCharacter: char, playerId: int)}
      --> vllt Farbe des Boardzeichens in die Tupel der Datenstruktur aufnehmen

  Services:
    join --> dieser Node ist Service Server
      Request Args: t.b.d.
      Response Args: status: bool oder int, playerId: int
*/

class WormGridNode : public rclcpp::Node {
  public:
    WormGridNode();  // Constructor

  private:
    // publishers
    rclcpp::Publisher<std_msgs::msg::Int32> gameId_publisher_;
    rclcpp::Publisher<ros2_worm_multiplayer::msg::Board> boardInfo_publisher_;

    // subscribers
    rclcpp::Subscription<ros2_worm_multiplayer::msg::Direction> playerInput_subscription_;

    // timer for generating time ticks
    rclcpp::WallTimer tick_timer_;

    // callback methods for publishing
    void GameIdPublishCallback();
    void BoardInfoPublishCallback();

    // callback methods for subscribing
    void PlayerInputCallback();

    // method combining all the routines to be run in 1 tick
    void RunTick() {
      BoardInfoPublishCallback;
      GameIdPublishCallback();
    }
};

/**
 * @brief Construct the Node and initialize instance members.
*/
WormGridNode::WormGridNode() : Node("worm_grid_node") {
  gameId_publisher_ = this->create_publisher<std_msgs::msg::Int32>(WormTopics::GameStart, WormConstants::GRID_MESSAGE_QUEUE_LENGTH);
  boardInfo_publisher_ = this->create_publisher<std_msgs::msg::Int32>(WormTopics::BoardInfo, WormConstants::GRID_MESSAGE_QUEUE_LENGTH);

  tick_timer_ = this->create_wall_timer(
    WormConstants::TICK_TIME,
    std::bind(
      &WormGridNode::RunTick, 
      this
    )
  );

  playerInput_subscription_ = this->create_subscription<std_msgs::msg::String>(
    WormTopics::PlayerInput, WormConstants::GRID_MESSAGE_QUEUE_LENGTH, std::bind(&WormGridNode::PlayerInputCallback, this, _1));
}

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WormGridNode>());
  rclcpp::shutdown();
  
  return 0;
}