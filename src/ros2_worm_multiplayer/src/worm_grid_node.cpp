#include <cstdio>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "constants.hpp"

/*
Topics und Messages des Nodes:

  Publish:
    GameId auf Topic GameStart
    Boarddatenstruktur auf Topic Board

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

static const auto GameStartTopic = "GameStart";

class WormGridNode : public rclcpp::Node {
  public:
    WormGridNode() : Node("worm_grid_node"), count_(0) {
      gameId_publisher_ = this->create_publisher<std_msgs::msg::String>(GameStartTopic, 10);
      gameId_timer_ = this->create_wall_timer(TICK_TIME, std::bind(&WormGridNode::GameIdPublishCallback, this));
    }

  private:
    // callback methods for publishing
    void GameIdPublishCallback() {}
    void BoardPublishCallback() {}

    // callback methods for subscribing
    void PlayerInputCallback() {}

};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WormGridNode>());
  rclcpp::shutdown();
  
  return 0;
}