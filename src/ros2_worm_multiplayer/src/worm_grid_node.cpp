#include <cstdio>
#include "rclcpp/rclcpp.hpp"

/*
Topics ung Messages des Nodes:

  Publish:
    GameId auf Topic GameStart
    Boarddatenstruktur auf Topiv Board

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
    WormTest() : Node("worm_test") {
    }
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