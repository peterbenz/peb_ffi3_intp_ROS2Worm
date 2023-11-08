#include <cstdio>
#include "rclcpp/rclcpp.hpp"

extern "C" {
extern int worm_main();
}

class WormTest : public rclcpp::Node {
  public:
    WormTest() : Node("worm_test") {
      worm_main();
    }
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WormTest>());
  rclcpp::shutdown();
  
  return 0;
}
