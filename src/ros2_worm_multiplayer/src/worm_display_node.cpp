#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ros2_worm_multiplayer/msg/board.hpp"


extern "C" 
{
  #include "worm.h"
  #include "board_model.h"
  #include "prep.h"
}



using std::placeholders::_1;
class Display : public rclcpp::Node
{
    public:
        Display() : Node("display")
        {
            subscription_ = this->create_subscription<ros2_worm_multiplayer::msg::Board>("Board", 10, std::bind(&Display::refreshD, this, _1));
            startup();
        }

    private:

        rclcpp::Subscription<ros2_worm_multiplayer::msg::Board>::SharedPtr subscription_;
        struct board theboard;
        int startup()
        {
          initializeCursesApplication();
          initializeColors();
          initializeBoard(&theboard);
          initializeLevel(&theboard); //todo remove
          refresh();
          return 0;

        }

        int shutdown()
        {
          cleanupCursesApp();
          return 0;
        }
        void refreshD(const ros2_worm_multiplayer::msg::Board& msg)
        {
          //todo

          int y = 0;
          for(auto &row : msg.board)
          {
            y++;
            int x = 0;
            for(auto &element : row.row)
            {
              int32_t color = element.color;
              ColorPairs colorPair = static_cast<ColorPairs>(color);
              char item = element.zeichen;

              BoardCodes boardCode = static_cast<BoardCodes>(boardCode);
              placeItem(&theboard, y, x, boardCode, item, colorPair); //color 0-256?, boardcord = 0 -> keine funktion
              x++;
            }
          }


          // Display all the updates through lncurses
          refresh();
        }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Display>());
  rclcpp::shutdown();
  return 0;
}