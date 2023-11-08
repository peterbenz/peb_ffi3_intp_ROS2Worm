#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


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
            subscription_ = this->create_subscription<std_msgs::msg::String>("topic", 10, std::bind(&Display::refreshD, this, _1));
            startup();
        }

    private:

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
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
        void refreshD(const std_msgs::msg::String& msg)
        {
          //todo


          //testing:
          RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());

          for (int y = 0; y <= MIN_NUMBER_OF_ROWS ; y++)
          {
            for (int x = 0; x <= MIN_NUMBER_OF_COLS ; x++)
            {
              placeItem(&theboard,y,x,BC_FOOD_2,SYMBOL_BARRIER,COLP_BARRIER);
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