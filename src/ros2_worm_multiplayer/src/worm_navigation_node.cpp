#include <cstdio>
#include <cinttypes>
#include "rclcpp/rclcpp.hpp"
#include "ros2_worm_multiplayer/msg/direction.hpp"

/**
 * @brief
*/
class Navigation : public rclcpp::Node
{
	public:
		Navigation();
		void inputLoop();

	private:
		/* Direction */
		int8_t dx_, dy_;

		/* Timer for keyboard read */
		rclcpp::TimerBase::SharedPtr timer_;

		/* Create an instance of an publisher with the Direction msg type */
		rclcpp::Publisher<ros2_worm_multiplayer::msg::Direction>::SharedPtr direction_pub_;
};

/**
 * @brief Initialize navigation node 
*/
Navigation::Navigation()
: Node("navigation_node"), dx_(0), dy_(0)
{
	/* rclcpP::QoS(10) provides default QoS profile with history depth of 10 */
	this->direction_pub_ = this->create_publisher<ros2_worm_multiplayer::msg::Direction>("PlayerInput", rclcpp::QoS(10));

	/* Create a timer to check for keyboard input every 100ms */
	this->timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&Navigation::inputLoop, this));
}

/**
 * 
*/
void Navigation::inputLoop()
{
	// TODO
}

/* */
int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);

	/* ::spin is used to enter a loop that keeps the node running */
	rclcpp::spin(std::make_shared<Navigation>());

	/* Reached when the node shutsdown */
	rclcpp::shutdown();

	return 0;
}
