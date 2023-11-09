#include <cstdio>
#include <curses.h>
#include <cinttypes>
#include "rclcpp/rclcpp.hpp"
#include "ros2_worm_multiplayer/msg/direction.hpp"
#include "worm_constants.hpp"

/**
 * @brief
*/
class Navigation : public rclcpp::Node
{
	public:
		Navigation();
		void inputLoop();

	private:
		void initializeCursesApplication();

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
	/* NCurses Init */
	this->initializeCursesApplication();

	/* rclcpP::QoS(10) provides default QoS profile with history depth of 10 */
	this->direction_pub_ = this->create_publisher<ros2_worm_multiplayer::msg::Direction>("PlayerInput", rclcpp::QoS(10));

	/* Create a timer to check for keyboard input every 100ms */
	this->timer_ = create_wall_timer(WormConstants::TICK_TIME, std::bind(&Navigation::inputLoop, this));
}

/**
 * 
*/
void Navigation::inputLoop()
{
	/* Check if a key is pressed */
	int key;
	bool dirty = false;

	if ((key = getch()) != ERR)
	{
		/* msg to store values */
		ros2_worm_multiplayer::msg::Direction wormDirection;

	  switch (key)
	  {
	  case 'q':
	  	// todo
			dirty = true;
	  	break;
	
	  case KEY_UP :
			wormDirection.dx = 0;
			wormDirection.dy = -1;
			dirty = true;
	  	break;

	  case KEY_DOWN :
			wormDirection.dx = 0;
			wormDirection.dy = 1;
			dirty = true;
	  	break;

	  case KEY_LEFT :
			wormDirection.dx = -1;
			wormDirection.dy = 0;
			dirty = true;
	  	break;

	  case KEY_RIGHT :
			wormDirection.dx = 1;
			wormDirection.dy = 0;
			dirty = true;
	  	break;
	  }

		if (dirty == true)
		{
			this->direction_pub_->publish(wormDirection);
			dirty = false;
		}
	}
	return;
}

/**
 * 
*/
void Navigation::initializeCursesApplication()
{
	initscr(); // Initialize the curses screen

	// Note:
	// The call to initscr() defines various global variables of the curses framework.
	// stdscr, LINES, COLS, TRUE, FALSE

	noecho();  // Characters typed ar not echoed
	cbreak();  // No buffering of stdin
	nonl();    // Do not translate 'return key' on keyboard to newline character
	keypad(stdscr, TRUE); // Enable the keypad
	curs_set(0);          // Make cursor invisible
	// Begin in non-single-step mode (getch will not block)
	nodelay(stdscr, TRUE);  // make getch to be a non-blocking call
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