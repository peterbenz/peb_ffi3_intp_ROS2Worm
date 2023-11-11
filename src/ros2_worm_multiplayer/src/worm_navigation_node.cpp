#include <cstdio>
#include <curses.h>
#include <cinttypes>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "ros2_worm_multiplayer/msg/player_input.hpp"
#include "ros2_worm_multiplayer/srv/join_server.hpp"
#include "worm_constants.hpp"

extern "C" {
#include "prep.h"
}

/**
 * @brief
*/
class Navigation : public rclcpp::Node
{
	public:
		Navigation();

	private:
		/* */
		void inputLoop();

		/* */
		void gamestart_callback(const std_msgs::msg::Int32& msg);

		/* */
		void gamelobby();

		/* Game Server IDs */
		std::vector<int> game_ids_;
		int cur_game_id;

		/* Msg type that includes direction that a specifc worm goes */
		ros2_worm_multiplayer::msg::PlayerInput pInput;

		/* Timer for keyboard read */
		rclcpp::TimerBase::SharedPtr timer_;

		/* Publisher for PlayerInputs */
		rclcpp::Publisher<ros2_worm_multiplayer::msg::PlayerInput>::SharedPtr pInput_pub;

		/* Subscriber to GameStart */
		rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gamestart_sub_;

		/* Join Service Client */
		rclcpp::Client<ros2_worm_multiplayer::srv::JoinServer>::SharedPtr client_;  
};


/**
 * @brief Initialize navigation node 
*/
Navigation::Navigation()
: Node("navigation_node")
{
	/* NCurses Init */
	initializeCursesApplication();

	/* Create Subscriber for GameStart waiting for a gameid to join */
	this->gamestart_sub_ = this->create_subscription<std_msgs::msg::Int32>(
		WormTopics::GameStart, 10, std::bind(&Navigation::gamestart_callback, this, std::placeholders::_1));

	/* Create Client for join request */
	//this->client_ = this->create_client<ros2_worm_multiplayer::srv::JoinServer>(WormServices::JoinService);

	/* rclcpP::QoS(10) provides default QoS profile with history depth of 10 */
	//this->pInput_pub = this->create_publisher<ros2_worm_multiplayer::msg::PlayerInput>(WormTopics::PlayerInput, rclcpp::QoS(10));

	/* Create a timer to check for keyboard input every 100ms */
	// this->timer_ = create_wall_timer(WormConstants::TICK_TIME, std::bind(&Navigation::inputLoop, this));
}


/**
 * 
*/
void Navigation::gamestart_callback(const std_msgs::msg::Int32& msg)
{
	// RCLCPP_INFO(this->get_logger(), "I found a Game with ID: '%d'", msg.data);
	this->game_ids_.push_back(msg.data);
	gamelobby();
}

/**
 * 
*/
void Navigation::gamelobby()
{
	clear();
	attron(COLOR_PAIR(1));

	for (const auto& id : this->game_ids_)
	{
		printw("Game available at: %d\n", id);
	}

	attroff(COLOR_PAIR(1));
	refresh();
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
	  switch (key)
	  {
	  case 'q':
	  	// todo
			dirty = true;
	  	break;
	
	  case KEY_UP :
			this->pInput.dir.dx = 0;
			this->pInput.dir.dy = -1;
			dirty = true;
	  	break;

	  case KEY_DOWN :
			this->pInput.dir.dx = 0;
			this->pInput.dir.dy = 1;
			dirty = true;
	  	break;

	  case KEY_LEFT :
			this->pInput.dir.dx = -1;
			this->pInput.dir.dy = 0;
			dirty = true;
	  	break;

	  case KEY_RIGHT :
			this->pInput.dir.dx = 1;
			this->pInput.dir.dy = 0;
			dirty = true;
	  	break;
	  }

		if (dirty == true)
		{
			this->pInput_pub->publish(this->pInput);
			dirty = false;
		}
	}
	return;
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