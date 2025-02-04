#include <cstdio>
#include <curses.h>
#include <cinttypes>
#include <string>
#include <iostream>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "ros2_worm_multiplayer/msg/player_input.hpp"
#include "ros2_worm_multiplayer/srv/join_server.hpp"
#include "worm_constants.hpp"

extern "C" {
#include "prep.h"
#include "worm.h"
}

/**
 * @brief
*/
class Navigation : public rclcpp::Node
{
	public:
		Navigation();

	private:

		/* Gamestate */
		enum gamestates 
		{
			PRELOBBY,
			LOBBY,
			INGAME,
			POSTGAME
		};

		/* */
		void start_gamelobby();

		/* */
		void inputLoop();

		/* */
		void gamestart_callback(const std_msgs::msg::Int32& msg);

		/* */
		void gamelobby();

		/* */
		void timer_callback();
		
		/* helper methods */
		void add_gameserver(const std_msgs::msg::Int32& msg);

		/* Game Server IDs */
		std::vector<int> game_ids_;
		int cur_game_id;

		/* */
		enum gamestates cur_gamestate;


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

		/* Callback Groups */
		rclcpp::CallbackGroup::SharedPtr client_cb_group_;
		rclcpp::CallbackGroup::SharedPtr sub_cb_group_;
};


/**
 * @brief Initialize navigation node 
*/
Navigation::Navigation()
: Node("navigation_node"), cur_game_id{-1}, cur_gamestate{PRELOBBY}
{
	/* NCurses Init */
	initializeCursesApplication();

	this->client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	this->sub_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

	/* Create Subscriber for GameStart waiting for a gameid to join */
	rclcpp::SubscriptionOptions sub_options;
	sub_options.callback_group = this->sub_cb_group_;
	this->gamestart_sub_ = this->create_subscription<std_msgs::msg::Int32>(
		WormTopics::GameStart, 10, std::bind(&Navigation::gamestart_callback, this, std::placeholders::_1), sub_options);

	/* Create Client for join request */
	this->client_ = this->create_client<ros2_worm_multiplayer::srv::JoinServer>(WormServices::JoinService,
		rmw_qos_profile_services_default, this->client_cb_group_);

	/* rclcpP::QoS(10) provides default QoS profile with history depth of 10 */
	this->pInput_pub = this->create_publisher<ros2_worm_multiplayer::msg::PlayerInput>(WormTopics::PlayerInput, rclcpp::QoS(10));

	/* Create a timer to check for keyboard input every 100ms */
	this->timer_ = create_wall_timer(WormConstants::TICK_TIME, std::bind(&Navigation::timer_callback, this));
}


/**
 * 
*/
void Navigation::start_gamelobby()
{
	/* gamelobby blocks until a game was selected */
	//this->gamelobby();

	if (this->cur_game_id > -1)
	{
		/* Create the actual join request */
		auto join_request = std::make_shared<ros2_worm_multiplayer::srv::JoinServer::Request>();
		join_request->gameid = this->cur_game_id;
		join_request->wormid = WormConstants::INVALID_WORM_ID;
		join_request->srv_request = WormConstants::ServiceRequests::SRV_JOIN;

		/* Waiting for service to wake up */
		while (!this->client_->wait_for_service(std::chrono::seconds(1)))
		{
			clear();
			attron(COLOR_PAIR(1));
			printw("Waiting for server...\n");
			attroff(COLOR_PAIR(1));
			refresh();
		}

		/* sending actual request to the service */
		auto future_response = this->client_->async_send_request(join_request);

		while (future_response.wait_for(WormConstants::RESPONSE_TIMEOUT) != std::future_status::ready)
		{
		  attron(COLOR_PAIR(1));
		  printw("We were not able to join this server");
		  attroff(COLOR_PAIR(1));
		  refresh();
		}

		auto response = future_response.get();

		attron(COLOR_PAIR(1));
		printw("Successfully joined server: %d\n playing as wormid: %d\n",this->cur_game_id, response->wormid);
		attroff(COLOR_PAIR(1));
		refresh();

		/* Store wormid for this node until the end of the game */
		this->pInput.wormid = response->wormid;

		this->cur_gamestate = INGAME;
	}
}


/**
 * Testing
*/
void Navigation::gamestart_callback(const std_msgs::msg::Int32& msg)
{
	/* collect all gameids in a vector list */
	this->add_gameserver(msg);
}


/**
 *  
*/
void Navigation::gamelobby()
{
	char input;

	if (this->game_ids_.size() > 0)
	{
	  clear();
	  attron(COLOR_PAIR(1));

	  printw("Game Lobby: \n");
	  for (int i = 0; i < this->game_ids_.size(); i++)
	  {
	   	printw("%d - Game ID: %d\n", i, this->game_ids_.at(i));
	  }

	  move(getmaxy(stdscr) - 1, 0);
	  printw("Chose a game to join %d - %ld: ", 0, this->game_ids_.size() - 1);

	  if ((input = getch()) != ERR && isdigit(input))
	  {
	  	input = std::atoi(&input);
	  	if (input < this->game_ids_.size())
	  	{
	  		this->cur_game_id = this->game_ids_.at(input);

				this->cur_gamestate = LOBBY;
	  	}
	  }

		attroff(COLOR_PAIR(1));
		refresh();
	}

	return;
}


/**
 * 
*/
void Navigation::timer_callback()
{
	switch (this->cur_gamestate)
	{
		case PRELOBBY:
			this->gamelobby();
			break;

		case LOBBY: 
			this->start_gamelobby();
			break;

		case INGAME:
			this->inputLoop();
			break;

		case POSTGAME:
			break;
	}
}


/**
 * @brief Adds a game id to the list of available game servers if it doesnt exist already
*/
void Navigation::add_gameserver(const std_msgs::msg::Int32& msg)
{
	if (std::find(this->game_ids_.begin(), this->game_ids_.end(), msg.data) == this->game_ids_.end())
	{
		this->game_ids_.push_back(msg.data);
	}
}


/**
 * 
*/
void Navigation::inputLoop()
{
	/* Check if a key is pressed */
	int key;
	bool dirty = false;

	clear();

	if ((key = getch()) != ERR)
	{
	  switch (key)
	  {
	  case 'q':
			this->cur_gamestate = POSTGAME;
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
	auto navigation_node = std::make_shared<Navigation>();

	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(navigation_node);

	executor.spin();

	rclcpp::shutdown();

	return 0;
}