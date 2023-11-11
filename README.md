# ffi3_intp_ROS2Worm

## Navigation Node

## Display Node

## Grid Node

publishes to the following topics:

- GameStart  -- using Message Type: std_msgs::msg::Int32  
  - used to transmit a gameId to identify a starting game
- BoardInfo  -- using Message Type: ros2_worm_multiplayer::msg::Board  
  - used to transmit all of the contents of the game board in the Board datastructure

subscribes to the following topics:

- PlayerInput  -- using Message Type: ros2_worm_multiplayer::msg::Direction  
