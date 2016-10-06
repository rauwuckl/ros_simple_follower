# ros_simple_follower
A very simple implementation for ROS to make a mobile robot follow a target. Either using a Laser Range Finder to follow the closest object or an RGB-D camera to follow a specific colour. It was developed for a Robotnik 'Summit XL' robot but should work anywhere. (You might have to change the topics in the python files though)

## Usage
'roslaunch simple_follower laser_follower.launch' or 'roslaunch simple_follower visual_follower.launch' respectively. 

## Architecture
There are three nodes. They can be launched using the .launch files in simple_follower/launch/nodes
- follower.launch
  - follow object which position is specified in the \'/object_tracker/current_position\' topic
- Each of these trackers publishes to that topic and can be used interchangably
  - visualTracker.launch (follows the biggest object with the colour specified in the launch file)
  - laserTracker.launch (follows the closest point in the laser range data)
  
