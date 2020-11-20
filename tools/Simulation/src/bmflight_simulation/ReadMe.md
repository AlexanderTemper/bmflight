#Start Sim

roslaunch bmflight_simulation test.launch


rosrun bmflight_simulation sitl_interface_node bmflight
rosrun bmflight_simulation gateway_node bmflight


rosrun joy joy_node
