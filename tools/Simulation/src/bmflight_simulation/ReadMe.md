#Start Sim


## launch sitl
../../bin/SITL.elf

## launch gateway tool
./Gateway/bin/gateway.out tcp 127.0.0.1 -simBridge

## launch world and mav
roslaunch bmflight_simulation test.launch

## launch joy node
rosrun joy joy_node

## launch gateway node
rosrun bmflight_simulation gateway_node bmflight

## launch interface
(sometimes buggy relaunch untill get motor data is gisplayed) //todo fix this
rosrun bmflight_simulation sitl_interface_node bmflight
