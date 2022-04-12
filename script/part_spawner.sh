#!/bin/bash

# spawn a part in the tray on agv1
sleep 5
rosservice call /ariac/start_competition
sleep 10
rosrun gazebo_ros spawn_model -sdf -x 0.1 -y 0.1 -z 0.05 -R 0 -P 0 -Y 1.57079632679 -file `rospack find nist_gear`/models/assembly_battery_blue_ariac/model.sdf -reference_frame agv1::kit_tray_1::kit_tray_1::tray -model assembly_battery_blue_5

rosrun gazebo_ros spawn_model -sdf -x -0.1 -y 0.1 -z 0.05 -R 0 -P 0 -Y 1.0471975512 -file `rospack find nist_gear`/models/assembly_sensor_red_ariac/model.sdf -reference_frame agv1::kit_tray_1::kit_tray_1::tray -model assembly_sensor_red_5

rosrun gazebo_ros spawn_model -sdf -x 0.1 -y -0.1 -z 0.05 -R 0 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_battery_blue_ariac/model.sdf -reference_frame agv1::kit_tray_1::kit_tray_1::tray -model assembly_battery_blue_6
