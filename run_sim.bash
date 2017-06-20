xterm -title "MAVLink" -hold -e "source ~/simulation/ros_catkin_ws/devel/setup.bash; cd ~/simulation/ardupilot/ArduCopter; ../Tools/autotest/sim_vehicle.sh -j 4 -f Gazebo" &
xterm -title "ErleCopter" -hold -e "source ~/simulation/ros_catkin_ws/devel/setup.bash; roslaunch ardupilot_sitl_gazebo_plugin erlecopter_spawn.launch" &
