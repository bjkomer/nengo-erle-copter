# nengo-erle-copter
Connecting Nengo to the Erle-Copter

# Setup

Follow the steps [here](docs.erlerobotics.com/simulation/configuring_your_environment) to configure your environment to be able to use both the real quadcopter and the gazebo simulation.

# Start the Gazebo simulation

In one terminal run:

```
source ~/simulation/ros_catkin_ws/devel/setup.bash
cd ~/simulation/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.sh -j 4 -f Gazebo
```

In another terminal run:

```
source ~/simulation/ros_catkin_ws/devel/setup.bash
roslaunch ardupilot_sitl_gazebo_plugin erlecopter_spawn.launch
```

# Configuring ROS

To use the simulator:

```
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=127.0.0.1
```

To use the real quadcopter:

```
export ROS_MASTER_URI=http://10.0.0.1:11311
export ROS_IP=10.0.0.2
```

You also need to add `10.0.0.1 erle-brain-3` to `/etc/hosts`

## Using ROS

To arm and disarm motors:
```
rosservice call /mavros/cmd/arming True
rosservice call /mavros/cmd/arming False
```

To change the flight mode:
```
rosservice call /mavros/set_mode 0 ALT_HOLD
rosservice call /mavros/set_mode 0 GUIDED
etc...
```

To takeoff from the current location
```
rosrun mavros mavcmd takeoffcur 0 0 2
```

When overriding RC commands, you first need to run this command:
```
rosrun mavros mavparam set SYSID_MYGCS 1
```

You can then publish commands like this:
```
rostopic pub -1 /mavros/rc/override mavros_msgs/OverrideRCIn '[1500, 1500, 1500, 65535, 65535, 65535, 65535, 65535]'
```

Channel pwm values range between 1000 and 2000. Publishing 0 on a channel gives control back to the RC controller. Publishing 65535 on a channel ignores that channel (uses previous value).

# Mavlink

simulator mavlink address: `127.0.0.1:14550`
quadcopter mavlink address: `10.0.0.2:6000`

Run mavproxy like this: `mavproxy.py --master 10.0.0.2:6000`

# Misc

The four modes that do not require GPS lock to operate:

* STABILIZE
* ALT_HOLD
* ACRO
* LAND
