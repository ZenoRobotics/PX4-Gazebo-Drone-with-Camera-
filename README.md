## Running Scripts with Gazebo, PX4, Mavros and QGroundControl (SITL)

Setup Directions:

Download or clonee code here. Then download and build the PX4 software which can be foud here with instructions: https://docs.px4.io/master/en/dev_setup/building_px4.html . The directions below for running the drone simulation below assumes that you place the PX4 software under the /PX4_Drone_Sim_Proj director. This isn't necessary, you just have to modify the T2 directions below to cd into your PX4-Autopilot directory location.

Assuming all of the software tools listed above have been properly installed, start them up in the following order: 

1) Build Mavros/Mavlink ROS directory

```
cd ~/PX4_Drone_Sim_Proj/catkin_drone_test_ws
catkin build
```

1) Start mavros in a terminal (T1) 

```
cd ~/PX4_Drone_Sim_Proj/catkin_drone_test_ws
source ../px4_lat_long_set.bash
source devel/setup.bash
roslaunch mavros px4.launch fcu_url:="udp://:14540@192.168.1.36:14557"
```

2) In another terminal (T2) start up Gazebo using PX4 connection

```
cd ~/PX4_Drone_Sim_Proj/PX4-Autopilot$  
source ~/PX4_Drone_Sim_Proj/px4_lat_long_set.bash
make px4_sitl gazebo_typhoon_h480
```

3) In yet another terminal (T3) run the QGC program

```
cd ~/PX4_Drone_Sim_Proj
./QGroundControl.AppImage
```

4) In a fourth terminal (T4), run either the waypoint mission script

```
cd ~/PX4_Drone_Sim_Proj/catkin_drone_test_ws
source devel/setup.bash
cd ./src/scripts
./wp_mission_Drone.py
```
