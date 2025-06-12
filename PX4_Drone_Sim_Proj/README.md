## Running Scripts with Gazebo, PX4, Mavros and QGroundControl (SITL)

### Setup Directions:

Download or clone code here. Then download and build the PX4 software, as well as mavros and mavlink, which can be found here with instructions: [https://docs.px4.io/master/en/dev_setup/building_px4.html](https://docs.px4.io/main/en/ros/mavros_installation.html) . 

The directions below for running the drone simulation below assumes that you place the PX4 software under the /PX4_Drone_Sim_Proj director. This isn't necessary, you just have to modify the T2 directions below to cd into your PX4-Autopilot directory location.

Assuming all of the software tools listed above have been properly installed, start them up in the following order:

Build Mavros/Mavlink ROS directory

    cd ~/PX4_Drone_Sim_Proj/catkin_drone_test_ws
    catkin build

Start mavros in a terminal (T1)

    cd ~/PX4_Drone_Sim_Proj/catkin_drone_test_ws
    source ../px4_lat_long_set.bash
    source devel/setup.bash
    roslaunch mavros px4.launch fcu_url:="udp://:14540@192.168.1.36:14557"

In another terminal (T2) start up Gazebo using PX4 connection

    cd ~/PX4_Drone_Sim_Proj/PX4-Autopilot$ 
    source ~/PX4_Drone_Sim_Proj/px4_lat_long_set.bash
    make px4_sitl gazebo_typhoon_h480

In yet another terminal (T3) run the QGC program

    cd ~/PX4_Drone_Sim_Proj
    ./QGroundControl.AppImage

In a fourth terminal (T4), run either the waypoint mission script

    cd ~/PX4_Drone_Sim_Proj/catkin_drone_test_ws
    source devel/setup.bash
    cd ./src/scripts
    ./wp_mission_Drone.py


## Notes:

### QGroundControl

Ubuntu 20.04 uses an older version of 'GLIBC_'. If you download and use the newest version of QGroundControl, you will most likely get the following error when tryin to run QGC:

    /tmp/.mount_QGrounFGCmaK/usr/bin/QGroundControl: /lib/x86_64-linux-gnu/libc.so.6: 
    version `GLIBC_2.33' not found (required by /tmp/.mount_QGrounFGCmaK/usr/lib/libFLAC.so.8)

The easiest way to get around this is to download an older version (e.g., v4.3.0) of QGC from:

https://github.com/mavlink/qgroundcontrol/releases/tag/v4.3.0

### PX4-Gazebo

When running make under PX4-Autopilot/ (e.g., make px4_sitl gazebo_typhoon_h480) you may get the following error:

    ... fatal error: opencv2/opencv.hpp: No such file or directory
    12 | #include <opencv2/opencv.hpp>
                 |^~~~~~~~~~~~~~~~~~~~
    compilation terminated.

A simple fix to this is to create a link for opencv2 as follows:

    sudo ln -s /usr/include/opencv4/opencv2 /usr/include/opencv2


    
