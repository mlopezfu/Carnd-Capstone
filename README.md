
This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. 

I have developed the system with the native installation. **One** of the two installation options, either native **or** docker installation.

You can look for more info on the original link https://github.com/udacity/CarND-Capstone

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).


### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator


# System Architecture
The following is a system architecture diagram showing the ROS nodes and topics used in the project.
![architecture](https://d17h27t6h515a5.cloudfront.net/topher/2017/September/59b6d115_final-project-ros-graph-v2/final-project-ros-graph-v2.png)

## Waypoint Updater
The purpose of waypoint updater is to update the target velocity property of each waypoint based on traffic light and obstacle detection data. The target veloicty at normal situdation is given from `waypoint_loader` node. If the red light is detected, we genetated stopping trajectory considering vehicle's deceleration limits. 

## Waypoint Follower
The longitudinal target velocity was set in `waypoint_updater` node. This node determine the target yawrate to keep the lane by using pure-pursuit algorithm.

## Traffic Light Detection
I have tried some classification with Neural Networks, but when I tried to run the simulator on them, then everything begin to slow down and the car goes off road.

I have developed this on my computer so I have to look for a quickest way to detect the red light.

Since we know the locations of the traffic lights and the car pose, it is posible to reduce the number of times you look for traffic light.
When its needed to look for it, its quick enough to make a color search.
The most important color is red, wich has two separated ranges in HSV space.
![Sufficient](./img/hsv2rgb.png)

https://en.wikipedia.org/wiki/HSL_and_HSV#HSV_to_RGB

Once the ranges are defined, a mask can be calculated, obtaining the following results:
![Sufficient](./img/tl_big_color.png)
![Sufficient](./img/tl_big_binary.png)

Where the numbers are the total of binary points in the mask, and the points for lower and upper ranges.

Even when the traffic light is far enough, the red light can be detected.

![Sufficient](./img/tl_small_color.png)
![Sufficient](./img/tl_small_binary.png)


## Final lap

After that, the autonomous driving is achieved with good perfomance on the development computer.

 <a href="http://www.youtube.com/watch?feature=player_embedded&v=oahyKMsqtmY
" target="_blank"><img src="http://img.youtube.com/vi/oahyKMsqtmY/0.jpg" 
alt="path planning" width="600" border="10" /></a>

 
 Quicktime recording of the desktop introduces charge on the CPU so more difficult for the computer to have the virtual machine, simulator and quicktime running at the same time whitout affect the car trajectory.


