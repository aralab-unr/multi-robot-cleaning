# Multi-Robotic System for cleaning

## Work
- This work was done in the [Advance Robotics and Autonmation (ARA) Lab](https://ara.cse.unr.edu/) 

## Package used:
- urg_node
- amcl
- map_sever
- gmapping
- [create_autonomy](https://github.com/AutonomyLab/create_autonomy)
- [libcreate](https://github.com/AutonomyLab/libcreate) **Not required but it's good to test out ticks and wheel radius to improve your odometry**
- serial

## Sofware used:
- Ubuntu 16.04 
- ROS Kinetic

## Technology used:
- iRobot Create 2
- Hokuyo laser
- IMU (BN0055)

![alt text](https://github.com/aralab-unr/multi-robot-cleaning/blob/master/Create_Autonomy%20Modification/68407733_889934048043712_2616492075268440064_n.jpg)     

# Package explanation:

## iRobot's Create Autonomy Modifications:

In the modifications folder, we modified the create_autonomy:
- Added subscriber: vacuum motor
- Added publisher: dirt level
- Merged the imu data from the imu_ros package with the odometry of the iRobot

## Mapping:

- We used a very standard method called gmapping utilizing the hokuyo laser data, the iRobot's odometry that was infused with imu data and the teleop_twist_keyboard package.
- Save the map with map_saver and send map with map_sever

## Dirt Map:

To create the dirt map, use the teleop_twist_keyboard, vacuum motor, and dirt level to get the datas. For more information: J.   Hess,   M.   Beinhofer,   D.   Kuhner,   P.   Ruchti,   and   W.   Burgard,“Poisson-driven  dirt  maps  for  efficient  robot  cleaning,”IEEE Inter-national Conf. on Robotics and Automation, pp. 2245–2250, 2013.

## Multi-Robot:

We used NUC intel and put them on each iRobot

### MultiMachine Networking:

Make sure the router is on and that both laptop/machine is connected to it. Connect like wifi. 

Also make sure to install openssh-service on all machines. (sudo apt-get install openssh-service)

Find the IP Address of the 2 computer/machine. If laptop hit the wifi bar and hit connection information. IP Address will be under IPv4.

On the master device open the .bashrc file:

gedit ~/.bashrc

To disable husarnet entries, find lines:

export ROS_MASTER_URI=http://master:11311
export ROS_IPV6=on

and comment them with #:

#export ROS_MASTER_URI=http://master:11311
#export ROS_IPV6=on

Then add two lines at file ending, replacing X.X.X.X and Y.Y.Y.Y with IP address of master device.

export ROS_MASTER_URI=http://X.X.X.X:11311
export ROS_IP=Y.Y.Y.Y

On other machines also open the .bashrc file, comment Husarnet entries and add two lines at file ending. This time replace X.X.X.X with IP address of master device and Y.Y.Y.Y with IP address of second robot.

Remember that roscore must be running on the device indicated as ROS master!!!

On the computer you chose to be the master, command:
- ssh <computer_name>@<IP Address>	ex: ara@192.168.1.86
- It will require you to enter the password of the other computer
- Then you will have complete control over that machine's terminal using just the master machine
	

### Multi Localization:

- On the master machine, run send_map.launch to publish the map.
- Place the robot_0 and robot_1 in each NUC
- On each NUC, run the group.launch in each package.

### Goal:

- Run publish_goal goal to publish the goals for each robot

### Send:

- Each robot should have a send node that subscribe to /goal or /goal1 and send the iRobot to the goals using cmd_cel topic while also changing the vacuum motor's power depending on the dirt.

## Video:
https://www.youtube.com/watch?v=sqrj5DN1SGQ
	
## Reference:
	C. P. Le, A. Q. Pham, H. M. La, D. Feil-Seifer.  A Multi-Robotic System for Environmental Dirt Cleaning. The 12th IEEE/SICE International Symposium on System Integration (SII), January 12-15, 2020, Hawaii, USA. [PDF](https://ara.cse.unr.edu/wp-content/uploads/2014/12/SII2020_ChuongLe_final.pdf) 

## Contact:
	[Hung La](hla@unr.edu)

