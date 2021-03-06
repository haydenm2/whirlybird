## Introduction

This package contains the necessary custom ROS messages used for the whirlybird workspace. 
- Command.msg: Contains the left and right pwm commands for each rotor. Their values range from [0,1]
- WbStates.msg: Contains the state of the whirlybird
- WbReferenceStates.msg: Contains the desired yaw and pitch commands.

## Setup

If you currently do not have a catkin workspace, start by creating one.

```
mkdir -p <catkin_ws_name>/src
cd <catkin_ws_name>
catkin_make
```

Clone the wb\_msgshirlybird/wb_msgs package into the source folder of your catkin workspace, build your workspace, and add the package to the
ROS path by sourcing the *setup.bash* file.

```
cd src
git clone https://magiccvs.byu.edu/gitlab/whirlybird/wb_msgs.git
cd ..
catkin_make
source devel/setup.bash
```

or if you have keys setup you can run the following

```
cd src
git clone git@magiccvs.byu.edu/gitlab/whirlybird/wb_msgs.git
cd ..
catkin_make
source devel/setup.bash
```

## ROS Environment Variables

To communicate with the whirlybird hardware, you will need to change your ROS Environment Variables. The bash script whirlybird_lab.bash creates functions to quickly modify
the ROS Environment Variables. To run these functions, you will need to source the bash script

``` bash
source <path_to_workspace>/src/wb_msgs/whirlybird_lab.bash
```

you can also add it to your .bashrc file.

``` bash
echo "source <path_to_workspace>/src/wb_msgs/whirlybird_lab.bash" >> ~/.bashrc
```

Now, call any of the functions in your terminal. For example
```
ros_local()
```

This changes the ROS\_MASTER\_URI to localhost. 

If you want to connect to a specific whirlybird, call it's corresponding function.
































<!--# Whirlybird ROS Messages Package-->


<!--## Get Started-->

<!--Login to your CAEDM account on the whirlybird computers.-->

<!--The commands below will be run in the terminal. You can open a terminal session by pressing `CTRL + ALT + t` on your keyboard.-->

<!--### Setup ROS-->
<!--You will first need to configure ROS. This is done by running the following commands in a terminal.-->
<!--``` bash-->
<!--echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc-->
<!--source ~/.bashrc-->
<!--```-->


<!--### Setup your workspace-->

<!--0) make sure your working directory is your "home" folder-->
<!--1) Make a parent folder "wb_ws" with a child folder "src", and change directory into the /src folder.-->
<!--2) Clone wb_msgs, wb_viz, wb_st_control-->
<!--3) Change directory to the workspace folder, and catkin_make-->
<!--4) Source the catkin workspace-->
<!--5) force a ros package cache update-->

<!--``` bash-->
<!--cd ~-->
<!--mkdir -p wb_ws/src-->
<!--cd wb_ws/src-->
<!--git clone https://magiccvs.byu.edu/gitlab/whirlybird/wb_msgs.git-->
<!--git clone https://magiccvs.byu.edu/gitlab/whirlybird/wb_viz.git-->
<!--git clone https://magiccvs.byu.edu/gitlab/whirlybird/wb_st_control.git-->
<!--cd ..-->
<!--catkin_make-->
<!--source devel/setup.bash-->
<!--rospack list-->
<!--```-->

<!--Test that messages and visualization is setup correctly-->

<!--``` bash-->
<!--roslaunch visualization.launch-->
<!--```-->

<!--Uncheck the box just beneath the 'Topics' section, labeled "Enabled". You can now change the roll, pitch, and yaw sliders. If everything is working correctly, you should see the corresponding changes in the graphics frame.-->


<!--**NOTE**: Anytime you run `catkin_make` from the root folder of your catkin workspace, in our case `wb_ws`, you need to source your workspace. This is done by changing directory to your workspace root directory then sourcing devel/setup.bash-->

<!--e.g.:-->
<!--```bash-->
<!--cd ~/wb_ws-->
<!--source devel/setup.bash-->
<!--```-->


