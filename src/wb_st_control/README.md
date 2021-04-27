## Introduction

This is the student control package for the whirlybird and is designed to contain different controllers that students design during the semester. A template controller node called controller.py 
is provided to help students create their own controllers.
This package also contains the necessary parameters for the whirlybird. (Except for the km value! You will udate this value in a later lab.)

The controller node subscribes to the reference signal and whirlybird states topics. It publishes to the whirlybird command topic

## Dependencies
- [wb_msgs](https://magiccvs.byu.edu/gitlab/whirlybird/wb_msgs) package: Contains all of the custom whirlybird messages to be used by other packages.
- [wb_viz](https://magiccvs.byu.edu/gitlab/whirlybird/wb_viz) package: Used to visualize the whirlybird, and contains sliders corresponding to every whirlybird message.
- [wb_st_dynamics](https://magiccvs.byu.edu/gitlab/whirlybird/wb_st_dynamics) package: Contains the dynamical whirlybird model and is used for simulation.

## Setup

If you currently do not have a catkin workspace, start by creating one.

```
mkdir -p <catkin_ws_name>/src
cd <catkin_ws_name>
catkin_make
```

Clone the [wb_msgs](https://magiccvs.byu.edu/gitlab/whirlybird/wb_msgs), the [wb\_viz](https://magiccvs.byu.edu/gitlab/whirlybird/wb_viz),  [wb_st_dynamics](https://magiccvs.byu.edu/gitlab/whirlybird/wb_st_dynamics) 
and the wb\_st\_control packages in the source folder of your catkin workspace, build your workspace, and add your packages to the
ROS path by sourcing the *setup.bash* file.

```
cd src
git clone https://magiccvs.byu.edu/gitlab/whirlybird/wb_msgs.git
git clone https://magiccvs.byu.edu/gitlab/whirlybird/wb_viz.git
git clone https://magiccvs.byu.edu/gitlab/whirlybird/wb_st_control.git
git clone https://magiccvs.byu.edu/gitlab/whirlybird/wb_st_dynamics.git
cd ..
catkin_make
source devel/setup.bash
```

Remember: In order for ROS to run python nodes, they need to have at least user executable permissions. You can add these permissions by the chmod command.
```
chmod +x <python_node.py>
```


## Running the Controller

The controller has two different ROS launch files depending on simulation or hardware. 
- hw.launch: Runs the visualization and controller nodes.
- sim.launch: Runs the visualization, dynamics, and controller nodes.

You will need to edit the launch files to specify the controller node that you wish to run. For example the contents sim.launch files looks like
``` xml
<launch>

  <include file="$(find wb_st_dynamics)/launch/dynamics_viz.launch"/> 

  <node name="wb_st_control" pkg="wb_st_control" type="controller.py" />
  <!-- <node name="wb_st_control" pkg="wb_st_control" type="pid_controller" /> -->

</launch>
```

You will need to change the controller.py parameter to reflect the name of the controller node you wish to run. Now that everything is setup properly, run 
your nodes using the roslaunch command
```
roslaunch wb_ta_control sim.launch
```

Note: the contoller.py file provided is not complete and is only provided to be used as a template to create your own controllers.

The other controller shown, *pid\_controller*, in the launch file is a compiled controller that you can use to test your dynamics. To run this controller, comment out the other controller
and uncomment this one so that the launch file looks like 
``` xml
<launch>

  <include file="$(find wb_st_dynamics)/launch/dynamics_viz.launch"/> 

  <!-- <node name="wb_st_control" pkg="wb_st_control" type="controller.py" /> -->
  <node name="wb_st_control" pkg="wb_st_control" type="pid_controller" />

</launch>
```

Now run the nodes using the roslaunch command
```
roslaunch wb_ta_control sim.launch
```



