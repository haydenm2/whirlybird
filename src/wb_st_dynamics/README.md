## Introduction

This is the student dynamic package for the whirlybird and contains the python node that simulates the whirlybird dynamics. This node is incomplete and requires the
students to implement the whirlybird dynamics.


## Dependencies
- [wb_msgs](https://magiccvs.byu.edu/gitlab/whirlybird/wb_msgs) package: Contains all of the custom whirlybird messages to be used by other packages.
- [wb_viz](https://magiccvs.byu.edu/gitlab/whirlybird/wb_viz) package: Used to visualize the whirlybird, and contains sliders corresponding to every whirlybird message.
- [wb_st_control](https://magiccvs.byu.edu/gitlab/whirlybird/wb_st_control) package: Contains different controllers and the whirlybird parameters.

## Setup

If you currently do not have a catkin workspace, start by creating one.

```
mkdir -p <catkin_ws_name>/src
cd <catkin_ws_name>
catkin_make
```

Clone the [wb_msgs](https://magiccvs.byu.edu/gitlab/whirlybird/wb_msgs), the [wb\_viz](https://magiccvs.byu.edu/gitlab/whirlybird/wb_viz),  [wb_st_control](https://magiccvs.byu.edu/gitlab/whirlybird/wb_st_control) 
and the wb\_st\_dynamics packages in the source folder of your catkin workspace, build your workspace, and add your packages to the
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

The dynamics project has one ROS launch file. 
- dynamics_viz.launch

This launch file will run the visualization node and the dynamics node. To run it


```
roslaunch wb_st_dynamics dynamics_viz.launch
```



