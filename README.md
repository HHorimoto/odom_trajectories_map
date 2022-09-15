# odom_trajectories_map

**This package makes a map by odometry's trajectories**

## Requirement
+ Ubuntu 18.04
+ ROS (Melodic)
+ Python 2.7.x

## Set Up
Download `odom_trajectories_map` package.

```shell
$ cd ~/catkin_ws/src/
$ git clone https://github.com/HHorimoto/odom_trajectories_map.git
$ cd ~/catkin_ws
$ catkin_make
```

## How To Use

1. Launch this launch file.

```shell
$ roslaunch odom_trajectories_map odom_trajectories_map.launch
```

### Parameters

+ ***odom_topics*** : odometry topic names. **I belive you have to change the default names**.
    default : `[/odom_1, /odom_2]`

+ ***points_size*** : the size of points on map.
    default : `5`

2. Play your rosbag

```shell
$ rosbag play path/to/your/rosbag.bag
```