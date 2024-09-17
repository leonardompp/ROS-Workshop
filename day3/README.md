h1>ROS 2 Workshop Instructions - Day 3</h1>

Simulations play a big part in robotics development. Although there are many simulators in the market, one in particular, called Gazebo, is the most optimized to work with ROS. Mastering simulations allows you to test ideas quickly without needing actual, physical hardware.

Furthermore, there are many packages in ROS, integratable with simulations, that let you explore important capabilities of robots, such are autonomous navigation, controls and sensor fusion. In this part of the tutorial, we are going to explore all these aspects. 

Finally, using all these packages can be hard if you decide to run each one by hand. So this tutorial will also introduce the concept of launch files. 

- [1) Introduction](#1-introduction)
- [2) Task 0: Setting up](#2-task-0-setting-up)


## 1) Introduction

For this day, you are going to work with the files inside the ```simulation_files``` package. These cointain files for interacting with the simulation. By the end of these exercises, you should have a car that can move autonomously inside a building.

## 2) Task 0: Setting up

Before we begin setting up, a word of advice is needed about naming. Gazebo's naming convention is one of the most annoying things in all of robotics. The old version, known now as ```Gazebo Classic```, used to be known simply as ```Gazebo```. But then they came out with a new version ```Ignition```, which later got renamed (and is now known) as ```Gazebo```. For ROS 2 Humble, the LTS version to be used is ```Gazebo Fortress```, which is in the ignition branch (and uses ```ign``` command line arguments). When searching online for solutions, do check the version being used in the answers you find. Most answers are for ```Gazebo Classic```, which has been around longer. In this tutorial, unless we specifically mention ```classic```, we will be talking about the new ```Gazebo/Ignition```.


![Gazebo naming](../../images/Gazebo.png "Gazebo naming")

You can start by installing the simulator. This is listed in the dependencies inside ```package.xml```, but you can also perform the installation manually.

From the [official Gazebo page](https://gazebosim.org/docs/fortress/ros_installation/)

```bash
sudo apt-get install ros-${ROS_DISTRO}-ros-gz
```

As always, you can install the required packages from ```rosdep``` (assuming a terminal at the root of your workspace).

```bash
rosdep install --from-paths src -y --ignore-src
```

---

**QUESTION**: Take a look at the packages inside ```package.xml```. TODO: Finish question

**HINT**: 

**ANSWER**: 

---

## 3) Task 1: Getting started with the simulation - launch files

As usual, colcon build your workspace and source the files. 

As mentioned through this tutorial, we will be running a simulation. You can activate Gazebo through the CLI by running:

```bash
ign gazebo
```

The more interesting way for our purposes is to run Gazebo from inside a ROS package. ```ros_gz_sim``` has an option for that:

```bash
ros2 launch ros_gz_sim gz_sim.launch.py
```

---

**QUESTION**: 

**HINT**: 

**ANSWER**: 

---