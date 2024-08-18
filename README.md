<h1>ROS 2 Workshop Instructions</h1>

## 1) Introduction

This workshop aims to showcase some functionalities of ROS 2, while also illustrating some of the best practices associated with it. The aim is to do that at a level that is more advanced than what is generally covered in the typical ROS tutorials found online. On the other hand, the workshop starts from the basics, so as to introduce ROS to people who have never used it before. 

## 2) Structure

This workshop is to be done like a typical "lab" at a college course, with a ROS-knowledgeable instructor guiding a small cohort of students while they complete the tasks themselves. Since ROS is a very hands-on skill, this is probably more productive for knowledge acquisition than pure lectures. With that said, an intro lecture to the general ROS philosophy can be provided if needed. 

The structure of the workshop is scheduled as follows:

- Day 0: ROS 2 lecture (if needed, depending on cohort's level of skill).
- Day 1: Programming a node with topics, services and parameters, together with RQt and CLI tools.
- Day 2: Using external libraries and packages inside a ROS node. ROS bags. Bridging.
- Day 3: Use of common external packages and how they typically integrate with ROS. Development of a ROS simulation.
- Day 4: Real life robot package study.

## 3) Prerequisites

- Intermediate level of C++ (which will be the language mostly used) and Python needed, especially in the context of OOP. Students should understand classes, inheritance, threading, pointers, etc.
- A linux machine running Ubuntu 22 and with ROS 2 Humble installed

**Note on docker**: the use of a docker could remove the necessity of an Ubuntu 22 machine. While that is true, the addition of a docker would add complexity to what is essentially a ROS **tutorial**. We might spend more time fighting docker than ROS itself in order to get things working. Furthermore, even though ROS 2 is theoretically multi-platform, please stick to linux if you plan to develop code for it...

**Note on version**: ROS 2 Humble was chosen as it is the distro used on Cerberus. This tutorial should be pretty interchangeable with other ROS 2 distros though.

## 4) Installation 

Follow the installation instructions on this [link](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) to install ROS 2 Humble on your machine. The desired package suite is the **Desktop Install**. Don´t forget to also install the ```ros-dev-tools```.

You should add the ROS sourcing to your ```.bashrc``` folder to ensure every new terminal sets up the correct ROS environment (this is called an "underlay"):

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

Also set your ROS configuration to only find nodes in your local machine. This prevents nodes from interfering with each other across multiple machines in the same network:

```bash
echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc
```

Finally, install the full suite of RQT tools:

```bash
sudo apt install '~nros-humble-rqt*'
```

If you with to test if your installation is running successful, try executing these examples in separate terminals:

```bash
ros2 run demo_nodes_cpp talker
```

```bash
ros2 run demo_nodes_py listener
```

## 5) Ownership note

Workshop author: Leonardo Mouta

Author email: [mouta@cmu.edu](mouta@cmu.edu)