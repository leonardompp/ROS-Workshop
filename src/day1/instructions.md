<h1>ROS 2 Workshop Instructions - Day 1</h1>

This first exercise aims to introduce the very basics of ROS 2. This is done by writing a simple node with basic functions, ir order to explore some key points in ROS. We'll also analyze the files used to build and manage dependencies for the package.

## 1) Introduction

You will be working with two packages for this task: ```spaceship_interfaces``` and ```spaceship```. As the names imply, we will be making a node that simulates a spaceship! 

Each part of the tutorial consists of implementing some functionality. The skeleton of the code has been provided, as well as the files to build the node. Your job is to fill in the blanks.

There are also conceptual questions in this file. Resources on where to find the answers are provided.

## 2) Task 0: Setting up

You may have noticed when downloading the files that they came inside a folder called day 1. You might have some trouble if you just try running or building the code directly. The reason for that is because ROS expects a very particular directory structure for it to work.

---

**QUESTION**: Considering the files given, what would the structure be for you to be able to use these packages as ROS packages? 

**HINT**: You can find this answer in the lecture slides.

**ANSWER**:

ROS expects a top-level workspace which must contain (at least) an src folder. Like this:

```
top_level_ws/
|src/
```

For the packages, you can put them individually inside the ```src``` folder:

```
top_level_ws/
|src/
||spaceship_interfaces/
||spaceship/
```

Alternatively, you can also just move the whole ```day1``` folder under ```src```. ROS know how to find the packages inside by searching for the CMakeLists.txt files.

```
top_level_ws/
|src/
||day1/
|||spaceship_interfaces/
|||spaceship/
```

---

From the top-level workspace folder, you can build the packages with:

```bash
colcon build --symlink-install
```

This command will by default build all packages that are in the ```src/*``` path. You can change both the path of packages and the packages built with ```colcon``` flags.

---

**QUESTION**: What additional folders appear when you tell ROS to build packages?

**HINT**: You can find this answer in the lecture slides.

**ANSWER**: 

```build/```

```install/```

```logs/```

---

---

**QUESTION**: Why is it a good practice to use ```--symlink-install```?

**HINT**: [link](https://answers.ros.org/question/296081/use-of-symlink-install/)

**ANSWER**:

Your ```install/``` folder contains many files from the packages, such as URDF files, launch files, config files and libraries. Rather then going for the original files in the package folders, when ROS needs those files it looks inside the ```install/```. This is a ROS 2 behavior that was not really present in ROS 1.

When you build without ```--symlink-install```, ROS copies the specified files. When you change them, you need to rebuild the packages. With the flag, changes in the original will propagate, as the files inside ```install/``` are only symbolic links. 

Note that this does not influence compiled files: if you change your cpp source, you´ll still need to rebuild the package.

---

Build the packages in the workspace. You might get some warnings about unused parameters. Just ignore them for now. 

In order to access the packages built, you need to source the current environment (called the "overlay"). This will add the packages and resources in the workspace to your ROS environment:

```bash
source install/setup.bash
```

**Note**: official ROS documentation says to open a new terminal before doing this. Honestly, this is overkill in most cases... if you're getting weird results, open a new terminal. For convenience, I even add the overlay sourcing to my ```.bashrc``` file because I'm lazy.

If you want to run the package for testing, use:

```bash
ros2 run spaceship spaceship
```

You should see... absolutely nothing (including no errors!).

## 2) Task 1: Publisher and code basics

Open the file ```spaceship.cpp```. You're going to start filling its skeleton now.

As mentioned, the code simulates a spaceship! At least in abstract terms...

Your first task will be to write a publisher that broadcasts the spaceship's distance relative to the Sun. If you take a look at the private variables in the ```Spaceship``` class, you'll see the constants needed for kinematics calculation have been provided:

```cpp
double speed_to_sun_ = 17.0; // kps
double initial_distance_to_sun_ = 24'579'015'078; // km
```
Speaking of the ```Spaceship``` class, this is a good moment to explain it. At it's declaration, we see:

```cpp
class Spaceship : public rclcpp::Node
```

This means it inherits from the ```rclcpp``` (Ros Client Library - C++) ```Node``` class. Whenever you're coding a node, be it in cpp or python (```rclpy```), your node class should inherit from the general node in the client library.

**Note**: it is generally a good idea to keep your code following the OOP paradigm, rather than just calling the client library functions in a procedural manner, as some tutorials show. You'll see why soon.

Let us go back to the problem of writing the broadcaster. 

---
**QUESTION**: in the ```private``` section of the class, under the comment for "part 1", create two class attributes: 1) a shared pointer timer to control the frequency of the published messages and 2) a publisher shared pointer that will be responsible for the actual sending of the messages into the ROS network. Assume the published message will just be a string. 

**HINT**: Take a look at the [official ROS tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html). They are a generally good source of info.

**ANSWER**:

Your code should look something like this (names of variables are irrelevant):

```cpp
rclcpp::TimerBase::SharedPtr distance_publishing_timer_; // Timer to control publisher frequency
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr distance_publisher_; // Actual publishing mechanism
```
---

We need to also have a way to store the time when the program started. This is a great opportunity to talk about time syncing in ROS. 

Time syncing is a pretty common problem in robotics. By default, ROS reads the system time (also know as "wall-time"). This means that if you're running nodes on different machines, you must find a way to get all clocks synced. On the other hand, nodes running on the same machine tend to keep their clocks neatly synced. You can also have the nodes read from a simulation time: if the node parameter ```use_sim_time``` is set to true (we´ll see parameters in a sec), the node will read time from topic ```/clock``` published by the simulation rather than from the system. All nodes have this parameter and it is set to false by default. 

You can read more about time and clocks in ROS here:

- [ROS Wiki](https://wiki.ros.org/Clock#Using_Simulation_Time_from_the_.2BAC8-clock_Topic)
- [ROS 2 Design](https://design.ros2.org/articles/clock_and_time.html)

---

**QUESTION**: Create a class attribute that will store the moment the node is started.

**HINT**: Take a look at the ```now()``` method in the [rclcpp API](https://docs.ros2.org/ardent/api/rclcpp/classrclcpp_1_1_node.html#af706b231620f1c120b7ccd738ec31867) 

**ANSWER**:

```cpp
rclcpp::Time start_time_;
```
---

Now let's take a look at the class constructor:

```cpp
Spaceship() : Node("voyager_spaceship")
```

This instantiates the ```rclcpp::Node``` from which the class inherited, giving the ROS node the name "voyager_spaceship. It is inside the constructor that you will initialize the class attributes declared previously.

---

**QUESTION**: Initialize the attributes declared in the previous questions, under the comment for task 1. Constraints:

- Name the topic where the messages will be published as ```distance```, with a queue size of 10
- Have the publisher publish at a frequency of 1Hz
- Use function ```distance_publishing_callback``` as the publisher callback

**HINT**: Adapt the ROS 2 tutorial for creating a node (previously linked here) to suit your needs.

**ANSWER**:

```cpp
this->distance_publisher_ = this->create_publisher<std_msgs::msg::String>("distance", 10); // Announce the topic that will be published to

this->distance_publishing_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&Spaceship::distance_publishing_callback, this)); // Set a callback timer for a frequency of 1 Hz

this->start_time_ = this->now(); // Save starting time
```
---



## Appendix

Conventions - expecting something from another team
Compositions
Parameters
Difference between services and topics
Theoretical questions
ROS 2 interfaces are not typically in the same path
Why placeholders?
Talk about callbacks and executors - thread efficiency
RQT tools

---

**QUESTION**: 

**HINT**: 

**ANSWER**:

---