<h1>ROS 2 Workshop Instructions - Day 1</h1>

This first exercise aims to introduce the very basics of ROS 2. This is done by writing a simple node with basic functions in order to explore some key points in ROS.

- [1) Introduction](#1-introduction)
- [2) Task 0: Setting up](#2-task-0-setting-up)
- [2) Task 1: Publisher and code basics](#2-task-1-publisher-and-code-basics)
- [3) CLI Tools](#3-cli-tools)
- [4) Task 2: Subscriber](#4-task-2-subscriber)
- [5) RQt Tools](#5-rqt-tools)
- [6) Task 3: ROS Parameters](#6-task-3-ros-parameters)
- [7) Task 4: ROS Services](#7-task-4-ros-services)
- [8) Advanced concepts and standards](#8-advanced-concepts-and-standards)

## 1) Introduction

You will be working with two packages for this task: ```spaceship_interfaces``` and ```spaceship```. As the names imply, we will be making a node that simulates a spaceship! 

Each part of the tutorial consists of implementing some sort of functionality. The skeleton code has been provided, as well as the files to build the node. Your job is to fill in the blanks.

There are also conceptual questions in this file. Resources on where to find the answers are provided.

## 2) Task 0: Setting up

You may have noticed when downloading the files that they came inside a folder called ```day1```. You **might** have some trouble if you just try running or building the code wherever. The reason for that is because ROS expects a very particular directory structure for in order to work.

---

**QUESTION**: Considering the files given, what would the structure be for you to be able to use the code as ROS packages? 

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

This means that you can use the repository as is, treating its root as the top-level workspace!

---

Using a terminal at the top-level workspace folder, you can build the packages with:

```bash
colcon build --symlink-install
```

This command will by default build all packages that are in the ```src/*``` path. You can change both the path of packages and the packages built by using ```colcon``` flags.

---

**QUESTION**: What additional folders appear when you tell ROS to build packages?

**HINT**: You can find this answer in the lecture slides.

**ANSWER**: 

```build/```

```install/```

```log/```

---

---

**QUESTION**: Why is it a good practice to use ```--symlink-install```?

**HINT**: [link](https://answers.ros.org/question/296081/use-of-symlink-install/)

**ANSWER**:

Your ```install/``` folder contains many files from the packages, such as URDF files, launch files, config files and libraries. Rather than searching for the original files in the package folders, when ROS needs those files it looks inside the ```install/``` folder. This is a ROS 2 behavior that was not really present in ROS 1.

When you build without ```--symlink-install```, ROS copies these files. When you change the originals, you need to rebuild the packages. With the flag, changes in the original will propagate, as the files inside ```install/``` are only symbolic links to the originals. 

Note that this does not influence compiled files: if you change your cpp source, you'll still need to rebuild the package.

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

Open the file ```spaceship.cpp``` on the ```spaceship``` package. You're going to start filling its skeleton now.

As mentioned, the code simulates a spaceship! At least in abstract terms...

Your first task will be to write a publisher that broadcasts the spaceship's distance relative to the Sun. If you take a look at the private variables in the ```Spaceship``` class, you'll see the constants needed for kinematics calculation have been provided:

```cpp
double speed_to_sun_ = 17.0; // kps; collinear; away

double initial_distance_to_sun_ = 24'579'015'078; // km
```
Speaking of the ```Spaceship``` class, this is a good moment to explain it. At it's declaration, we see:

```cpp
class Spaceship : public rclcpp::Node
```

This means it inherits from the ```rclcpp``` (Ros Client Library - C++) ```Node``` class. Whenever you're coding a node, be it in C++ or Python (```rclpy```), your node class should inherit from the general node in the client library.

**Note**: it is generally a good idea to follow the OOP paradigm in your code, rather than just calling the client library functions in a procedural manner, as some tutorials show.

Let us go back to the problem of writing the broadcaster. 

---
**QUESTION**: in the ```private``` section of the class, under the comment for "task 1", create two class attributes: 1) a shared pointer timer to control the frequency of the published messages and 2) a publisher shared pointer that will be responsible for the actual sending of the messages into the ROS network. Assume the published message will be a string. 

**HINT**: Take a look at this [official ROS tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html). These tutorials are a good source of info.

**ANSWER**:

(Names of variables are irrelevant, as long as you remember them):

```cpp
rclcpp::TimerBase::SharedPtr distance_publishing_timer_; // Timer to control publisher frequency

rclcpp::Publisher<std_msgs::msg::String>::SharedPtr distance_publisher_; // Actual publishing mechanism
```
---

We need to also have a way to store the time when the program started. This is a great opportunity to talk about time syncing in ROS. 

Time syncing is a pretty common problem in robotics. By default, ROS reads the system time (also known as "wall-time"). This means that if you're running nodes on different machines, you must find a way to get all clocks synced. On the other hand, nodes running on the same machine usually keep their clocks neatly synced. You can also have the nodes read from a simulation time: if the node parameter ```use_sim_time``` is set to true (we´ll see parameters in a sec), the node will read time from topic ```/clock``` published by the simulation, rather than from the system. All nodes have this parameter and it is set to ```false``` by default. 

You can read more about time and clocks in ROS here:

- [ROS Wiki](https://wiki.ros.org/Clock#Using_Simulation_Time_from_the_.2BAC8-clock_Topic)
- [ROS 2 Design](https://design.ros2.org/articles/clock_and_time.html)

---

**QUESTION**: Create a class attribute that will store the moment the node is started.

**HINT**: Take a look at the ```now()``` method in the [rclcpp API](https://docs.ros2.org/ardent/api/rclcpp/classrclcpp_1_1_node.html#af706b231620f1c120b7ccd738ec31867). 

**ANSWER**:

```cpp
rclcpp::Time start_time_;
```
---

Now take a look at the class constructor:

```cpp
Spaceship() : Node("voyager_spaceship")
```

This instantiates the ```rclcpp::Node``` from which the class inherited, giving the ROS node the name "voyager_spaceship". It is inside the constructor that you will initialize the class attributes declared previously.

---

**QUESTION**: Initialize the attributes declared in the previous questions, under the comment for task 1. Constraints:

- Name the topic where the messages will be published as ```distance```, with a queue size of 10
- Have the publisher publish at a frequency of 1Hz
- Use function ```distance_publishing_callback``` as the publisher callback
- Remember to save the starting time to the attribute

**HINT**: Adapt the [ROS 2 tutorial for creating a node](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html) to suit your needs.

**ANSWER**:

```cpp
this->distance_publisher_ = this->create_publisher<std_msgs::msg::String>("distance", 10); // Announce the topic that will be published to

this->distance_publishing_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&Spaceship::distance_publishing_callback, this)); // Set a callback timer for a frequency of 1 Hz

this->start_time_ = this->now(); // Save starting time
```
---

Now it is time to build the callback. As mentioned, the function ```distance_publishing_callback``` will be responsible for publishing the distance message. We will build it by parts.

---

**QUESTION**: Inside the function body, declare the message that will be published by the node. 

**HINT**: You an use ```auto``` keyword for variable declaration. Consult the code already written to see what message type object is being expected.

**ANSWER**:
```cpp
auto message = std_msgs::msg::String(); // Message declaration
```
---

We need to do math to calculate the current position of the spaceship relative to the Sun. For that, we need to find the time passed since the start of the program.

---
**QUESTION**: Get the time, in seconds, passed since the node started.

**HINT**: Consult the operators available for the ```rclcpp::Time``` class [here](https://docs.ros2.org/ardent/api/rclcpp/classrclcpp_1_1_time.html). Notice there is another class involved.

**ANSWER**:

```cpp
rclcpp::Duration time_duration = this->now() - this->start_time_; // Subtraction of two Time classes

double time_duration_seconds = (double) (time_duration.nanoseconds()/1e9); // Conversion to seconds in double type
```
---

Now use the constants provided with the time you just calculated to find the distance to the Sun.

---

**QUESTION**: Write the math expression that yields the distance from Sun to spaceship.

**HINT**: Just assume the simplest case possible, with the velocity pointed entirely away from the Sun.

**ANSWER**:

```cpp
double current_distance = this->initial_distance_to_sun_ + this->speed_to_sun_*time_duration_seconds; // Math exp
```

---

Finally, publish the message to the topic!

---

**QUESTION**: Create a string that contains both the distance from the Sun and the time elapsed, in seconds, since the program start. Then publish the message to the topic. 

**HINT**: Take a look at the [tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)
 to get the right idea.

**ANSWER**:

```cpp
message.data = "Distance to Sun is " + std::to_string(current_distance) + " km at time " + std::to_string(time_duration_seconds) + " s"; // Message generation

this->distance_publisher_->publish(message); // Message publication
```
---

## 3) CLI Tools

Compile the code using ```colcon``` as shown before (ignore the warnings relating to other parts of the code) and resource the overlay workspace. Then run the ```spaceship``` node in the ```spaceship``` package. ```ros2 run spaceship spaceship``` is the command to run from a terminal that has the overlay sourced.

You should see... nothing again!

The reason for that is because your node is publishing messages inside a topic, not printing them to the terminal. But using the CLI (command line interface) provided by ROS, you can see the messages regardless.

Keep the ```spaceship``` node running and open a new terminal. Type ```ros2``` and use the "tab" key to autocomplete. A number of suggestions should have popped up.

---

**QUESTION**: Of the suggestions that popped up, which command do you think is useful to get information on the nodes running? What about the topics?

**HINT**: The names of the commands are pretty self explanatory...

**ANSWER**:

- ```ros2 node``` for node info.
- ```ros2 topic``` for topic info.

---

The CLI is a fantastic tool for understanding your ROS environment, and you can read more about it [here](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html).

For example, say you want a list of nodes running. You can do:

```bash
ros2 node list
```

And see the following result in the terminal:

```
/voyager_spaceship
```
---

**QUESTION**: What would be the command for printing running topics? What are the topics currently running?

**HINT**: It is analogous to the command for nodes...

**ANSWER**:

```bash
ros2 topic list
```
With topics:

```
/distance
/parameter_events
/rosout
```
---

We want to see what messages are being produced on a topic. 


---

**QUESTION**: Type ```ros2 topic -h``` to get a list of available commands for topic. Which one do you think is useful for echoing messages being published?

**HINT**: Look at the names again...

**ANSWER**:

```bash
ros2 topic echo
```

---

If we run ```echo``` followed by a topic name, we get the messages written:

```bash
ros2 topic echo /distance
> data: Distance to Sun is 24579041054.228409 km at time 1528.013436 s
```

Notice the time in the messages: even though we specified the topic to be published at 1Hz, the interval between two successive messages is not exactly 1 sec (but close!). Keep in mind when programming. 

## 4) Task 2: Subscriber

We already wrote a publisher. Now assume the spaceship is also supposed so subscribe to a topic. We want this topic to listen for velocities. More specifically, 6-axis velocities. A 6 velocity command is usually called a "Twist".

---

**QUESTION**: Using the CLI tools, search for ROS interfaces involving twists. List the msg interfaces that implement twists.  

**HINT**: ```ros2 interface list | grep "Twist"```

**ANSWER**:

```
geometry_msgs/msg/Twist
geometry_msgs/msg/TwistStamped
geometry_msgs/msg/TwistWithCovariance
geometry_msgs/msg/TwistWithCovarianceStamped
```
---

Say we don´t care about covariance for now. We also don´t care about stamping the message, which would yield information about the frame id and message time (we´ll talk about frames in a future day). Therefore, we will use just the plain ```geometry_msgs/msg/Twist``` message.

---

**QUESTION**: In a similar manner to what you did for the publisher, write a private class attribute that will subscribe to messages of type ```geometry_msgs/msg/Twist```. 

**HINT**: If stuck, take a look here [here](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html) for inspiration.

**ANSWER**:

```cpp
rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscriber_; // Subscriber mechanism

```

---

You might have noticed we did not declare a timer. The reason for that is because subscribers don´t operate in a constant manner. Instead, an action will only be performed when a message is received by the node on the subscriber's topic.

Following the same path that was done for the publisher, you now have to start the subscriber, binding it to a function. That is done in the node constructor. 


---

**QUESTION**: Initialize the subscriber in the class constructor. Constraints:

- Name the topic where the messages will be received as ```cmd_vel```, with a queue size of 10
- Use function ```velocity_subscription_callback``` as the subscriber callback

**HINT**: Adapt the ROS 2 [tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html) to suit your needs.

**ANSWER**:

```cpp
this->velocity_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&Spaceship::velocity_subscription_callback, this, std::placeholders::_1)); // Announce the topic that will listen to velocities + callback
```
---

This is a good moment to talk about two things in the code above:

1) The choice for ```cmd_vel``` as a topic name was not arbitrary. It is very common for velocity twists to be sent across topics with that name, following an original standard set by the turtlebot robots. You can find this info in [REP 119](https://ros.org/reps/rep-0119.html). [REPs - ROS Enhancement Proposals](https://ros.org/reps/rep-0000.html) are a set of ROS guidelines established by the developer community to standardize symbols and practices. Most professional packages, such as ROS 2 Control or Nav2, follow the REP standards.

2) Notice the argument ```std::placeholders::_1```, which was not present when binding the publisher. Take a look at the signature for the function ```velocity_subscription_callback()```. You can see it takes an argument. Since the argument will depend on the message being received, the binding takes a placeholder as an argument to keep that space "reserved" for the upcoming message.


Now is is time to write the body of the callback.

---

**QUESTION**: Write the body of the ```velocity_subscription_callback```. This will be simple: just write all 6 components of the twist to the console with 2 decimal places. Don't use ```printf``` function: logging in ROS is done in another way.

**HINT**: Check out the [tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html) for writing a node and the [RCLCPP_INFO](https://docs.ros2.org/bouncy/api/rclcpp/logging_8hpp.html#aeb160b6dd1edb7273480560c1027b264) function.

**ANSWER**:

```cpp
RCLCPP_INFO(this->get_logger(), "Spaceship got the following velocity command: v=(%.2f, %.2f, %.2f), w=(%.2f, %.2f, %.2f)", msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z); // Logging
```
---

## 5) RQt Tools

Compile the package and run the spaceship node. Even though this time we logged the info inside the callback to the console, still nothing is printing. The reason for that is because there are no messages coming in through ```cmd_vel```.

You could run a teleop node. For example, on another terminal, run:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

If you press the keys shown in the teleop console and then move back to the spaceship console (have that running also), you should see some message like:

```
[INFO] [1723317348.860140518] [voyager_spaceship]: Spaceship got the following velocity command: v=(0.50, 0.00, 0.00), w=(0.00, 0.00, 0.00)
```

That is because the keyboard teleop node publishes its commands to ```cmd_vel```. That is useful when you want to remote control the robot.

Another option is to use RQt. [RQt](https://docs.ros.org/en/humble/Concepts/Intermediate/About-RQt.html) is a GUI for interacting with ROS. It consists of multiple plugins for topic interaction, service calling, image viewing, etc. Debugging using graphical tools is much easier than through the terminal. 

Start RQt by just typing ```rqt``` on a terminal.

![RQt interface](../../images/rqt.png "RQt interface")

If you select the "plugins" tab on the top of the screen, then "topics->message publisher", a new interface should pop up. From there, you can select the topic, message type and frequency to publish. Click on the "+" symbol to add the selection to the list. You can type right in this window to add a value under any of the "expressions". Once you are done editing values, check the box next to the topic to start publishing, such as in the picture below. 

![RQt topic](../../images/rqt_topic.png "RQt topic")

The info on your other spaceship terminal are shown below. Note their frequency compared to the message publisher in the image.  This frequency has been set by your settings in RQt.  Uncheck the box and you'll see that the output stops.

```
[INFO] [1723318007.389180580] [voyager_spaceship]: Spaceship got the following velocity command: v=(1.00, 0.00, 0.00), w=(0.00, -1.00, 0.00)
[INFO] [1723318008.438732786] [voyager_spaceship]: Spaceship got the following velocity command: v=(1.00, 0.00, 0.00), w=(0.00, -1.00, 0.00)
[INFO] [1723318009.489920590] [voyager_spaceship]: Spaceship got the following velocity command: v=(1.00, 0.00, 0.00), w=(0.00, -1.00, 0.00)
[INFO] [1723318010.539392951] [voyager_spaceship]: Spaceship got the following velocity command: v=(1.00, 0.00, 0.00), w=(0.00, -1.00, 0.00)
[INFO] [1723318011.568370053] [voyager_spaceship]: Spaceship got the following velocity command: v=(1.00, 0.00, 0.00), w=(0.00, -1.00, 0.00)
[INFO] [1723318012.568077373] [voyager_spaceship]: Spaceship got the following velocity command: v=(1.00, 0.00, 0.00), w=(0.00, -1.00, 0.00)

```

RQt is a fantastic tool! Take some time to look at its other options when you can.
Try the following plugins:
* Introspection > Node graph
* Topics > Topic Monitor

## 6) Task 3: ROS Parameters

In ROS 2, parameters are node properties that can be used to configure the node at startup and at runtime. In a sense, they work like class attributes. Unlike ROS1, in ROS 2 parameters are saved by the nodes themselves, and not by a parameter server. You can read more about parameters [here](https://docs.ros.org/en/humble/Concepts/Basic/About-Parameters.html).

Parameters consist of a key and value, with an optional descriptor you can fill in. 

For our spaceship case, suppose we can have multiple module configurations. For example, the spaceship can be launched with a "computer", "network module" and "sensors". But at some point, we might lose a module. Or use software to add functionality that creates new modules. In order to reflect these changes, we need to update the code. 

---

**QUESTION**: Given the text above, assume we want to have a parameter in the node that stores the existing modules by their names. Considering the value types accepted by the parameters in ROS 2, what type should this parameter be?

**HINT**: Look at "overview" [here](https://docs.ros.org/en/humble/Concepts/Basic/About-Parameters.html).

**ANSWER**:

```cpp
string[]
```

---
---

**QUESTION**: In the class constructor, declare a parameter named "components", of the same type determined above, with initial components "Computer", "Engines", "Network" and "Sensors".

**HINT**: [Tutorial on C++ parameters](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-CPP.html)

**ANSWER**:

```cpp
this->declare_parameter("components", std::vector<std::string>{"Computer", "Engines", "Network", "Sensors"}); // Declare parameter
```
---

After recompiling the package and relaunching the node, you can see that the parameter has actually been created inside the node. 

To get all parameters:

```bash
ros2 param list
```
Output:
```
/voyager_spaceship:
  components
  qos_overrides./parameter_events.publisher.depth
  qos_overrides./parameter_events.publisher.durability
  qos_overrides./parameter_events.publisher.history
  qos_overrides./parameter_events.publisher.reliability
  use_sim_time
```

Notice there are parameters, like ```use_sim_time``` (mentioned previously), that we did not explicitly declare, but that are created automatically by the parent class. The other parameters, named with prefix "qos", have to do with [quality of service](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html). This is a whole can of worms that far escapes the scope of this workshop, but do check these settings out if you plan on developing ROS professionally. 

To get a parameter value:

```bash
ros2 param get /voyager_spaceship components 
```

Output:

```
String values are: ['Computer', 'Engines', 'Network', 'Sensors']
```

You can also use the CLI tools to set parameters as well. 

---

**QUESTION**: Suppose we lost the sensors component in a collision with an asteroid. Use the CLI tools to remove that component from the parameters. 

**HINT**: Inspiration [here](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html).

**ANSWER**:

```bash
ros2 param set /voyager_spaceship components "[Computer, Engines, Network]"
```

---


## 7) Task 4: ROS Services

Finally, let us implement a service that tells the status of the loaded components. We want some service that has a string as a request (the component's name) and also has a string as a response (the component's status). 

---

**QUESTION**: We want to create our own interface for this problem. Take a look at package ```spaceship_interfaces```. What is the name of the service interface defined? What are its request and response fields? 

**HINT**: Look inside the "srv" folder. That is where service interfaces are [defined](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html).

**ANSWER**:

Service interface is called "Status.srv"

It has a request named "component" (string) and a response named "status" (string)

---

In ROS 2, it is common to keep interfaces in a separate package to the one where it will be used. That is because packages that generate interfaces are built differently, so [getting interfaces and nodes in the same package](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface.html) can cause some headache. 

Go back to the ```spaceship``` package. If you look at the included packages, you'll see that the ```spaceship_interfaces``` package is there already with its service. 

---

**QUESTION**: Like the previous publisher and subscriber, create a service attribute in the private section of the class. This service will take the interface defined in the ```spaceship_interfaces```

**HINT**: [Official tutorial on service servers and clients](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html).

**ANSWER**:

```cpp
rclcpp::Service<spaceship_interfaces::srv::Status>::SharedPtr status_server_; // Service server mechanism
```
---

---

**QUESTION**: On the class constructor, start a service named ```get_status``` that binds to the callback function ```system_status```. 

**HINT**: This is extremely similar to the subscriber, but notice that the callback takes two arguments: the request and the reply. This means you'll need two placeholders. You can also find help in the [official tutorial on service servers and clients](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html).

**ANSWER**:

```cpp
this->status_server_ = this->create_service<spaceship_interfaces::srv::Status>("get_status", std::bind(&Spaceship::system_status, this, std::placeholders::_1, std::placeholders::_2)); // Announce the service server
```
---

Take a look at the callback function you just bound the service to, named ```system_status```. Notice its arguments. Like the message in the subscriber, we make the request a ```const```, meaning it must not be altered by the code. However, unlike the subscriber, we pass the arguments as shared pointers rather than simple references. This is because service interfaces, as you can see in the [official tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html), have their response changed in place. As such, shared pointer is the safest way to go about it. You could also make the subscriber callback argument a shared pointer if you wanted to, but it doesn't matter as much. 

Now we will write the body of the service server callback.

---

**QUESTION**: Inside the service server callback, load the parameter with the components into a vector variable. 

**HINT**: If the command to write the parameter is ```set_parameter```, what is the command to get parameters?... You also have to load the parameter in a way C++ understands. Answer can be adapted from [here](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-CPP.html) too.

**ANSWER**:

```cpp
std::vector<std::string> components = this->get_parameter("components").as_string_array(); // Load and convert param
```

---

---

**QUESTION**: Process the request passed the following way: if the component is present in the parameters, just say it is good. If not, say the component is non-existent. This exercise involves obtaining the requested component from the interface and also altering the interface to give back the request. 

**HINT**: The [official tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html) is a good place to check for interface manipulation. Also, [std::count](https://www.geeksforgeeks.org/std-count-cpp-stl/) is a quick fix to see if the requested element is in the parameter list. 

**ANSWER**:

```cpp
int cnt = std::count(components.begin(), components.end(), request->component);

if(cnt > 0)
{
  response->status = "Component is good!";
}
else
{
  response->status = "Non-existent component!";
}
```

---

One question that pops up a lot is when to use services or topics to transmit information. The general idea is:

| Topics    | Services |
| :--------- | :-------- |
| Continuous streaming  | Information on demand    |
| Subscriber independent | Need some sort of variable input    |

People say that you should always use topics for things like sensors. That is not true. If your sensor data is heavy, like an image, it might be best to keep the data on demand only and make it a service (like a snapshot). One very important similarity between topics and services is that they both should be **fast**. Don´t use a service for a lengthy calculation or process. For those cases, prefer the more advanced concept of [actions](https://docs.ros.org/en/humble/Tutorials/Intermediate/Creating-an-Action.html), which unfortunately is too much to cover in this workshop. 

Finally, use RQt to call the service you just created. Remember to recompile the package.

---

**QUESTION**: Open RQt. Which plugin would you use for calling the service? Try calling the service with a couple values.

**HINT**: Look inside the "services" menu of the plugin.

**ANSWER**:

"services->service caller"

Here is an example of request and response:

![RQt service](../../images/RQt_service.png "RQt service")

---

## 8) Advanced concepts and standards

The node developed here is very simple and serves as a good starting point, but it is not the actual standard that is used (or at least should be used) by professional ROS developers. Three main features of ROS 2 nodes have been left out: executors, compositions, and managed nodes.

[Executors](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Executors.html) are a more efficient way of running nodes than the ```spin()``` function at the end of the source code. They basically allow for a better threading model, permitting, for example, multiple threads. This solves many blocking call issues (not all of them though). 

[Compositions](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Composition.html) are another big thing in ROS 2, and they allow your node to be loaded more like a library than like an executable. Composable nodes walk hand-in-hand with executors, as many nodes can be loaded into a single process rather than multiple different processes in the machine. [This increases speed and reduces the hardware resources needed for running the ROS 2 program](https://arxiv.org/pdf/2305.09933). 


Finally, [managed nodes](https://design.ros2.org/articles/node_lifecycle.html) introduce the concept of lifecycle, which lets you treat the node as a state machine, more closely controlling what features are available at which points of the node's execution. 


For more information on these topics, Marco Matteo Bassa's [book](https://leanpub.com/averyinformaljourneythroughros2) is a fantastic source of information.

---

**CHALLENGE**: Transform the node code so as to implement one of the concepts mentioned above. You can choose any.

---