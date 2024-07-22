# Introduction to ROS

## Running Executables
List of Packages
```bash
ros2 pkg list
```
List of <packages, executables>
```bash
ros2 pkg executables
```
Run Node or Executable
```bash
ros2 run <package_name> <executable_name>
```

## Turtlesim
```bash
# Install Turtlesim
sudo apt update
sudo apt install ros-humble-turtlesim

# Run Turtlesim
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key
```
### Try to move the turtle using keyboard teleoperation

## RQt as GUI Interface for ROS
```bash
# Install RQt
sudo apt update
sudo apt install '~nros-humble-rqt*'

# Run RQt
rqt
```
### 1. Spawn a turtle in RQt
    * Plugins => Services => Service Caller
    * Under Service drop down, choose /spawn
    * Enter x, y position and click "Call"
### 2. Edit turtle path color
    * Under Service drop down, choose /turtle1/set_pen
    * Modify "r" value to 255 and click "Call"
    * Move the turtle
### 3. Change the teleoperation for new turtle
    ```bash
    ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel
    ```

## ROS Basic Knowledge
Here is an explanation of the core concepts in ROS:
* Nodes:
    Nodes are the fundamental building blocks of a ROS-based system. A node is a process that performs computation. ROS is designed to be modular, with many small nodes each responsible for a specific part of the system. Nodes can communicate with each other using topics, services, and actions.
* Topics:
    Topics are named buses over which nodes exchange messages. A node can publish messages to a topic as a publisher, and other nodes can subscribe to that topic to receive the messages. This is a way to implement asynchronous communication between nodes. For example, a camera node might publish images to a topic, and a processing node might subscribe to that topic to receive and process the images.
* Services:
    Services provide a synchronous communication mechanism in ROS. They are used for request-response interactions. A service is defined by a pair of message structures: one for the request and one for the response. A node can offer a service, and another node can call that service and wait for a response. For example, a robot arm control node might offer a service to move the arm to a specified position.
* Actions:
    Actions are designed for handling long-running tasks. Unlike services, actions support preemptive goals, feedback, and result messages, allowing for better control over long-duration tasks. Actions use three types of messages: goal, feedback, and result. For example, a navigation node might provide an action to move to a specified location, offering feedback on the progress and allowing the goal to be preempted if needed.
* Parameters:
    Parameters are used to store configuration settings and other data that can be accessed by nodes. They are stored on the ROS Parameter Server, which allows nodes to read and write parameters. Parameters can be used for things like setting thresholds, calibration data, or mode of operation.
* Interfaces (msg and srv):
    * Message (msg) Files define the data structure for messages sent over topics. These are simple text files where each line defines a field in the message, with a type and a name.
    * Service (srv) Files define the request and response data structures for services. These are also text files, split into two parts by a --- delimiter, with the request fields on top and the response fields on the bottom.
* Plugins:
    Plugins are modular components that can be loaded and used at runtime. In ROS, plugins allow you to extend the functionality of existing systems. For example, in the ROS visualization tool RViz, you can create plugins to add custom displays or tools. Plugins are typically implemented using a pluginlib library, which handles loading and unloading of plugins.
* Launch:
    Launch file is a files that contains code for running multiple nodes in one command.
Each of these components plays a crucial role in building a modular, flexible, and scalable robot software system using ROS.

# PLAN
## Topics, Services, Parameters, Actions, Launch