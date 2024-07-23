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

## RQt as GUI Interface for ROS
```bash
# Install RQt
sudo apt update
sudo apt install '~nros-humble-rqt*'

# Run RQt
rqt
```
1. Spawn a turtle in RQt
    * Plugins => Services => Service Caller
    * Under Service drop down, choose /spawn
    * Enter x, y position and click "Call"
2. Edit turtle path color
    * Under Service drop down, choose /turtle1/set_pen
    * Modify "r" value to 255 and click "Call"
    * Move the turtle
3. Change the teleoperation for new turtle
    ```bash
    ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel
    ```

## ROS Basic Knowledge
* ### ROS2 Nodes
    Nodes are the fundamental building blocks of a ROS-based system. A node is a process that performs computation. ROS is designed to be modular, with many small nodes each responsible for a specific part of the system. Nodes can communicate with each other using topics, services, and actions.

* ### ROS2 Topics:
    Topics are named buses over which nodes exchange messages. A node can publish messages to a topic as a publisher, and other nodes can subscribe to that topic to receive the messages. This is a way to implement asynchronous communication between nodes. For example, a camera node might publish images to a topic, and a processing node might subscribe to that topic to receive and process the images.
    1. List of Topics
        ```bash
        ros2 topic list
        ros2 topic list -t # more details
        ```
    2. Topic Output
        ```bash
        ros2 topic echo <topic_name>
        ros2 topic echo /turtle1/cmd_vel
        ```
    3. Topic Info
        ```bash
        ros2 topic info <topic_name>
        ros2 topic info /turtle1/cmd_vel
        ```
    4. Publish Data to a Topic
        ```bash
        ros2 topic pub <topic_name> <msg_type> '<args>'
        ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "linear:
            x: 0.0
            y: 10.0
            z: 0.0
            angular:
            x: 0.0
            y: 0.0
            z: 0.0"
        ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "linear:
            x: 0.0
            y: 10.0
            z: 0.0
            angular:
            x: 0.0
            y: 0.0
            z: 0.0"
        ```
    5. Topic Frequency
        ```bash
        ros2 topic hz <topic_name>
        ros2 topic hz /turtle1/pose
        ```

* ### ROS2 Services:
    Services provide a synchronous communication mechanism in ROS. They are used for request-response interactions. A service is defined by a pair of message structures: one for the request and one for the response. A node can offer a service, and another node can call that service and wait for a response. For example, a robot arm control node might offer a service to move the arm to a specified position.
    1. List of Services
        ```bash
        ros2 service list
        ros2 service list -t # service type
        ```
    2. Service Type
        ```bash
        ros2 service type <service_name>
        ros2 service type /clear
        ```
    3. Find Service
        ```bash
        ros2 service type <service_name>
        ros2 service type std_srvs/srv/Empty
        ```
    4. Calling a Service
        ```bash
        ros2 service call <service_name> <service_type> <arguments>
        ros2 service call /clear std_srvs/srv/Empty
        ros2 service call /spawn turtlesim/srv/Spawn
        ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: ''}"
        ```

* ### ROS2 Parameters:
    Parameters are used to store configuration settings and other data that can be accessed by nodes. They are stored on the ROS Parameter Server, which allows nodes to read and write parameters. Parameters can be used for things like setting thresholds, calibration data, or mode of operation.
    1. List of Parameters
        ```bash
        ros2 param list
        ```
    2. Get Parameter Value
        ```bash
        ros2 param get <node_name> <parameter_name>
        ros2 param get /turtlesim background_g
        ```
    3. Set Parameter Value
        ```bash
        ros2 param set <node_name> <parameter_name> <value>
        ros2 param set /turtlesim background_g 255
        ```
    4. View All Params For A Node
        ```bash
        ros2 param dump <node_name>
        ros2 param dump /turtlesim
        ```
    5. Store Param in YAML File
        ```bash
        ros2 param dump /turtlesim > turtlesim.yaml
        ```
    6. Load Param from YAML File
        ```bash
        ros2 param load /turtlesim turtlesim.yaml
        ```
    7. Load Param on Startup
        ```bash
        ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>
        ros2 run turtlesim turtlesim_node --ros-args --params-file turtlesim.yaml
        ```

* ### ROS2 Actions:
    Actions are designed for handling long-running tasks. Unlike services, actions support preemptive goals, feedback, and result messages, allowing for better control over long-duration tasks. Actions use three types of messages: goal, feedback, and result. For example, a navigation node might provide an action to move to a specified location, offering feedback on the progress and allowing the goal to be preempted if needed.
    1. Try Actions on Teleop Window
        * Press 'G'
        * Press 'F' to cancel
    2. See Action Clients in Node Info
        ```bash
        ros2 node info /teleop_turtle
        ```
    3. See All Actions
        ```bash
        ros2 action list
        ros2 action list -t # with type
        ```
    4. See Action Info
        ```bash
        ros2 action info <action_name>
        ros2 action info /turtle1/rotate_absolute
        ```
    5. Send Goal
        ```bash
        ros2 action send_goal <action_name> <action_name> <goal>
        ros2 action info /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}"
        ```

* ### ROS2 Launch Multiple Nodes
    Using launch file, we can startup two nodes with just one command.
    1. Launch Two Nodes
        ```bash
        ros2 launch <package_name> <launch_name>
        ros2 launch turtlesim multisim.launch.py
        ```
    2. Details of Launch File (Next Section)
        ```python
        # turtlesim/launch/multisim.launch.py

        from launch import LaunchDescription
        import launch_ros.actions

        def generate_launch_description():
            return LaunchDescription([
                launch_ros.action.Node(
                    namespace="turtlesim1", package='turtlesim', executable='turtlesim_node', output='screen'),
                launch_ros.action.Node(
                    namespace="turtlesim1", package='turtlesim', executable='turtlesim_node', output='screen'),
            ])
        ```