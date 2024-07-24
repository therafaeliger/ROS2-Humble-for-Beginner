# ROS2 Humble Development

## ROS Programming Structure
```bash
workspace_folder/
    src/
      cpp_package/
          CMakeLists.txt
          include/cpp_package/
          package.xml
          src/

      py_package/
          package.xml
          resource/py_package
          setup.cfg
          setup.py
          py_package/
      ...
```

## ROS2 Workspace
A ROS Workspace is a directory where you organize and build your ROS projects. It typically follows a standard structure that includes subdirectories for storing source code, build files, and other resources. The workspace is used to manage multiple ROS packages, allowing for modular development and easy dependency management.

Creating and Using a ROS Workspace
```bash
# Create a Workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build Package:
colcon build --symlink-install

# Source the Workspace:
source install/setup.bash
```

## ROS Packages
A ROS Package is a fundamental unit of software organization in ROS. Each package contains the software and resources needed to implement specific functionalities, such as nodes, libraries, datasets, configuration files, and third-party dependencies. Packages are designed to be modular and reusable.

```bash
# Creating a ROS Package:
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake <package_name>
cd ~/ros2_ws

# Build Package:
colcon build --packages-select <package_name>

# Source the Workspace:
source install/setup.bash
```

In summary, a ROS Workspace is an environment that contains and organizes your ROS packages, while ROS Packages are the individual units within the workspace that contain the code and resources for specific functionalities.

## ROS2 Publisher-Subscriber
### Publisher
* A Publisher is a node that sends (publishes) messages to a topic. A topic is a named bus over which nodes exchange messages.
* To publish messages, the publisher node needs to create a publisher object for a specific topic and message type.
* The publisher periodically publishes messages to the topic, which can then be received by any number of subscribers.
### Subscriber
* A Subscriber is a node that receives (subscribes to) messages from a topic.
* To receive messages, the subscriber node needs to create a subscriber object for a specific topic and message type.
* The subscriber listens to the topic and processes incoming messages using a callback function that gets triggered whenever a new message arrives.

In ROS2, the Publisher-Subscriber model is a fundamental communication paradigm that allows nodes to exchange messages in a decoupled manner. This model is used to implement asynchronous communication between nodes.

1. Create Package 
    ```bash
    cd ~/ros2_ws/src
    ros2 pkg create --build-type ament_cmake cpp_pubsub
    ```
2. Source code: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html
3. Update CMakeLists.txt (see file)
4. Update package.xml (see file)
5. Build Using Colcon
    ```bash
    colcon build --packages-select cpp_pubsub

    # Source the Workspace:
    source install/setup.bash
    ```
6. Running "talker" and "listener" on Different Terminal
    ```bash
    ros2 run cpp_pubsub talker
    ```
    ```bash
    ros2 run cpp_pubsub listener
    ```

## ROS2 Service-Client
### Service
* A Service is a node that provides a request-response mechanism. It waits for incoming service requests and responds with a result.
* A service is defined by two message types: one for the request and one for the response.
* When a node offers a service, it registers it with the ROS2 Master, making it available for clients to call.
### Client
* A Client is a node that requests services from a service provider. It sends a request to the service and waits for a response.
* The client specifies the request type and expects a response of a corresponding type.
* Clients use the service definition to make requests and handle responses.

In ROS2, the Service-Client model is used for synchronous communication between nodes, where one node (the client) makes a request to another node (the service) and waits for a response. This model is useful for operations where a request-response interaction is needed, and it complements the Publisher-Subscriber model for scenarios where immediate feedback or specific queries are required.

1. Create Package 
    ```bash
    cd ~/ros2_ws/src
    ros2 pkg create --build-type ament_cmake cpp_srvcli --dependencies rclcpp example_interfaces
    ```
    * Dependencies will be added to CMakeLists.txt and package.xml, [example_interface] contains request and response format.
        ```bash
        int64 a
        int64 b
        ---
        int64 sum
        ```
2. Source code: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html
3. Update CMakeLists.txt (see file)
4. Update package.xml (see file)
5. Build Using Colcon
    ```bash
    colcon build --packages-select cpp_pubsub

    # Source the Workspace:
    source install/setup.bash
    ```
6. Running "server" and "client" on Different Terminal
    ```bash
    ros2 run cpp_pubsub server
    ```
    ```bash
    ros2 run cpp_pubsub client 2 4
    ```

## ROS2 Interface
1. Create Package 
    ```bash
    cd ~/ros2_ws/src
    ros2 pkg create --build-type ament_cmake tutorial_interfaces
    ```
2. Create "msg" and "srv" Folder Inside The Package
    * Create your custom message (for example: Num.msg, Sphere.msg)
        ```bash
        # Num.msg
        int64 num

        # Sphere.msg
        geometry_msgs/Point center
        float64 radius
        ```
    * Create your custom service (for example: AddThreeInts.srv)
        ```bash
        int64 a
        int64 b
        int64 c
        ---
        int64 sum
        ```
3. Update CMakeLists.txt (see file)
4. Update package.xml (see file)
5. Build Using Colcon
    ```bash
    colcon build --packages-select tutorial_interfaces

    # Source the Workspace:
    source install/setup.bash
    ```
6. Check Interfaces Return
    ```bash
    ros2 interface show tutorial_interfaces/msg/Num
    ros2 interface show tutorial_interfaces/msg/Sphere
    ros2 interface show tutorial_interfaces/srv/AddThreeInts
    ```
7. Test on "cpp_pubsub" and "cpp_srvcli"

## ROS2 Action
1. Create "action" Folder Inside The "tutorial_interface" Package
    * Create your custom action (for example: Fibonacci.action)
        ```bash
        int32 order
        ---
        int32[] sequence
        ---
        int32[] partial_sequence
        ```
2. Update CMakeLists.txt (see file)
3. Update package.xml (see file)
4. Build Using Colcon
    ```bash
    colcon build --packages-select tutorial_interfaces

    # Source the Workspace:
    source install/setup.bash
    ```
5. Check Interfaces Return
    ```bash
    ros2 interface show tutorial_interfaces/action/Fibonacci
    ```
6. Test on "cpp_srvcli"