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
colcon build

# Source the Workspace:
source install/setup.bash
```

## ROS Packages
A ROS Package is a fundamental unit of software organization in ROS. Each package contains the software and resources needed to implement specific functionalities, such as nodes, libraries, datasets, configuration files, and third-party dependencies. Packages are designed to be modular and reusable.

```bash
# Creating a ROS Package:
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake package_name
cd ~/ros2_ws

# Build Package:
colcon build

# Source the Workspace:
source install/setup.bash
```

In summary, a ROS Workspace is an environment that contains and organizes your ROS packages, while ROS Packages are the individual units within the workspace that contain the code and resources for specific functionalities.

#####################################################################################################################################################################################
# PLAN!!! (nnt dihapus)
### Tutor Content: pub-sub (v), server-client (v), interfaces (v), params, plugins, actions, launch file
### Next: TF2, urdf, RViz2, Gazebo, SLAM TOOLBOX, NAVIGATION STACK, OpenCV
#####################################################################################################################################################################################

# ROS2 Humble Publisher-Subscriber

In ROS2, the Publisher-Subscriber model is a fundamental communication paradigm that allows nodes to exchange messages in a decoupled manner. This model is used to implement asynchronous communication between nodes. Here’s a detailed explanation of how it works:

### Publisher-Subscriber Model

#### Publisher
- A **Publisher** is a node that sends (publishes) messages to a topic. A topic is a named bus over which nodes exchange messages.
- To publish messages, the publisher node needs to create a publisher object for a specific topic and message type.
- The publisher periodically publishes messages to the topic, which can then be received by any number of subscribers.

#### Subscriber
- A **Subscriber** is a node that receives (subscribes to) messages from a topic.
- To receive messages, the subscriber node needs to create a subscriber object for a specific topic and message type.
- The subscriber listens to the topic and processes incoming messages using a callback function that gets triggered whenever a new message arrives.

### Example

Here's an example to demonstrate the Publisher-Subscriber model in ROS2 using Python.

#### Publisher Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, world!'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Subscriber Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### How It Works

1. **Publisher Node (`minimal_publisher`)**:
   - Creates a publisher object for the topic named `'topic'` with the message type `String`.
   - Uses a timer to periodically call the `timer_callback` method, which creates and publishes a `String` message with the data `'Hello, world!'`.

2. **Subscriber Node (`minimal_subscriber`)**:
   - Creates a subscriber object for the topic named `'topic'` with the message type `String`.
   - Defines a `listener_callback` method that gets called whenever a new message is received on the topic. This method logs the received message data.

### Running the Example

1. Start the publisher node:
   ```sh
   ros2 run <your_package_name> minimal_publisher
   ```

2. Start the subscriber node:
   ```sh
   ros2 run <your_package_name> minimal_subscriber
   ```

The subscriber node will log the message `'Hello, world!'` each time it receives a message from the publisher node.

### Conclusion

The Publisher-Subscriber model in ROS2 is a powerful mechanism for decoupling nodes and allowing them to communicate asynchronously. This model is particularly useful in robotic applications where multiple sensors and actuators need to exchange information in a scalable and efficient manner.


# ROS2 Humble Service-Client

In ROS2, the Service-Client model is used for synchronous communication between nodes, where one node (the client) makes a request to another node (the service) and waits for a response. This model is useful for operations where a request-response interaction is needed, and it complements the Publisher-Subscriber model for scenarios where immediate feedback or specific queries are required.

### Service-Client Model

#### Service
- A **Service** is a node that provides a request-response mechanism. It waits for incoming service requests and responds with a result.
- A service is defined by two message types: one for the request and one for the response.
- When a node offers a service, it registers it with the ROS2 Master, making it available for clients to call.

#### Client
- A **Client** is a node that requests services from a service provider. It sends a request to the service and waits for a response.
- The client specifies the request type and expects a response of a corresponding type.
- Clients use the service definition to make requests and handle responses.

### Example

Here’s a basic example to illustrate the Service-Client model in ROS2 using Python.

#### Service Node

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.handle_add_two_ints)

    def handle_add_two_ints(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Request: a={request.a}, b={request.b}; Response: sum={response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    minimal_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Client Node

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()
        self.req.a = 3
        self.req.b = 4
        self.send_request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(self.callback)

    def callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Result: {response.sum}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClient()
    rclpy.spin(minimal_client)
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### How It Works

1. **Service Node (`minimal_service`)**:
   - Creates a service object for the service named `'add_two_ints'` with the request and response types defined in the `AddTwoInts` service.
   - Implements the `handle_add_two_ints` method to process incoming requests and generate responses.

2. **Client Node (`minimal_client`)**:
   - Creates a client object for the service named `'add_two_ints'`.
   - Sends a request with parameters `a=3` and `b=4` to the service and waits for the response.
   - The response is logged by the callback function, which handles the result of the service call.

### Running the Example

1. Start the service node:
   ```sh
   ros2 run <your_package_name> minimal_service
   ```

2. Start the client node:
   ```sh
   ros2 run <your_package_name> minimal_client
   ```

The client node will request the sum of `3` and `4` from the service, and the service node will respond with the result, which the client will then log.

### Conclusion

The Service-Client model in ROS2 is essential for scenarios where nodes need to perform synchronous operations and receive immediate feedback. It complements the Publisher-Subscriber model by providing a mechanism for requesting specific data or actions and receiving responses.

* Interfaces (msg and srv):
    * Message (msg) Files define the data structure for messages sent over topics. These are simple text files where each line defines a field in the message, with a type and a name.
    * Service (srv) Files define the request and response data structures for services. These are also text files, split into two parts by a --- delimiter, with the request fields on top and the response fields on the bottom.
* Plugins:
    Plugins are modular components that can be loaded and used at runtime. In ROS, plugins allow you to extend the functionality of existing systems. For example, in the ROS visualization tool RViz, you can create plugins to add custom displays or tools. Plugins are typically implemented using a pluginlib library, which handles loading and unloading of plugins.
* Launch:
    Launch file is a files that contains code for running multiple nodes in one command.