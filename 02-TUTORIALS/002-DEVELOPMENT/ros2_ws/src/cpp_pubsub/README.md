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