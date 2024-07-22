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