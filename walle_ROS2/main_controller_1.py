# main_controller_1.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

# Simulated actuators (replace with actual servo/motor libraries)
class Servo:
    def __init__(self, name):
        self.name = name
        self.position = 0
    def move(self, pos):
        self.position = pos
        print(f"{self.name} moved to {pos}")

class DCMotor:
    def __init__(self, name):
        self.name = name
    def forward(self): print(f"{self.name} moving forward")
    def backward(self): print(f"{self.name} moving backward")
    def stop(self): print(f"{self.name} stopped")

class MainController1(Node):
    def __init__(self):
        super().__init__('main_controller_1')
        self.subscription = self.create_subscription(String, '/actions_main1', self.listener_callback, 10)
        self.get_logger().info("Main Controller 1 Started")

        # Initialize actuators
        self.head_servos = [Servo("Head1"), Servo("Head2")]
        self.wheel_motors = [DCMotor("Wheel1"), DCMotor("Wheel2"), DCMotor("Wheel3")]

    def listener_callback(self, msg):
        action = msg.data
        self.get_logger().info(f'Received: {action}')

        # Head commands
        if action.startswith("head1:"):
            pos = int(action.split(":")[1])
            self.head_servos[0].move(pos)
        elif action.startswith("head2:"):
            pos = int(action.split(":")[1])
            self.head_servos[1].move(pos)
        # Wheel commands
        elif action == "forward":
            for w in self.wheel_motors: w.forward()
        elif action == "backward":
            for w in self.wheel_motors: w.backward()
        elif action == "stop":
            for w in self.wheel_motors: w.stop()

def main(args=None):
    rclpy.init(args=args)
    node = MainController1()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
