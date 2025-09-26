# main_controller_2.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HandServo:
    def __init__(self, name):
        self.name = name
        self.position = 0
    def move(self, pos):
        self.position = pos
        print(f"{self.name} moved to {pos}")

class MainController2(Node):
    def __init__(self):
        super().__init__('main_controller_2')
        self.subscription = self.create_subscription(String, '/actions_main2', self.listener_callback, 10)
        self.get_logger().info("Main Controller 2 Started")

        self.hand_servos = [HandServo(f"Hand{i}") for i in range(1,5)]

    def listener_callback(self, msg):
        action = msg.data
        self.get_logger().info(f'Received: {action}')

        # Example: "hand1:90;hand2:45"
        commands = action.split(";")
        for cmd in commands:
            name, pos = cmd.split(":")
            idx = int(name[-1])-1
            self.hand_servos[idx].move(int(pos))

def main(args=None):
    rclpy.init(args=args)
    node = MainController2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
