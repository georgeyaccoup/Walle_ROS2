# chatgpt_processor.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai

openai.api_key = "YOUR_OPENAI_API_KEY"

class ChatGPTProcessor(Node):
    def __init__(self):
        super().__init__('chatgpt_processor')
        self.sub = self.create_subscription(String, '/voice_commands', self.callback, 10)
        self.pub_main1 = self.create_publisher(String, '/actions_main1', 10)
        self.pub_main2 = self.create_publisher(String, '/actions_main2', 10)
        self.get_logger().info("ChatGPT Processor Started")

    def callback(self, msg):
        command = msg.data
        self.get_logger().info(f"Voice Command: {command}")
        response = openai.ChatCompletion.create(
            model="gpt-4",
            messages=[{"role":"user","content":command}]
        )
        action_text = response['choices'][0]['message']['content'].strip()

        # Example: format "main1:forward;main2:hand1:90;hand2:45"
        for cmd in action_text.split(";"):
            if cmd.startswith("main1:"):
                m1_msg = String()
                m1_msg.data = cmd.replace("main1:","")
                self.pub_main1.publish(m1_msg)
            elif cmd.startswith("main2:"):
                m2_msg = String()
                m2_msg.data = cmd.replace("main2:","")
                self.pub_main2.publish(m2_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ChatGPTProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
