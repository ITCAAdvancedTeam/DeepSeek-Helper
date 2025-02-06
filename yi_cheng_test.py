import rclpy
import json
import os
import argparse
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool
from ollama import Client  # Assuming synchronous client


class TrafficSignInterpreter(Node):
        
    def __init__(self, model_name, cfg_file):
        super().__init__('traffic_sign_interpreter')
        self.client = Client()  # Synchronous client
        self.model_name = model_name
        self.ros2_messages = self.load_ros2_messages(cfg_file)
        self._publishers = self.create_publishers()  # Use _publishers instead of publishers

        # self.subscription = self.create_subscription(
        #     String,
        #     "/traffic_sign_text",
        #     self.traffic_sign_callback,
        #     10
        # )

        print(f"{json.dumps({key: value['description'] for key, value in self.ros2_messages.items()}, indent=2)}\n\n")
        self.get_logger().info(f"🚦 Traffic Sign Interpreter Node Started... Listening on /traffic_sign_text")
        self.get_logger().info(f"🧠 Using Model: {model_name}")
        self.get_logger().info(f"📂 Config File: {cfg_file}")

    def load_ros2_messages(self, cfg_file):
            """Load ROS2 messages from an external JSON file."""
            if not os.path.exists(cfg_file):
                self.get_logger().error(f"❌ Error: ROS2 messages file '{cfg_file}' not found.")
                exit(1)

            try:
                with open(cfg_file, "r", encoding="utf-8") as file:
                    return json.load(file)
            except json.JSONDecodeError as e:
                self.get_logger().error(f"❌ Error: Invalid JSON format in '{cfg_file}'.\nDetails: {e}")
                exit(1)

    def create_publishers(self):
        """Create ROS2 publishers dynamically based on message types."""
        publishers = {}
        for key, msg_info in self.ros2_messages.items():
            topic_name = f"/usr/{key}"
            msg_type = msg_info["ros2_message_type"]

            if msg_type == "std_msgs/msg/Float32":
                publishers[key] = self.create_publisher(Float32, topic_name, 10)
            elif msg_type == "std_msgs/msg/Bool":
                publishers[key] = self.create_publisher(Bool, topic_name, 10)
            elif msg_type == "std_msgs/msg/String":
                publishers[key] = self.create_publisher(String, topic_name, 10)

        return publishers



def main(args=None):
    """Main function to initialize ROS2 and parse arguments."""
    parser = argparse.ArgumentParser(description="Traffic Sign Interpreter Node")
    parser.add_argument("--model", type=str, default="deepseek-r1:1.5b", help="DeepSeek Model to use for interpretation")
    parser.add_argument("--cfg_file", type=str, default="./json_files/traffic_signs.json", help="Path to traffic sign configuration JSON file")

    args = parser.parse_args()

    # Convert the parsed arguments to a list of strings
    rclpy_args = []
    if args.model:
        rclpy_args.extend(["--model", args.model])
    if args.cfg_file:
        rclpy_args.extend(["--cfg_file", args.cfg_file])

    rclpy.init(args=rclpy_args)
    node = TrafficSignInterpreter(args.model, args.cfg_file)

    
    try:
        rclpy.spin(node)  # Use standard spin to wait for ROS2 callbacks
    except KeyboardInterrupt:
        node.get_logger().info("👋 Shutting down Traffic Sign Interpreter.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()