import rclpy
import json
import os
import asyncio
import argparse
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool
from ollama import AsyncClient

class TrafficSignInterpreter(Node):
    def __init__(self, model_name, cfg_file):
        super().__init__('traffic_sign_interpreter')
        self.client = AsyncClient()
        self.model_name = model_name
        self.ros2_messages = self.load_ros2_messages(cfg_file)
        self.publishers = self.create_publishers()

        self.subscription = self.create_subscription(
            String,
            "/traffic_sign_text",
            self.traffic_sign_callback,
            10
        )
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

    async def match_traffic_sign(self, text):
        """Use DeepSeek to match traffic sign text to a category and extract value."""
        if not text:
            self.get_logger().warn("⚠️ Empty traffic sign text received.")
            return None, None

        messages = [{"role": "user", "content": f"Match this sign to a category: {text}"}]
        key_response = ""

        async for part in (await self.client.chat(model=self.model_name, messages=messages, stream=True)):
            key_response += part['message']['content'].strip()

        key_response = key_response.strip('"\'')

        if key_response not in self.ros2_messages:
            self.get_logger().warn(f"⚠️ Invalid key: {key_response}")
            return None, None

        self.get_logger().info(f"🔍 Matched Traffic Sign Category: {key_response}")

        messages = [{"role": "user", "content": f"Extract value from: {text}"}]
        value_response = ""

        async for part in (await self.client.chat(model=self.model_name, messages=messages, stream=True)):
            value_response += part['message']['content'].strip()

        extracted_value = self.parse_value(value_response, key_response)

        return key_response, extracted_value

    def parse_value(self, value_response, key_response):
        """Ensure AI response is properly formatted."""
        msg_type = self.ros2_messages[key_response]["ros2_message_type"]

        if msg_type == "std_msgs/msg/Bool":
            return value_response.lower() in ["true", "1", "yes"]

        if msg_type == "std_msgs/msg/String":
            return value_response

        try:
            return float(value_response)
        except ValueError:
            self.get_logger().warn(f"⚠️ Invalid value: {value_response}")
            return None

    async def process_traffic_sign(self, text):
        """Process traffic sign text, match category, extract value, and publish to ROS2."""
        self.get_logger().info("DeepSeek AI Processing... 🔄")
        msg_type, extracted_value = await self.match_traffic_sign(text)

        if msg_type and extracted_value is not None:
            topic = f"/usr/{msg_type}"
            self.get_logger().info(f"✅ Publishing to {topic}: {extracted_value}")

            # Publish to ROS2 topic
            msg_type = self.ros2_messages[msg_type]["ros2_message_type"]
            if msg_type == "std_msgs/msg/Float32":
                msg = Float32()
                msg.data = extracted_value
            elif msg_type == "std_msgs/msg/Bool":
                msg = Bool()
                msg.data = extracted_value
            else:  # String message
                msg = String()
                msg.data = extracted_value

            self.publishers[msg_type].publish(msg)
        else:
            self.get_logger().warn("⚠️ No matching ROS2 message found.")

    def traffic_sign_callback(self, msg):
        """ROS2 Subscription Callback for /traffic_sign_text."""
        self.get_logger().info(f"📩 Received: {msg.data}")
        asyncio.create_task(self.process_traffic_sign(msg.data))

def main(args=None):
    """Main function to initialize ROS2 and parse arguments."""
    parser = argparse.ArgumentParser(description="Traffic Sign Interpreter Node")
    parser.add_argument("--model", type=str, default="deepseek-r1:1.5b", help="DeepSeek Model to use for interpretation")
    parser.add_argument("--cfg", type=str, default="./json_files/traffic_signs.json", help="Path to traffic sign configuration JSON file")

    args = parser.parse_args()

    rclpy.init(args=args)
    node = TrafficSignInterpreter(args.model, args.cfg)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("👋 Shutting down Traffic Sign Interpreter.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
