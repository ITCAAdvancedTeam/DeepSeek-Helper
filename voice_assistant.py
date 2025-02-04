import argparse
import asyncio
import json
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from ollama import AsyncClient

class AssistantNode(Node):
    def __init__(self, model_name, msg_file):
        super().__init__('ros2_publisher')
        self.client = AsyncClient()
        self.model_name = model_name
        self.ros2_messages = self.load_ros2_messages(msg_file)
        self.publishers = self.create_publishers()

    def load_ros2_messages(self, msg_file):
        """Load ROS2 messages configuration from an external JSON file."""
        if not os.path.exists(msg_file):
            self.get_logger().error(f"❌ Error: ROS2 messages configuration file '{msg_file}' not found.")
            exit(1)

        try:
            with open(msg_file, "r", encoding="utf-8") as file:
                return json.load(file)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"❌ Error: Invalid JSON format in '{msg_file}'.\nDetails: {e}")
            exit(1)

    def create_publishers(self):
        """Create ROS2 publishers dynamically based on message types."""
        publishers = {}
        for key, msg_info in self.ros2_messages.items():
            topic_name = f"/usr/{key}"
            msg_type = msg_info["message_type"]

            if msg_type == "std_msgs/msg/Float32":
                publishers[key] = self.create_publisher(Float32, topic_name, 10)
            elif msg_type == "std_msgs/msg/Bool":
                publishers[key] = self.create_publisher(Bool, topic_name, 10)
            else:
                self.get_logger().warn(f"⚠️ Unsupported message type: {msg_type} for {key}")

        return publishers

    async def match_message(self, text):
        """Use DeepSeek Ollama to match user input with a ROS2 message and extract values."""
        if not text:
            self.get_logger().warn("⚠️ No input provided.")
            return None, None

        # Step 1: AI Determines the Matching ROS2 Message
        messages = [
            {
                "role": "user",
                "content": (
                    f"Given this user command:\n\n"
                    f"Command: \"{text}\"\n\n"
                    f"Match it to the best ROS2 message category from this list:\n"
                    f"{json.dumps({key: value['description'] for key, value in self.ros2_messages.items()}, indent=2)}\n\n"
                    f"Return **only** the exact key (e.g., 'ego_vehicle_speed') without extra text."
                )
            }
        ]
        key_response = ""

        async for part in (await self.client.chat(model=self.model_name, messages=messages, stream=True)):
            key_response += part['message']['content'].strip()

        key_response = key_response.strip('"\'')

        # Validate the extracted key
        if key_response not in self.ros2_messages:
            self.get_logger().warn(f"⚠️ Invalid key response from AI: {key_response}")
            return None, None

        self.get_logger().info(f"🔍 Matched ROS2 Message: {key_response}")

        # Step 2: AI Determines the Value (With Correct Unit)
        messages = [
            {
                "role": "user",
                "content": (
                    f"Extract the numerical value from this command:\n\n"
                    f"Command: \"{text}\"\n"
                    f"ROS2 Message: \"{key_response}\" ({self.ros2_messages[key_response]['description']})\n"
                    f"Expected Unit: \"{self.ros2_messages[key_response]['unit']}\"\n\n"
                    f"Return only the extracted value in the correct unit, formatted like this:\n"
                    f"VALUE (without any additional text)."
                )
            }
        ]
        value_response = ""

        async for part in (await self.client.chat(model=self.model_name, messages=messages, stream=True)):
            value_response += part['message']['content'].strip()

        # Validate extracted value
        extracted_value = self.parse_value(value_response, key_response)

        return key_response, extracted_value

    def parse_value(self, value_response, key_response):
        """Ensure AI response is properly formatted."""
        msg_type = self.ros2_messages[key_response]["message_type"]

        if msg_type == "std_msgs/msg/Bool":
            return value_response.lower() in ["true", "1", "yes"]

        try:
            return float(value_response)
        except ValueError:
            self.get_logger().warn(f"⚠️ Could not extract a valid number from: {value_response}")
            return None

    async def process_command(self, user_input):
        """Process: Match Message, Extract Values, and Publish to ROS2."""
        self.get_logger().info("DeepSeek AI Processing... 🔄")
        msg_type, extracted_value = await self.match_message(user_input)

        if msg_type and extracted_value is not None:
            topic = f"/usr/{msg_type}"
            self.get_logger().info(f"\n✅ Publishing to {topic}: {extracted_value} {self.ros2_messages[msg_type]['unit']}")

            # Publish to ROS2 topic
            msg = Float32() if self.ros2_messages[msg_type]["message_type"] == "std_msgs/msg/Float32" else Bool()
            msg.data = extracted_value
            self.publishers[msg_type].publish(msg)

        else:
            self.get_logger().warn("\n⚠️ No matching ROS2 message found.")

async def main():
    parser = argparse.ArgumentParser(description="ROS2 Command-Line Interpreter with ROS2 Publishing")
    parser.add_argument("--model", type=str, default="deepseek-r1:1.5b", help="Specify the DeepSeek model (default: deepseek-r1:1.5b)")
    parser.add_argument("--msg_file", type=str, default="./json_files/ego_command.json", help="Path to the ROS2 messages JSON file")

    args = parser.parse_args()
    rclpy.init()
    node = AssistantNode(args.model, args.msg_file)

    while True:
        user_input = input("\n📝 Enter a command (or type 'exit' to quit): ")
        if user_input.lower() == "exit":
            print("👋 Exiting interpreter.")
            break

        await node.process_command(user_input)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    asyncio.run(main())
