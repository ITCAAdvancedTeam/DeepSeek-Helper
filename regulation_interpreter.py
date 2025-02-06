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

    def match_traffic_sign(self, text):
        """Use DeepSeek to match traffic sign text to a category and extract value."""
        if not text:
            self.get_logger().warn("⚠️ Empty traffic sign text received.")
            return None, None

        # Step 1: Match the text to a category
        messages = [
            {
                "role": "user",
                "content": (
                    f"Given this user command:\n\n"
                    f"Command: \"{text}\"\n\n"
                    f"Match it to the best ROS2 message category from this list:\n"
                    f"{json.dumps({key: value['description'] for key, value in self.ros2_messages.items()}, indent=2)}\n\n"
                    f"Return the best-matched key (e.g., speed_limit)"
                    # f"Return the best-matched key (e.g., speed_limit) without extra text, formatted like this:\n"
                    # f"KEY"
                )
            }
        ]

        try:
            # Synchronous request to get the category (including reasoning)
            key_response = self.client.chat(model=self.model_name, messages=messages)  
            key_response = key_response['message']['content'].strip()
            self.get_logger().info(f"key_response: {key_response}")

            # Search for the key from the end of the response
            key_response = self.extract_key_from_end(key_response)

        except KeyError as e:
            self.get_logger().error(f"⚠️ Error in processing key response: {e}")
            return None, None

        if key_response not in self.ros2_messages:
            self.get_logger().warn(f"⚠️ Invalid key: {key_response}")
            return None, None

        self.get_logger().info(f"🔍 Matched Traffic Sign Category: {key_response}")

        # Step 2: Extract the value and unit
        if self.ros2_messages[key_response]['ros2_message_type'] == "std_msgs/msg/Float32":
            messages = [
                {
                    "role": "user",
                    "content": (
                        f"Extract the numerical value from this command:\n\n"
                        f"Command: \"{text}\"\n"
                        f"ROS2 Message: \"{key_response}\" ({self.ros2_messages[key_response]['description']})\n"
                        f"Expected Unit: \"{self.ros2_messages[key_response]['unit']}\"\n\n"
                        f"Extract the numerical value from the command, and convert it into the expected unit if applicable. "
                        f"Return **only** the numerical value, **without any additional text or units**, formatted like this:\n"
                        f"VALUE"
                    )
                }
            ]

            # Synchronous request to get the value (including reasoning)
            value_response = self.client.chat(model=self.model_name, messages=messages)
            value_response = value_response['message']['content'].strip()
            self.get_logger().info(f"value_response: {value_response}")
            # Extract the value based on the ROS2 message type
            value_response = self.extract_value_based_on_type(value_response, key_response)
        elif self.ros2_messages[key_response]['ros2_message_type'] == "std_msgs/msg/Bool":
            messages = [
                {
                    "role": "user",
                    "content": (
                        f"Extract the boolean value from this command:\n\n"
                        f"Command: \"{text}\"\n"
                        f"ROS2 Message: \"{key_response}\" ({self.ros2_messages[key_response]['description']})\n"
                        f"Determine the value by comparing with the positive or negative expression of the key. "
                        f"Return 1 if it is true and 0 if it is false, formatted like this:\n"
                        f"VALUE"
                    )
                }
            ]

            # Synchronous request to get the value (including reasoning)
            value_response = self.client.chat(model=self.model_name, messages=messages)
            value_response = value_response['message']['content'].strip()
            self.get_logger().info(f"value_response: {value_response}")
            # Extract the value based on the ROS2 message type
            value_response = self.extract_value_based_on_type(value_response, key_response)
        else:
            value_response = text
            self.get_logger().info(f"value_response: {value_response}")
        
        self.get_logger().info(f"🔍 Extract value: {value_response}")

        if value_response is None:
            self.get_logger().warn("⚠️ Unable to extract a valid value.")
            return None, None

        return key_response, value_response

    def extract_key_from_end(self, response):
        """Extract the key by searching from the end of the response, considering spaces and line breaks."""
        # Split the response by spaces and line breaks to get the words or tokens
        words = response.split()

        # Iterate through the words from the end, searching for the last valid key
        for word in reversed(words):
            # Check if the word matches any key in ros2_messages
            word = word.strip('"')
            word = word.strip('*')
            if word in self.ros2_messages:
                return word  # Return the first key found from the end

        return None

    def extract_value_based_on_type(self, response, key_response):
        """Extract the value based on the corresponding ROS2 message type, searching from the end of the reasoning response."""
        msg_type = self.ros2_messages[key_response]["ros2_message_type"]

        # Remove leading/trailing spaces and clean the response
        response = response.strip()

        # Reverse the response to search from the end
        reversed_response = response[::-1]

        # Check the expected message type and extract/convert the value accordingly
        if msg_type == "std_msgs/msg/Float32":
            # Try to find a numeric value from the end of the string
            for word in reversed_response.split():
                try:
                    # Attempt to convert each word to a float
                    value = float(word[::-1])  # Reverse the word back to normal
                    return value
                except ValueError:
                    continue  # If the word is not a valid float, continue with the next one

            self.get_logger().warn(f"⚠️ Unable to extract a valid Float32 from '{response}'.")
            return None

        elif msg_type == "std_msgs/msg/Bool":
            # For boolean values, check common truthy/falsy values from the end of the string
            for word in reversed_response.split():
                if word[::-1].lower() in ["true", "1", "yes"]:
                    return True
                elif word[::-1].lower() in ["false", "0", "no"]:
                    return False

            self.get_logger().warn(f"⚠️ Unable to extract a valid boolean from '{response}'.")
            return None

        elif msg_type == "std_msgs/msg/String":
            # For string values, return the response as-is, searching from the end
            # You can add extra logic if you want to clean the string further
            return response

        else:
            self.get_logger().warn(f"⚠️ Unsupported ROS2 message type: {msg_type}.")
            return None

    def process_traffic_sign(self, text):
        """Process traffic sign text, match category, extract value, and publish to ROS2."""
        self.get_logger().info("DeepSeek AI Processing... 🔄")
        msg_type, extracted_value = self.match_traffic_sign(text)

        if msg_type and extracted_value is not None:
            topic = f"/usr/{msg_type}"
            self.get_logger().info(f"✅ Publishing to {topic}: {extracted_value}")

            # Publish to ROS2 topic
            msg_type_info = self.ros2_messages[msg_type]["ros2_message_type"]
            if msg_type_info == "std_msgs/msg/Float32":
                msg = Float32()
                msg.data = extracted_value
            elif msg_type_info == "std_msgs/msg/Bool":
                msg = Bool()
                msg.data = extracted_value
            else:  # String message
                msg = String()
                msg.data = extracted_value

            self._publishers[msg_type].publish(msg)  # Use _publishers
        else:
            self.get_logger().warn("⚠️ No matching ROS2 message found.")

    def traffic_sign_callback(self, msg):
        """ROS2 Subscription Callback for /traffic_sign_text."""
        self.get_logger().info(f"📩 Received: {msg.data}")

        # Synchronously process the traffic sign message
        self.process_traffic_sign(msg.data)

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
