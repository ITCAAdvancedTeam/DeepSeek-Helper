import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import argparse
 
class ImagePublisher(Node):
    """
    Create an ImagePublisher class, which is a subclass of the Node class.
    """
    def __init__(self, img_file):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('image_publisher')

        # Create the publisher. This publisher will publish an Image
        # to the topic. The queue size is 10 messages.
        self.publisher_ = self.create_publisher(Image, '/traffic_sign_image', 10)

        # We will publish a message every 2 seconds
        timer_period = 2  # seconds

        # Create the timer
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        self.image = cv2.imread(img_file)

    def timer_callback(self):
        """
        Callback function.
        This function gets called every 2 seconds.
        """

        self.publisher_.publish(self.br.cv2_to_imgmsg(self.image))

        # Display the message on the console
        self.get_logger().info('Publishing traffic sign image')

def main(args=None):

    parser = argparse.ArgumentParser(description="Traffic Sign Image Publisher Node")
    parser.add_argument("--img_file", type=str, default='./traffic_sign_images/speed_limit_70.jpg', help="Path to traffic sign images")

    args = parser.parse_args()
    rclpy_args = []
    if args.img_file:
        rclpy_args.extend(["--img_file", args.img_file])

    rclpy.init(args=rclpy_args)

    # Create the node
    image_publisher = ImagePublisher(args.img_file)

    # Spin the node so the callback function is called.
    rclpy.spin(image_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_publisher.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()

if __name__ == '__main__':
    main()
