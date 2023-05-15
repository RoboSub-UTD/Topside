import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class VideoSubscriber(Node):

    def __init__(self):
        super().__init__('video_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'video_stream',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg)
            if cv_image is not None and cv_image.size != 0:  # Check if the image is not empty
                cv2.imshow('video', cv_image)
                k = cv2.waitKey(1)  # Display the image for 1 ms

                # If 'q' is pressed on the keyboard,
                # rclpy.shutdown() will be called to stop the node.
                if k & 0xFF == ord('q'):
                    rclpy.shutdown()
            else:
                self.get_logger().warn('Received an incomplete frame')

        except Exception as e:
            self.get_logger().error('Error in listener_callback: %s' % str(e))


def main(args=None):
    rclpy.init(args=args)

    video_subscriber = VideoSubscriber()

    rclpy.spin(video_subscriber)

    video_subscriber.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()