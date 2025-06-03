import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class EmotionPublisher(Node):
    def __init__(self):
        super().__init__('emotion_publisher')
        self.publisher = self.create_publisher(String, 'emotion_topic', 10)
        self.timer = self.create_timer(5.0, self.publish_emotion)  # 每5秒发布一次
        self.emotions = ["happy", "sad"]
        self.current_index = 0

    def publish_emotion(self):
        msg = String()
        msg.data = self.emotions[self.current_index]
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.current_index = (self.current_index + 1) % len(self.emotions)

def main():
    rclpy.init()
    node = EmotionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
