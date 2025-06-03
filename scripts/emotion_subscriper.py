import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame
import sys

class EmotionSubscriber(Node):
    def __init__(self):
        super().__init__('emotion_subscriber')
        self.subscription = self.create_subscription(
            String,
            'emotion_topic',
            self.emotion_callback,
            10)
        pygame.init()
        self.screen = pygame.display.set_mode((400, 400))
        pygame.display.set_caption("ROS2 Emotion Display")

    def emotion_callback(self, msg):
        emotion = msg.data
        self.get_logger().info(f'Received: "{emotion}"')
        draw_emotion(self.screen, emotion)  # 调用绘图函数

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)  # 非阻塞式处理消息
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

def draw_emotion(screen, emotion):
    """根据情感绘制表情"""
    screen.fill((255, 255, 255))  # 白色背景
    # 绘制脸部轮廓（黄色圆形）
    pygame.draw.circle(screen, (255, 255, 0), (200, 200), 100)  # 位置(200,200), 半径100
    # 绘制眼睛（黑色圆形）
    pygame.draw.circle(screen, (0, 0, 0), (160, 170), 15)
    pygame.draw.circle(screen, (0, 0, 0), (240, 170), 15)
    
    # 根据情感绘制嘴巴
    if emotion == "happy":
        # 笑脸：向上弯曲的弧线
        pygame.draw.arc(screen, (0, 0, 0), (150, 180, 100, 50), 3.14, 6.28, 5)  # 参数：矩形区域、起始角、终止角、线宽
    elif emotion == "sad":
        # 哭脸：向下弯曲的弧线 + 眼泪
        pygame.draw.arc(screen, (0, 0, 0), (150, 220, 100, 50), 0, 3.14, 5)        
        pygame.draw.ellipse(screen, (0, 0, 255), (135, 200, 20, 30))  # 左眼下方的眼泪
        pygame.draw.ellipse(screen, (0, 0, 255), (245, 200, 20, 30))  # 右眼下方的眼泪
    pygame.display.flip()  # 更新屏幕

def main():
    rclpy.init()
    subscriber = EmotionSubscriber()
    subscriber.run()

if __name__ == '__main__':
    main()
