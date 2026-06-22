import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class CirclePublisher(Node):

    def __init__(self):
        super().__init__('circle_publisher')
        # /vel_enu トピックを geometry_msgs/msg/Point 型で作成。キューサイズは10
        self.publisher_ = self.create_publisher(Point, 'vel_enu', 10)
        
        # 0.2秒（5Hz）ごとにタイマーコールバックを呼び出す
        timer_period = 0.2  # 秒
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # 円のパラメータ設定
        self.radius = 1.0       # 円の半径
        self.omega = 1.0        # 角速度 (rad/s)
        self.time_elapsed = 0.0 # 経過時間

    def timer_callback(self):
        msg = Point()
        
        # 円軌跡の計算
        msg.x = self.radius * math.cos(self.omega * self.time_elapsed)
        msg.y = self.radius * math.sin(self.omega * self.time_elapsed)
        msg.z = 0.0  # zは常に0
        
        # メッセージの配信
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}')
        
        # 経過時間を更新（タイマー周期分進める）
        self.time_elapsed += 0.2

def main(args=None):
    rclpy.init(args=args)
    circle_publisher = CirclePublisher()
    
    try:
        rclpy.spin(circle_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        circle_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()