import rclpy
import numpy as np
from cssrlib.gnss import ecef2enu , pos2ecef
from rclpy.node import Node
from geometry_msgs.msg import Point

class Ecef2enu(Node):
    def __init__ (self):
        super().__init__('Ecef2Enu_Node')

        self.sub_pos = self.create_subscription(Point,'/Kalman_out_pos',self.convert,10)
        self.sub_vel = self.create_subscription(Point,'/Kalman_out_vel',self.convert_v,10)
        
        self.pub_pos = self.create_publisher(Point,'/pos_enu',10)
        self.pub_vel = self.create_publisher(Point,'/vel_enu',10)

        self.base_llh = np.array([35.69010481,140.02119201,52.07312653])
        self.base_ecef = pos2ecef(self.base_llh,isdeg=True)

    def convert(self,msg):
        x=np.array([msg.x,msg.y,msg.z])
        
        dx = x - self.base_ecef
        enux = ecef2enu(self.base_llh,dx)
        
        msg_enu = Point()
        msg_enu.x = enux[0]
        msg_enu.y = enux[1] 
        msg_enu.z = enux[2]

        self.pub_pos.publish(msg_enu)
        self.get_logger().info(f'Pos:{enux}')
    
    def convert_v(self,msg):
        x=np.array([msg.x,msg.y,msg.z])
        
        enux = ecef2enu(self.base_llh,x)
        
        msg_enu = Point()
        msg_enu.x = enux[0]
        msg_enu.y = enux[1] 
        msg_enu.z = enux[2]

        self.pub_vel.publish(msg_enu)
        self.get_logger().info(f'Vel:{enux}')
        
def main(args=None):
    rclpy.init(args=args)
    node = Ecef2enu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
