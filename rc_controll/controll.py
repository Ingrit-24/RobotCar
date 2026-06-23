import rclpy
import numpy as np
import csv
import os
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point
from std_msgs.msg import Float64

class Controll(Node):
    def __init__(self):
        super().__init__('Controll_Node')

        self.KP = 0.225
        self.KI = 0.005
        self.Vg = 0.7
        
        
        self.inte = 0
        self.rad25 = np.deg2rad(25)
        
        self.robo_pos = np.zeros(3) #enu
        self.robo_vel = np.zeros(3)
        self.delta_t = 0
        
        self.sub_pos = self.create_subscription(Point,'/pos_enu',self.update_pos,10)
        self.sub_vel = self.create_subscription(Point,'/vel_enu',self.update_vel,10)
        self.sub_dt = self.create_subscription(Float64,'/Kalman_out_dt',self.update_dt,10)
        self.pub = self.create_publisher(AckermannDriveStamped,'/ackermann_cmd',10)
        
        self.renew_flag = 0
        
        package_share_dir = get_package_share_directory('rc_controll')
        csv_path = os.path.join(package_share_dir, 'data/advance.csv')
        self.route = self.get_route(csv_path)
        self.route_max = len(self.route)
        
        self.theta = np.arctan2(self.route[1,1]-self.route[0,1],self.route[1,0]-self.route[0,0])
        out = AckermannDriveStamped()
        out.drive.steering_angle = 0.0
        out.drive.speed = 1.5
        self.pub.publish(out)
        
        self.timer = self.create_timer(0.01, self.controll)
        self.get_logger().info(f"route shape: {self.route.shape}, robo_pos shape: {self.robo_pos.shape}")

    def get_route(self,filepass):
        with open(f'{filepass}','r',encoding='utf-8') as f:
            reader = csv.reader(f)
            list = [[float(val) for val in row] for row in reader]
        return np.array(list)
    
    def update_pos(self,msg):
        self.robo_pos[0] = msg.x
        self.robo_pos[1] = msg.y
        self.robo_pos[2] = msg.z
        self.renew_flag = 1
        self.get_logger().info(f"{self.robo_pos}")
        return
    
    def update_vel(self,msg):
        self.robo_vel[0] = msg.x
        self.robo_vel[1] = msg.y
        self.robo_vel[2] = msg.z
        self.renew_flag = 1
        return
    
    def update_dt(self,msg):
        self.delta_t = msg.data
    
    def controll(self):
        if self.renew_flag == 0:
            return 
        
        self.renew_flag = 0
        
        norms = np.linalg.norm(self.route[:,:2] - self.robo_pos[:2],axis=1)
        minidx = np.argmin(norms)
        
        if minidx > self.route_max - 4:
            goalidx = 3-(self.route_max - minidx)
        else:
            goalidx = minidx + 3
        
        routex = self.route[goalidx] - self.robo_pos 
        goaltheta = np.arctan2(routex[1],routex[0])
        self.theta = np.arctan2(self.robo_vel[1],self.robo_vel[0])
        
        ds = np.arctan2(np.sin(goaltheta - self.theta), np.cos(goaltheta - self.theta))
        self.get_logger().info(f'Nowind:{minidx} Now:{np.rad2deg(self.theta)} Goal:{np.rad2deg(goaltheta)}')
        self.get_logger().info(f'ds{np.rad2deg(ds)}')
         
        self.inte += ds * self.delta_t
        output_s = ds * self.KP + self.inte * self.KI 
        
        
        if output_s > self.rad25:
            output_s = self.rad25
        if output_s < -self.rad25:
            output_s = -self.rad25
        
        v = np.linalg.norm([self.robo_vel[0],self.robo_vel[1]])
        
        
        out = AckermannDriveStamped()
        if v < 0.40:
            out.drive.steering_angle = 0.0
        else:
            out.drive.steering_angle = output_s
        out.drive.speed = self.Vg
        self.pub.publish(out)
        self.get_logger().info(f'OUT:{out.drive.steering_angle}')
    
        
def main(args=None):
    rclpy.init(args=args)
    node = Controll()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
