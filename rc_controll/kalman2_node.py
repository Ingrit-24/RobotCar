import rclpy
import numpy as np
from rclpy.node import Node

from gnss_ros_standardization.msg import GnssSolution
from geometry_msgs.msg import Point
from std_msgs.msg import Float64

class KalmanNode(Node):
    def __init__(self):
        super().__init__('Kalman_Node')

        self.subscription = self.create_subscription(GnssSolution,'/gnss/solution',self.kalmanfilter,10)
        self.pub_pos = self.create_publisher(Point,'/Kalman_out_pos',10)
        self.pub_vel = self.create_publisher(Point,'/Kalman_out_vel',10)
        self.pub_dt = self.create_publisher(Float64,'/Kalman_out_dt',10)


        self.pasttime = None
        self.Q=np.zeros((2,6,6))

        self.Q[0,0,0]=0.02
        self.Q[0,1,1]=0.02
        self.Q[0,2,2]=0.02
        self.Q[0,3,3]=0.1
        self.Q[0,4,4]=0.1
        self.Q[0,5,5]=0.1
        
        self.Q[1,0,0]=0.0002
        self.Q[1,1,1]=0.0002
        self.Q[1,2,2]=0.0002
        self.Q[1,3,3]=0.05
        self.Q[1,4,4]=0.05
        self.Q[1,5,5]=0.05

        self.P=np.eye(6)
        self.X=np.zeros(6)

        self.A = np.eye(6)
        self.R = np.eye(6)
        self.H = np.eye(6)
        self.Xm = np.zeros(6)

    def kalmanfilter(self,msg):
        current_time = self.get_clock().now()
        if self.pasttime is not None:
            dt = (current_time - self.pasttime).nanoseconds / 1e9
            self.pasttime = current_time
            if dt <= 0.0:
                dt = 0.2
        else:
            self.pasttime = current_time
            dt = 0.2
            self.X[0]=msg.pos_ecef.x
            self.X[1]=msg.pos_ecef.y
            self.X[2]=msg.pos_ecef.z
            self.X[3]=msg.vel_ecef.x
            self.X[4]=msg.vel_ecef.y
            self.X[5]=msg.vel_ecef.z
            return 


        self.A[0,3]=dt
        self.A[1,4]=dt
        self.A[2,5]=dt

        self.Xm[0] =msg.pos_ecef.x
        self.Xm[1] =msg.pos_ecef.y
        self.Xm[2] =msg.pos_ecef.z
        self.Xm[3] =msg.vel_ecef.x
        self.Xm[4] =msg.vel_ecef.y
        self.Xm[5] =msg.vel_ecef.z

        self.R[0,0]=msg.pos_cov_ecef[0]
        self.R[0,1]=msg.pos_cov_ecef[1]
        self.R[0,2]=msg.pos_cov_ecef[2]
        self.R[1,0]=msg.pos_cov_ecef[3]
        self.R[1,1]=msg.pos_cov_ecef[4]
        self.R[1,2]=msg.pos_cov_ecef[5]
        self.R[2,0]=msg.pos_cov_ecef[6]
        self.R[2,1]=msg.pos_cov_ecef[7]
        self.R[2,2]=msg.pos_cov_ecef[8]

        self.R[3,3]=msg.vel_cov_ecef[0]
        self.R[3,4]=msg.vel_cov_ecef[1]
        self.R[3,5]=msg.vel_cov_ecef[2]
        self.R[4,3]=msg.vel_cov_ecef[3]
        self.R[4,4]=msg.vel_cov_ecef[4]
        self.R[4,5]=msg.vel_cov_ecef[5]
        self.R[5,3]=msg.vel_cov_ecef[6]
        self.R[5,4]=msg.vel_cov_ecef[7]
        self.R[5,5]=msg.vel_cov_ecef[8]
        
        if msg.status == 1:
            mode = 0
        else:
            mode = 1
        
        Q_current = np.zeros((6, 6))
        Q_current[0, 0] = self.Q[mode, 0, 0] * dt
        Q_current[1, 1] = self.Q[mode, 1, 1] * dt
        Q_current[2, 2] = self.Q[mode, 2, 2] * dt
        Q_current[3, 3] = self.Q[mode, 3, 3] * dt
        Q_current[4, 4] = self.Q[mode, 4, 4] * dt
        Q_current[5, 5] = self.Q[mode, 5, 5] * dt
        
        preX = self.A @ self.X
        self.P = self.A @ self.P @ self.A.T + Q_current
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.X = preX + K @ (self.Xm-preX)
        self.P = self.P - K @ self.H @ self.P

        pos_msg = Point()
        pos_msg.x=self.X[0]
        pos_msg.y=self.X[1]
        pos_msg.z=self.X[2]

        vel_msg = Point()
        vel_msg.x=self.X[3]
        vel_msg.y=self.X[4]
        vel_msg.z=self.X[5]

        dt_msg = Float64()
        dt_msg.data = float(dt)

        self.pub_pos.publish(pos_msg)
        self.pub_vel.publish(vel_msg)
        self.pub_dt.publish(dt_msg)

        self.get_logger().info(f'X:{self.X[0]} Y:{self.X[1]} Z:{self.X[2]} delta t:{dt}')
def main(args=None):
    rclpy.init(args=args)
    node = KalmanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
