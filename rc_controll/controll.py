import rclpy
import numpy as np
import csv
import os
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from std_msgs.msg import Float64

class Controll(Node):
    def __init__(self):
        super().__init__('controll')

        self.KP = 1
        self.KI = 0.5
        self.inte = 0
        self.base_vel = 2.0
        
        package_share_dir = get_package_share_directory('rc_controll')
        csv_path = os.path.join(package_share_dir, 'data/route.csv')
        self.route = self.get_route(csv_path)
        
    
    def get_route(self,filepass):
        with open(f'{filepass}','r',encoding='utf-8') as f:
            reader = csv.reader(f)
            list = [row for row in reader]
        return np.array(list)
    
    
    
    
        
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
