import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import tf2_ros
from std_msgs.msg import Float32MultiArray
#from geometry_msgs.msg import PoseStamped, TransformStamped

from high_level.personal_tools import linear_transformation


##
#%%

laser_lidar_scale = [0, 64]
laser_model_scale = [0, 4]

##
#%%


def convertToAIdata2(scan_list, theta=90, number_points=18):
    angle_by_point = 360 / len(scan_list)
    opening_index_0 = int((180 - theta) / angle_by_point)
    opening_index_1 = int((180 + theta) / angle_by_point)
    # Add points and so enlarged the opening angle until we got a suitable number of points
    add_coast = True
    while not ((opening_index_1 - opening_index_0) % number_points)  == 0 :
        if add_coast :
            opening_index_0 -= 1
        else :
            opening_index_1 += 1
        add_coast = not add_coast
    step = (opening_index_1 - opening_index_0)//number_points
    ai_list = [
        linear_transformation(laser_scan, laser_lidar_scale, laser_model_scale)
        for laser_scan in scan_list[opening_index_0 : opening_index_1 : step] if laser_scan < 4
        ]
    return ai_list

def convertToAIdata(scanList) :
    length = len(scanList)
    indexStart = int(length/8)
    indexFinal = int(length*7/8)
    ListwithoutBackward = scanList[indexStart : indexFinal]
    lastList = []
    step = int(len(ListwithoutBackward)/18)
    for i in range(18):
        lastList.append(ListwithoutBackward[i*step])
    print(len(lastList))
    return lastList

class LidarToAi(Node):
    def __init__(self):
        super().__init__('laser_to_ai')
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pub = self.create_publisher(Float32MultiArray,'/covaps/toAI', 10)

    def scan_callback(self, scan_msg):
        msg = Float32MultiArray()   
        msg.data = convertToAIdata2(scan_msg.ranges)
        self.get_logger().info("{} | {}".format(str(len(msg.data)), str(msg.data[8])))
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarToAi()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == '_main_':
    main()
