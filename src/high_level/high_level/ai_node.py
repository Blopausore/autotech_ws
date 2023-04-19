##
#%%
import numpy
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
import os

from std_msgs.msg import Float32MultiArray
from stable_baselines3 import PPO

from covaps.msg import Order
from high_level.personal_tools import linear_transformation

##
#%%

model_scale = [-1, 1]
com_scale = [-255, 255]

speed_limit = [-50, 125]
angle_limit = [-200, 200]

derivative_coefficient = 0.01
##
#%%

print(os.getcwd())
models_path = "./src/"
model_name = "model"

class AI(Node):
    number_laser_points = 1081
    model = PPO.load(models_path + model_name)#mettre le nom du dossier qui contient le modèle

    def __init__(self, **kargs):
        super().__init__("ai_node")

        self.laser_scan = Odometry()
        self.occupancy = OccupancyGrid()
        self.odometry = Odometry()

        ## PUBLISHER

        self.cmd_vel_publisher = self.create_publisher(
                Twist, "ai_node/topic/twist", 10
        )

        ## SUBSCRIBER
        self.laser_scan_subscriber = self.create_subscription(
            Odometry, "localisation/topic/odometry",self.callback_laser_scan,10
        )
        self.occupancy_grid_subscriber = self.create_subscription(
            OccupancyGrid, "map_server/topic/occupancy_grid",self.callback_occupancy,10
        )
        self.odometry_subscriber = self.create_subscription(
            Odometry, "odometry/topic/odometry",self.callback_odometry,10
        )
        self.get_logger().info("AI abstract node has been started")

    def callback_laser_scan(self,odometry : Odometry):
        self.laser_scan = odometry

    def callback_occupancy(self,occupancy : OccupancyGrid):
        self.occupancy = occupancy

    def callback_odometry(self,odometry : Odometry):
        self.odometry = odometry


##
#%%

class AINode(AI):
    linear_speed = 0
    angular_speed = 0
    derivative_speed = 0

    def __init__(self, **kargs):
        super().__init__(**kargs)

        # Publisher
        self.cmd_car = self.create_publisher(Order, "/ai/cmd_car", 10)
        # Subscriber
        self.sub_car = self.create_subscription(
                Float32MultiArray, "/to_ai", self.callback_pub, 10
        )
        
        # End initialize
        self.get_logger().info("AI node has been started")
        

    def angle_rescale(self, x, coefficient=1):
        '''Make a linear transformation to put x who was in the scale model_scale to a y in com_scale'''
        return linear_transformation(
            x,
            model_scale,
            [com_scale[0]*coefficient, com_scale[1]*coefficient]
        )
        

    def put_in_scale(self, x, limits):
        '''Make sure that x is in the model scale'''
        return max(
            limits[0],
            min(
                limits[1],
                x
            )
        )

    def callback_pub(self, array : Float32MultiArray):
        
        data = wrapperDupauvre(array.data) 
        action, _ = self.model.predict(
            data,
            deterministic=True
        )
        
        self.linear_speed = self.put_in_scale(
            self.angle_rescale(numpy.float32(action[0]), derivative_coefficient) + self.linear_speed,
            speed_limit
        )
 
        self.angular_speed = self.put_in_scale(
            self.angle_rescale(numpy.float32(action[1])),
            angle_limit
        )
        
        order_angular = create_order(
            "angular",
            self.angular_speed
        )

        self.cmd_car.publish(order_angular)

        order_linear = create_order(
            "speed",
            self.linear_speed
        )
        self.cmd_car.publish(order_linear)
        
        self.get_logger().info(
            "linear | angular :  {} | {}   scaling : {} | {}".format(
                str(round(action[0],3)), str(round(action[1], 3)), str(self.linear_speed), str(self.angular_speed))
        )
    

##
#%%



        
def create_order(type_, val_):
    order = Order()
    order.val = int(val_)
    order.type = type_
    return order

def wrapperDupauvre(listfloat) :
    returnedList = []
    for val in listfloat :
        returnedList.append([val])
    return returnedList

def main(args=None):

    rclpy.init(args=args)
    node = AINode()
    rclpy.spin(node)
    rclpy.shutdown()


