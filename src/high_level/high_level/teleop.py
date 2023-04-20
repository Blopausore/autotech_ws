#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from getkey import getkey, keys
from covaps.msg import Order


HIGH_LIMIT_SPEED = 255
LOW_LIMIT_SPEED = 0
HIGH_LIMIT_ANG = 90
LOW_LIMIT_ANG = 0

SPEED_STEP = 1
ANGLE_STEP = 10

class TeleopNode(Node):
    linearSpeed = 100
    angularPos = 45

    def __init__(self):
        super().__init__("Teleoperator_node")
        self.get_logger().info("Teleoperator node started")
        self.get_logger().info("press UP to speed up")
        self.get_logger().info("press DOWN to slow down")
        self.get_logger().info("press LEFT to turn left")
        self.get_logger().info("press RIGHT to turn right")
        self.get_logger().info("press I to get information on the car")
        self.get_logger().info("press S to stop the car")
        self.get_logger().info("press Q to quit")
        self.initial_state()
        self.cmd_car = self.create_publisher(Order, "/ai/cmd_car", 10)

    def initial_state(self):
        self.linearSpeed = 228
        self.angularPos = 45
        

    def sendOrder(self, value, type):
        order = Order()
        order.val = value
        order.type = type
        self.cmd_car.publish(order)

    def readKey(self):
        res = True
        key = getkey()

        #Speed control
        if (key == keys.UP):
            self.get_logger().info("speed up : {}".format(self.linearSpeed + SPEED_STEP))
            self.linearSpeed += SPEED_STEP
            while (self.linearSpeed >= HIGH_LIMIT_SPEED) :
                self.get_logger().info("reached limit")
                self.linearSpeed -= SPEED_STEP
            self.sendOrder(self.linearSpeed, "speed")
        elif (key == keys.DOWN):
            self.get_logger().info("slow down : {}".format(self.linearSpeed - SPEED_STEP))
            self.linearSpeed -= SPEED_STEP
            while (self.linearSpeed <= LOW_LIMIT_SPEED) :
                self.get_logger().info("reached limit")
                self.linearSpeed += SPEED_STEP
            self.sendOrder(self.linearSpeed, "speed")

        #Angle control
        elif (key == keys.RIGHT):
            self.get_logger().info("turn left : {}".format(self.angularPos + ANGLE_STEP))
            self.angularPos += ANGLE_STEP
            while self.angularPos >= HIGH_LIMIT_ANG :
                self.get_logger().info("reached limit")
                self.angularPos -= ANGLE_STEP
            self.sendOrder(self.angularPos,"angular")
        elif (key == keys.LEFT):
            self.get_logger().info("turn right : {}".format(self.angularPos - ANGLE_STEP))
            self.angularPos -= ANGLE_STEP
            while self.angularPos <= LOW_LIMIT_ANG :
                self.get_logger().info("reached limit")
                self.angularPos += ANGLE_STEP
            self.sendOrder(self.angularPos, "angular")
            
        elif (key == 'I'):
            self.get_logger().info("linear speed:" + str(self.linearSpeed))
            self.get_logger().info("angular position:" + str(self.angularPos))

        elif (key == 'S'):
            self.get_logger().info("car stopped")
            self.initial_state()
            self.sendOrder(self.linearSpeed, "speed")
            self.sendOrder(self.angularPos, "angular")

        elif (key == 'Q'):
            self.get_logger().info("quit teleop")
            self.initial_state()
            self.sendOrder(self.linearSpeed, "speed")
            self.sendOrder(self.angularPos, "angular")
            res = False    
        else :
            self.get_logger().info("invalid key used")
        return res

    def main(self):
        cntn = True
        while cntn :
            cntn = self.readKey()

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    node.main()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
