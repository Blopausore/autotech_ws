#!/usr/bin/env python3
import rclpy
import serial
from rclpy.node import Node
import os
from covaps.msg import Order
#import RPi.GPIO as GPIO
from time import sleep

SPEED_LIMIT = 128
ANGLE_LIMIT = 90

class ComNode(Node):
    def __init__(self):
        super().__init__("com_node")

        # Recommend verbose : 2 for teleop | 1 for ai
        self.verbose = 1
        self.get_logger().info("com node started v : 1.0")
        
        self.find_path_micro()
        self.baudrate = 115200
        self.arduino = serial.Serial(self.path_micro, self.baudrate, timeout= 1)

        self.stm = serial.Serial()
        self.stm.port = self.path_micro
        self.stm.baudrate = self.baudrate

        self.cmd_recv = self.create_subscription(Order, "/ai/cmd_car", self.rcv_order, 10)


    def find_path_micro(self):
        self.get_logger().info("Try to find USB path")
        for path in os.listdir("/dev"):
            if path[:-1] == "ttyUSB":
                self.path_micro = os.path.join("/dev/", path)
                self.get_logger().info("USB path found : {}".format(self.path_micro))
                return
        self.get_logger().info("Error : USB path not found")

        
    def rcv_order(self, order: Order):
        if (order.type == "speed"):
            self.changeLinear(order.val)
        elif (order.type == "angular"):
            self.changeAngular(order.val)
        else :
            self.get_logger().warning("invalide type")


    def changeLinear(self, speedValue):
        self.changePWMSpeed(speedValue)

    def changeAngular(self, angleValue):
        self.changePWMDir(angleValue)

    '''changePWM
    Order :
        97 : angle value chanel
            -> send value [0, ANGLE_LIMIT] #currently ANGLE_LIMIT = 254
        98 : speed value chanel
            -> send value [0, SPEED_LIMIT] #currently SPEED_LIMIT = 90

        99 : speed direction
            -> send 1 for backward and 0 for forward
        
        100 : angle direction
            -> send 1 for right and 0 for left
    '''

    def changePWMSpeed(self, speedValue):
        order_type = 98
        arg = min(abs(speedValue), SPEED_LIMIT)
        self.sendOrder(order_type, arg)

        if self.verbose == 1:
                self.get_logger().info("{} : {}".format("Linear speed", str(speedValue)))

        
    def changePWMDir(self, angleValue):
        order_type = 97
        arg = min(abs(angleValue), ANGLE_LIMIT)
        self.sendOrder(order_type, arg)

        if self.verbose == 1:
            self.get_logger().info("{}{} : {}".format("\t", "Angular speed", str(angleValue)))


    def sendOrder(self, octet1 : int, octet2 : int): #octet1 and octet2 should not be more that 8 bits
        trame = int.to_bytes( (octet1 << 8 ) | octet2, 2, "big")
        self.arduino.write(trame)

def main(args=None):
    rclpy.init(args=args)
    node = ComNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()



