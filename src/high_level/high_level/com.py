#!/usr/bin/env python3
import rclpy
import serial
from rclpy.node import Node
import os
from covaps.msg import Order
#import RPi.GPIO as GPIO
from time import sleep

SPEED_LIMIT = 500




class ComNode(Node):
    def __init__(self):
        super().__init__("com_node")

        # Recommend verbose : 2 for teleop | 1 for ai
        self.verbose = 1
        self.get_logger().info("com node started")
        
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
                self.path_micro = os.path.join("/dev/",path)
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

    def changePWMSpeed(self, speedValue):
        order_type = 98
        isRight = (speedValue >= 0)
        self.defineRightOrLeft(isRight)
        arg = abs(speedValue)
        self.sendOrder(order_type, arg)
        if self.verbose == 1:
                self.get_logger().info(str(speedValue))
        if self.verbose == 2:
            self.get_logger().info("change speed " + str(arg))
            self.get_logger().info("is For: "+ str(isRight))
        
    def changePWMDir(self, angleValue):
        order_type = 97
        isFor = (angleValue >= 0)
        self.defineForOrBack(isFor)
        arg = abs(angleValue)
        self.sendOrder(order_type, arg)
        if self.verbose == 1:
            self.get_logger().info("\t" + str(angleValue))
        if self.verbose == 2:
            self.get_logger().info("change rot "+ str(arg))
            self.get_logger().info("is For: "+ str(isFor))


    def defineForOrBack(self, isFor) :
        if (isFor) :
            self.sendOrder(99, 1)
        else :
            self.sendOrder(99, 0)

    def defineRightOrLeft(self, isRight) :
        if (isRight) :
            self.sendOrder(100, 1)
        else :
            self.sendOrder(100, 0)

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



