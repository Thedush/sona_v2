#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
# from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import serial 
import time 



class AvrNode(Node):
    def __init__(self):
        super().__init__('avr_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'battery', 10)
        self.subscription = self.create_subscription(Twist, '/onix_velocity_controller/cmd_vel_unstamped', self.cmd_vel_callback, 10)
        self.cmd_vel_msg = Twist()
        self.battery_percent = 100.0  # Initial battery percentage
        self.battery_current = 50.0
        self.battery_voltage = 48.0
        self.battery_status = 1.0
        self.value = ['0.00','0.00','0.00','0.0']
        timer_period = 5.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_battery_info)
        self.arduino = serial.Serial(port='/dev/ttyarduino', baudrate=9600, timeout=.1) 
        self.inmotion = False

    def write_read(self,x): 
        self.arduino.write(bytes(x, 'utf-8')) 
        time.sleep(1) 
        getData=self.arduino.readline()
        dataString = getData.decode('utf-8')
        data=dataString[0:][:-2]
        readings = data.split(",")
        #print(x)
        # data = self.arduino.readline() 
        if len(readings) < 4:
            #print(self.value)
            return self.value
        else:
            #print(readings)
            return readings 
    
    def cmd_vel_callback(self, msg):
        # Handle cmd_vel messages here
        self.cmd_vel_msg = msg
        if msg.linear.x > 0 or msg.angular.z > 0:
            self.inmotion = True
        else: 
            self.inmotion = False
            
        if self.inmotion :
            self.value = self.write_read('1001\n') 
        if not self.inmotion:
            if self.battery_percent > 70: # green
                self.value = self.write_read('1002\n') 
            if self.battery_percent > 30 and self.battery_percent < 70: # yellow
                self.value = self.write_read('1003\n') 
            if self.battery_percent < 30 : # red color
                self.value = self.write_read('1004\n')
            if self.battery_status >= 0 : # red color
                self.value = self.write_read('1005\n')   

            
        #print(value) # printing the value 

    def publish_battery_info(self):
        # Calculate battery info (replace this with your logic)
        # For demonstration purposes, we decrease the battery percentage by 1% every cycle
        self.battery_voltage = float(self.value[0])
        self.battery_percent = float(self.value[1])
        self.battery_current = float(self.value[2])
        self.battery_status = float(self.value[3])

        battery_msg = Float32MultiArray()
        battery_msg.data = [self.battery_voltage,self.battery_percent,self.battery_current,self.battery_status]
        self.publisher_.publish(battery_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AvrNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
