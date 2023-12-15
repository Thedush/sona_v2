#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray,Int16
class CmdVelSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_vel_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            1  # 1 Hz
        )
        self.publisher = self.create_publisher(
            Twist,
            'nav_vel',
            10  # 10 Hz
        )
        self.rfid_subscriber = self.create_subscription(Int16MultiArray, 'zone', self.zone_callback, 10)
        self.plc_subscription = self.create_subscription(Int16, 'robot_state_topic', self.robot_state_callback, 10)
        self.timer = self.create_timer(0.1, self.publish_cmd_vel)  # 10 Hz
        self.cmd_vel = Twist()
        self.zone = [1,1]
        self.robot_state = 0

    def cmd_vel_callback(self, msg):
        self.cmd_vel = msg
    
    def robot_state_callback(self, msg):
    # a =5
        self.robot_state = msg.data
    
    def zone_callback(self,msg):
     
        self.zone = msg.data
        # print("message data",zone)

    def publish_cmd_vel(self):
        if hasattr(self, 'cmd_vel'):
            #print("message data",self.robot_state)
            if not self.zone[0] and self.robot_state:
                self.publisher.publish(self.cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    cmd_vel_subscriber = CmdVelSubscriber()
    rclpy.spin(cmd_vel_subscriber)
    cmd_vel_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
