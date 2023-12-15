import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray,Int16,Bool



'''
1 - straight 
2- Left
3 - Right 
second two digit tells the route 
for example if the rfid number is 380
then 3 -right 
80 - route id
205 -  2 (left) 05 route id 

station id ,split from rfid 
and direction also 


if rfid first digit more than 4 , then it is special id
for example 510 - 5 is first digit so it is special id we can connect to special function

Special id for rfid 
501 - Main stop
502- skip next rfid
503 - pitstop



Robot state 
0 - stop
1 - go straight
2 - left
3 - right
4 - reverse

'''

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')

        self.rfid_subscriber = self.create_subscription(
            Int16MultiArray,
            '/rfid_tape_data',
            self.rfid_callback,
            10
        )

        self.station_subscriber = self.create_subscription(
            Int16MultiArray,
            '/plc_data',
            self.station_callback,
            10
        )

        self.robot_state_publisher = self.create_publisher(
            Int16,
            'robot_state_topic',
            10
        )
        self.stop_publisher = self.create_publisher(
            Bool,'/buzzer',
            10
        )
        self.station_reset_pub = self.create_publisher(Bool, 'stop_robot', 10)
        self.robot_state = 0
        self.sub_station = 0
        self.stop_robot = 0
        self.station_id = 0
        self.rfid_data = 0
        self.rfid_station_id = 0
        self.rfid_direction = 0
        self.rfid_signal = 0
        self.move_on = 0
        self.locked_rfid  = 0
        self.changed = False
        self.locknext = False
        self.timer_period = 0.1  # 1 second
        self.timer = self.create_timer(self.timer_period, self.publish_robot_state)
        
        

    def rfid_callback(self, rfid_msg):
        # Process RFID data here
        # Example: Check RFID data and determine the robot state
        self.rfid_data = rfid_msg.data[2]
        self.rfid_station_id = self.rfid_data % 100
        self.rfid_direction =  self.rfid_data // 100
        self.rfid_signal = rfid_msg.data[1]
        

    def station_callback(self, station_msg):
        # Process station data here
        # Example: Check station data and determine the robot state
        self.station_id = station_msg.data[0]
        self.move_on  = station_msg.data[1]
        self.sub_station  = station_msg.data[2]


   

    def publish_robot_state(self):


    	if self.station_id  == 0:
    		self.robot_state = 0 # stop robot
    	else:
    		self.robot_state = 1 # Move robot


    	if self.rfid_data == 505:
    		self.locknext = True
    	if self.locknext and self.rfid_station_id == self.station_id :
    		self.locked_rfid = self.rfid_data
    		self.locknext = False

    	print("rfid_direction",self.rfid_direction ,"rfid_station_id",self.rfid_station_id,"station_id",self.station_id,"rfid_data",self.rfid_data,"locked_rfid",self.locked_rfid)
    	# print("move_on",self.move_on)
    	if self.rfid_direction < 5 and self.rfid_station_id == self.station_id and self.rfid_data != 501 and self.rfid_data != self.locked_rfid :
    		print(self.rfid_direction)
    		if self.rfid_direction == 1:
    			self.robot_state = 1 # straight
    			
    		if self.rfid_direction == 2:
    			self.robot_state = 2 # Left	

    		if self.rfid_direction == 3:
    			self.robot_state = 3 # right
    		self.changed = True
    		self.locked_rfid = 0

    	if self.changed :
    		if self.rfid_data == 501:
    			self.robot_state = 0 # stop
    			station_rst_msg = Bool()
    			station_rst_msg.data = True
    			self.station_reset_pub.publish(station_rst_msg)
    			self.station_reset_pub.publish(station_rst_msg)
    			self.changed = False
    			self.locked_rfid = 0
    		if self.rfid_data == self.sub_station and self.rfid_signal:
                  self.stop_robot = 1 
    		if self.move_on:
                	self.stop_robot = 0
                	empty_bool = Bool()
                	empty_bool.data = False
                	self.stop_publisher.publish(empty_bool)
    		if self.stop_robot :
                    self.robot_state = 0
                    empty_bool = Bool()
                    empty_bool.data = True
                    self.stop_publisher.publish(empty_bool)		
    	if self.rfid_data == 506:
    		self.robot_state = 3
    	if self.rfid_data == 507:
    		self.robot_state = 1    
    		#else:
    	         #   self.robot_state = 1
    	print("Robot State", self.robot_state, "RFID Signal", self.rfid_signal, "changed",self.changed, "rfiddata",self.rfid_data )
    	msg = Int16()
    	msg.data = self.robot_state
    	self.robot_state_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
