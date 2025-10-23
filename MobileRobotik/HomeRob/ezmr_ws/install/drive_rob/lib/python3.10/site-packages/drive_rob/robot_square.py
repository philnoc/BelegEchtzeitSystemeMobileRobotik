import rclpy 
from rclpy.node import Node 
from geometry_msgs.msg import Twist 
class TurtleSquare(Node): 
	def __init__(self): 
		super().__init__('square') 
		self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10) 
		self.timer_period = 0.1 # 10 Hz 
		self.timer = self.create_timer(self.timer_period, self.move) 
		self.state = 'straight' 
		self.step_count = 0  
		self.loopcount = 0
		self.turn_duration = int(1.8 / self.timer_period) 
		self.straight_duration = int(2.5 / self.timer_period)
	def move(self): 
		msg = Twist() 
		if self.state == 'straight': 
			if self.loopcount==4:
				self.state='finish'
			elif self.step_count < self.straight_duration: 
				msg.linear.x = 1.0 
				msg.angular.z = 0.0 
				self.step_count += 1 
			else:  
				self.state = 'turn' 
				self.step_count = 0 
		elif self.state == 'turn': 
			if self.step_count < self.turn_duration: 
				msg.linear.x = 0.0 
				msg.angular.z = 1.0 
				self.step_count += 1 
			else:  
				self.state = 'straight' 
				self.step_count=0
				self.loopcount+=1
		else:
				self.step_count = 0
				msg.linear.x = 0.0
				msg.angular.z = 0.0
				self.publisher.publish(msg)
				self.destroy_timer(self.timer)
				self.destroy_node()
				return 
		self.publisher.publish(msg) 
		
def main(args=None): 
	rclpy.init(args=args) 
	node = TurtleSquare() 
	rclpy.spin(node) 
	rclpy.shutdown() 
	
if __name__ == '__main__': 
	main()
