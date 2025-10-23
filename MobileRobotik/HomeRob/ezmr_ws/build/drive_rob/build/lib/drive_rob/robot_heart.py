import rclpy 
from rclpy.node import Node 
from geometry_msgs.msg import Twist 
class TurtleHeart(Node): 
	def __init__(self): 
		super().__init__('half_circle_and_turn') 
		self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10) 
		self.timer_period = 0.1 # 10 Hz 
		self.timer = self.create_timer(self.timer_period, self.move) 
		self.state = 'half_circle1' 
		self.step_count = 0 
		self.circle_duration = int(4.3 / self.timer_period) 
		self.turn_duration = int(3.9 / self.timer_period) 
		self.forward_duration = int(3.5 / self.timer_period) 
		self.turn2_duration = int(2.25 / self.timer_period) 
		
		self.get_logger().info("Turtle starts: half circle + turn") 
	def move(self): 
		msg = Twist() 
		if self.state == 'half_circle1': 
			if self.step_count < self.circle_duration: 
				msg.linear.x = 1.0 
				msg.angular.z = 1.0 
				self.step_count += 1 
			else: 
				self.get_logger().info("Half circle done, turning 180°") 
				self.state = 'turn1' 
				self.step_count = 0 
		elif self.state == 'turn1': 
			if self.step_count < self.turn_duration: 
				msg.linear.x = 0.0 
				msg.angular.z = 1.0 
				self.step_count += 1 
			else: 
				self.get_logger().info("Turn complete!") 
				self.state = 'half_circle2' 
				self.step_count=0 
		elif self.state == 'half_circle2': 
			if self.step_count < self.circle_duration: 
				msg.linear.x = 1.0 
				msg.angular.z = 1.0 
				self.step_count += 1 
			else: 
				self.get_logger().info("Half circle done, turning 180°") 
				self.step_count = 0 
				self.state = 'forward1' 
		elif self.state == 'forward1': 
			if self.step_count < self.forward_duration: 
				msg.linear.x = 1.0 
				msg.angular.z = 0.0 
				self.step_count += 1 
			else: 
				self.get_logger().info("forward done, turning 180°") 
				self.step_count = 0 
				self.state='turn2' 
		elif self.state == 'turn2': 
			if self.step_count < self.turn2_duration: 
				msg.linear.x = 0.0 
				msg.angular.z = 1.0 
				self.step_count += 1 
			else: 
				self.get_logger().info("Half circle done, turning 180°") 
				self.step_count = 0 
				self.state='forward2' 
		elif self.state == 'forward2': 
			if self.step_count < self.forward_duration: 
				msg.linear.x = 1.0 
				msg.angular.z = 0.0 
				self.step_count += 1 
			else: 
				self.get_logger().info("forward done") 
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
	node = TurtleHeart() 
	rclpy.spin(node) 
	rclpy.shutdown() 
	if __name__ == '__main__': 
		main()
