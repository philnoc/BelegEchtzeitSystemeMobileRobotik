import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from irobot_create_msgs.action import Undock

class NavMapClient(Node):
	def __init__(self):
		super().__init__('undock_client')
		self._client=ActionClient(self, Undock, '/undock')
		self._send_goal_future = None
		
	def send_goal(self):
		self.get_logger().info("Warte auf Undock-Server...")
		self._client.wait_for_server()
		
		goal_msg = Undock.Goal()
		self.get_logger().info("Sende Undock Goal")
		self._send_goal_future=self._client.send_goal_async(goal_msg)
		self._send_goal_future.add_done_callback(self.goal_response_callback)
		
	def goal_response_callback(self, future):
		goal_handle = future.result()
		if not goal_handle.accepted:
			self.get_logger().warn("Undock-Goal wurde abgelehnt")
			return
			
		self.get_logger().info("Undock-Goal akzeptiert. Warte auf Ergebnis...")
		result_future = goal_handle.get_result_async()
		result_future.add_done_callback(self.result_callback)
		
	def result_callback(self, future):
		result=future.result().result
		self.get_logger().info('Undock abgeschlossen')
		
		
def main():
	rclpy.init()
	node=NavMapClient()
	node.send_goal()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	node.destroy_node()
	rclpy.shutdown()
	
if __name__=='__main__':
	main()
		
	
	

