import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Pose2D, TransformStamped
from std_msgs.msg import Float32, Int32
from nav_msgs.msg import Odometry
import tf2_ros

import math
class Robot(Node):
	def __init__(self):
		super().__init__('robot_core_node')
		self.wheel_radius = 0.0375 #m
		self.wheelbase = 0.187 #m
		self.wheeltrack = 0.208 #m
		self.wheel_circum = 2*3.14159 * self.wheel_radius
		
		self.max_rpm = 80.0 #rpm
		
		self.vel_sub = self.create_subscription(Twist, 'cmd_vel', self.vel_callback,10)
		
		self.fl_pub = self.create_publisher(Float32, 'wheel_command_fl', 10)
		self.fr_pub = self.create_publisher(Float32, 'wheel_command_fr', 10)
		self.bl_pub = self.create_publisher(Float32, 'wheel_command_bl', 10)
		self.br_pub = self.create_publisher(Float32, 'wheel_command_br', 10)

		self.fl_speed_sub = self.create_subscription(Float32, "fl_actual_speed", self.fl_sp_callback,10)
		self.fr_speed_sub = self.create_subscription(Float32, "fr_actual_speed", self.fr_sp_callback,10)
		self.bl_speed_sub = self.create_subscription(Float32, "bl_actual_speed", self.bl_sp_callback,10)
		self.br_speed_sub = self.create_subscription(Float32, "br_actual_speed", self.br_sp_callback,10)
		
		
		self.vx = 0.0
		self.vy = 0.0
		self.w = 0.0
		
		self.timer = self.create_timer(0.04,self.timer_callback)
		
		self.cmd_fl = Float32()
		self.cmd_fr = Float32()
		self.cmd_bl = Float32()
		self.cmd_br = Float32()

	

		self.last_time = self.get_clock().now()

		#Odometry
		self.tick_per_rev = 1080*4
		self.robot_pose = Pose2D() #x y theta
		self.dx = 0.0
		self.dy = 0.0
		self.dtheta = 0.0

		self.odom_pub = self.create_publisher(Odometry, 'odom/raw', 10)

		#TF
		self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
		
		
		self.fl_sp = 0.0
		self.fr_sp = 0.0
		self.bl_sp = 0.0
		self.br_sp = 0.0
		
		self.get_logger().info("robot core is running")



	def fl_sp_callback(self, msg):
		self.fl_sp = msg.data
	
	def fr_sp_callback(self, msg):
		self.fr_sp = msg.data

	def bl_sp_callback(self, msg):
		self.bl_sp = msg.data

	def br_sp_callback(self, msg):
		self.br_sp = msg.data

	def get_speed(self):
		vx = ((self.fl_sp + self.fr_sp + self.bl_sp + self.br_sp ) / 4 ) / 60 * self.wheel_circum
		vy = ((-self.fl_sp + self.fr_sp + self.bl_sp - self.br_sp ) / 4 ) / 60 * self.wheel_circum
		wz = (((-self.fl_sp + self.fr_sp - self.bl_sp + self.br_sp ) / 4 ) / 60 * self.wheel_circum ) / ( (self.wheeltrack+self.wheelbase)/2.0)
		#print(f"vx:{vx} vy:{vy} wz:{wz}")
		return vx,vy,wz



	def vel_callback(self, vel):
		#print("callback function")
		l1 = self.wheeltrack/2
		l2 = self.wheelbase/2

		self.vx = vel.linear.x
		self.vy = vel.linear.y
		self.w = vel.angular.z
		
		
		R = self.wheel_radius
		
		w_fl = 1/R * ( self.vx - self.vy - self.w*(l1+l2) )
		w_fr = 1/R * ( self.vx + self.vy + self.w*(l1+l2) )
		w_bl = 1/R * ( self.vx + self.vy - self.w*(l1+l2) )
		w_br = 1/R * ( self.vx - self.vy + self.w*(l1+l2) )
		
		self.wheel_speed_setpoint(w_fl, w_fr, w_bl, w_br)
	def timer_callback(self):
		self.fl_pub.publish(self.cmd_fl)
		self.fr_pub.publish(self.cmd_fr)
		self.bl_pub.publish(self.cmd_bl)
		self.br_pub.publish(self.cmd_br)
		self.odometry()
	
	def wheel_speed_setpoint(self,w_fl, w_fr, w_bl, w_br):
		rpm_fl = w_fl*9.549297
		rpm_fr = w_fr*9.549297  #1 rad/s = 9.549297 rpm
		rpm_bl = w_bl*9.549297
		rpm_br = w_br*9.549297
		
		rpm_fl = max(min(rpm_fl,self.max_rpm), -self.max_rpm)
		rpm_fr = max(min(rpm_fr,self.max_rpm), -self.max_rpm)
		rpm_bl = max(min(rpm_bl,self.max_rpm), -self.max_rpm)
		rpm_br = max(min(rpm_br,self.max_rpm), -self.max_rpm)
		
		if rpm_fl == 0.0:
			self.cmd_fl.data = 0.0
		if rpm_fr == 0.0:
			self.cmd_fr.data = 0.0
		if rpm_bl == 0.0:
			self.cmd_bl.data = 0.0
		if rpm_br == 0.0:
			self.cmd_br.data = 0.0
			
		self.cmd_fl.data = rpm_fl
		self.cmd_fr.data = rpm_fr
		self.cmd_bl.data = rpm_bl
		self.cmd_br.data = rpm_br	
	
	def quaternion_from_euler(self, roll, pitch, yaw):
		cy = math.cos(yaw*0.5)
		sy = math.sin(yaw*0.5)
		cp = math.cos(pitch*0.5)
		sp = math.sin(pitch*0.5)
		cr = math.cos(roll*0.5)
		sr = math.sin(roll*0.5)
		q = [0]*4
		q[0] = cy * cp * cr + sy * sp * sr
		q[1] = cy * cp * sr - sy * sp * cr
		q[2] = sy * cp * sr + cy * sp * cr
		q[3] = sy * cp * cr - cy * sp * sr
		
		return q

	def odometry(self):
		ts = self.get_clock().now()
		
		dt = ts - self.last_time
		dt = dt.nanoseconds*1e-9

		vx,vy,wz = self.get_speed()
		
		prev_robot_pose = self.robot_pose
		
		
		self.dx = (vx*math.cos(prev_robot_pose.theta) - vy * math.sin(prev_robot_pose.theta) ) * dt
		self.dy = (vx*math.sin(prev_robot_pose.theta) + vy * math.cos(prev_robot_pose.theta) ) * dt
		self.dtheta = wz * dt

		new_robot_pose = Pose2D()
		new_robot_pose.x = prev_robot_pose.x + self.dx
		new_robot_pose.y = prev_robot_pose.y + self.dy
		new_robot_pose.theta = prev_robot_pose.theta + self.dtheta

		self.robot_pose.x = new_robot_pose.x
		self.robot_pose.y = new_robot_pose.y
		self.robot_pose.theta = new_robot_pose.theta

	

		#publish odometry data
		odom = Odometry()
		odom.header.stamp = self.get_clock().now().to_msg()
		odom.header.frame_id = "odom"
		odom.child_frame_id = "base_link"
		odom.pose.pose.position.x = self.robot_pose.x
		odom.pose.pose.position.y = self.robot_pose.y
		odom.pose.pose.position.z = 0.0

		quat = self.quaternion_from_euler(0.0,0.0, self.robot_pose.theta)
		odom.pose.pose.orientation.w = quat[0]
		odom.pose.pose.orientation.x = quat[1]
		odom.pose.pose.orientation.y = quat[2]
		odom.pose.pose.orientation.z = quat[3]

		odom.pose.covariance[0] = 0.001
		odom.pose.covariance[7]  = 0.001
		odom.pose.covariance[35]  = 0.001

		self.odom_pub.publish(odom)

		t = TransformStamped()
		t.header.stamp = self.get_clock().now().to_msg()
		t.header.frame_id = "odom"
		t.child_frame_id = "base_link"
		t.transform.translation.x = self.robot_pose.x
		t.transform.translation.y = self.robot_pose.y
		t.transform.translation.z = 0.0
		t.transform.rotation.w = quat[0]
		t.transform.rotation.x = quat[1]
		t.transform.rotation.y = quat[2]
		t.transform.rotation.z = quat[3]

		#self.tf_broadcaster.sendTransform(t)
		

		self.last_time = ts


def main():
	rclpy.init()
	
	rb = Robot()
	rclpy.spin(rb)
	
	rclpy.shutdown()
	rb.destroy_node()
	
if __name__ == "__main__":
	main()
