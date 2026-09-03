#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster

class MiRController(Node):
    def __init__(self):
        super().__init__('mir_controller')

        # Robot parameters from URDF
        self.wheel_radius = 0.0625
        self.wheel_separation = 0.445208

        # Odometry variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # Wheel state variables
        self.left_wheel_vel = 0.0
        self.right_wheel_vel = 0.0
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0

        # Subscriptions
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)

        # Publishers
        # Using a Float64MultiArray to publish to a standard JointGroupVelocityController
        self.vel_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/velocity_controller/commands',
            10)
        
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10)

        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10)

        # Transform Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer to publish odometry and tf (e.g. 50Hz)
        self.odom_timer = self.create_timer(0.02, self.publish_odometry)

        self.get_logger().info('Custom MiR Controller Node Started.')

    def cmd_vel_callback(self, msg):
        """
        Inverse Kinematics: Convert linear and angular velocity to wheel velocities.
        """
        v = msg.linear.x
        omega = msg.angular.z

        # Calculate wheel velocities (m/s)
        v_left = v - (omega * self.wheel_separation / 2.0)
        v_right = v + (omega * self.wheel_separation / 2.0)

        # Convert to angular velocities (rad/s)
        omega_left = v_left / self.wheel_radius
        omega_right = v_right / self.wheel_radius

        # Set target velocities for integration
        self.left_wheel_vel = omega_left
        self.right_wheel_vel = omega_right

        # Publish the command to the velocity controller
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [omega_left, omega_right]
        self.vel_cmd_pub.publish(cmd_msg)

    def joint_state_callback(self, msg):
        """
        Update current wheel velocities from joint_states.
        """
        try:
            left_idx = msg.name.index('left_wheel_joint')
            right_idx = msg.name.index('right_wheel_joint')
            
            # Using actual velocity from simulation/hardware if available
            # If velocity isn't published by some systems, you might need to derive it from position
            if len(msg.velocity) > max(left_idx, right_idx):
                self.left_wheel_vel = msg.velocity[left_idx]
                self.right_wheel_vel = msg.velocity[right_idx]
        except ValueError:
            # joint not found in this message
            pass

    def publish_odometry(self):
        """
        Forward Kinematics: Compute odometry from wheel velocities and publish.
        """
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Calculate linear velocities of the wheels (m/s)
        v_l = self.left_wheel_vel * self.wheel_radius
        v_r = self.right_wheel_vel * self.wheel_radius

        # Calculate robot's linear and angular velocity
        v = (v_r + v_l) / 2.0
        omega = (v_r - v_l) / self.wheel_separation

        # Integrate to get pose
        delta_theta = omega * dt
        self.theta += delta_theta
        
        # We can use the average angle for a better approximation of the movement
        avg_theta = self.theta - delta_theta / 2.0
        
        self.x += v * math.cos(avg_theta) * dt
        self.y += v * math.sin(avg_theta) * dt

        # Create quaternion from yaw
        q = self.quaternion_from_euler(0, 0, self.theta)

        # 1. Publish Transform
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        # 2. Publish Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega

        self.odom_pub.publish(odom)

        # 3. Publish JointStates to spin the wheels in RViz
        self.left_wheel_pos += self.left_wheel_vel * dt
        self.right_wheel_pos += self.right_wheel_vel * dt
        
        joint_msg = JointState()
        joint_msg.header.stamp = current_time.to_msg()
        joint_msg.name = ['left_wheel_joint', 'right_wheel_joint', 
                          'fl_caster_rotation_joint', 'fl_caster_wheel_joint',
                          'fr_caster_rotation_joint', 'fr_caster_wheel_joint',
                          'bl_caster_rotation_joint', 'bl_caster_wheel_joint',
                          'br_caster_rotation_joint', 'br_caster_wheel_joint']
        joint_msg.position = [self.left_wheel_pos, self.right_wheel_pos, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        joint_msg.velocity = [self.left_wheel_vel, self.right_wheel_vel, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_state_pub.publish(joint_msg)

    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion (w in last place)
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = sr * cp * cy - cr * sp * sy # x
        q[1] = cr * sp * cy + sr * cp * sy # y
        q[2] = cr * cp * sy - sr * sp * cy # z
        q[3] = cr * cp * cy + sr * sp * sy # w
        return q

def main(args=None):
    rclpy.init(args=args)
    node = MiRController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
