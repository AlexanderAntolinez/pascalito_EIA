#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

import numpy as np
from scipy.spatial.transform import Rotation as R




class DirectKinematicsNode(Node):
    def __init__(self):
        super().__init__('direct_kinematics_node')
        self.wheel_radius = 0.04  # Radio de las ruedas (m)
        self.lx = 0.09  # Distancia en el eje X (m)
        self.ly = 0.096  # Distancia en el eje Y (m)

        self.subscription = self.create_subscription(Float32MultiArray,'/outputs',self.listener_callback,10)
        
        self.publisher = self.create_publisher(Twist, '/vel_raw', 10)


    def get_robot_speed(self, vel_motors):

        #vel_motors = (np.array(self.joint_states.position) - self.prev_pos_motors)/self.dt

        
        d = (self.lx + self.ly)
        J = (self.wheel_radius/4)*np.array([[1, 1, 1, 1],
                                        [-1, 1, 1, -1],
                                        [-1/d, 1/d, -1/d, 1/d]])

        q_dot = np.dot(J, vel_motors)

        return q_dot


    def listener_callback(self, msg):

        self.get_logger().info(f'Recibido mensaje en /outputs: {msg.data}')
        omega = msg.data
        if len(omega) != 4:
            self.get_logger().error('Se esperaban 4 valores de velocidad de motor')
            return
        
        omega = np.array(omega)*2.0
        #print(omega)
        [vx, vy, wz] = self.get_robot_speed(omega)
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = wz

        self.publisher.publish(twist)
        self.get_logger().info(f'Publicado cmd_vel_feedback: {twist}')

def main(args=None):
    rclpy.init(args=args)
    node = DirectKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
