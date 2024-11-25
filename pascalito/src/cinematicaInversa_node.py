#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

class InverseKinematicsNode(Node):
    def __init__(self):

        super().__init__('inverse_kinematics_node')
        self.wheel_radius = 0.04  # Radio de las ruedas (m)
        self.lx = 0.09  # Distancia en el eje X (m)
        self.ly = 0.096  # Distancia en el eje Y (m)
        self.subscription = self.create_subscription(Twist,'/cmd_vel',self.listener_callback,10)
        
        self.publisher = self.create_publisher(Float32MultiArray, '/set_points', 10)

        self.get_logger().info('Nodo de cinematica inversa creado')

    def listener_callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z*1.9

        # Cálculo de las velocidades de las ruedas
        factor = 1 / self.wheel_radius
        wheel_speeds = [
            factor * (vx - vy - (self.lx + self.ly) * wz),
            factor * (vx + vy - (self.lx + self.ly) * wz),
            factor * (vx + vy + (self.lx + self.ly) * wz),
            factor * (vx - vy + (self.lx + self.ly) * wz),
        ]

        output_msg = Float32MultiArray(data=wheel_speeds)
        self.publisher.publish(output_msg)

        #Imprimir mensaje en consola cuando se ponga el debug
        self.get_logger().debug(f'Publicado set_points: {output_msg.data}')


        

def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
