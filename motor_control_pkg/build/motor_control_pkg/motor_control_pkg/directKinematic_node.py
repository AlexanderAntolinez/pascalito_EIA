import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

class DirectKinematicsNode(Node):
    def __init__(self):
        super().__init__('direct_kinematics_node')
        self.wheel_radius = 0.04  # Radio de las ruedas (m)
        self.lx = 0.09  # Distancia en el eje X (m)
        self.ly = 0.096  # Distancia en el eje Y (m)

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/outputs',
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, '/vel_raw', 10)

    def listener_callback(self, msg):
        omega = msg.data
        if len(omega) != 4:
            self.get_logger().error('Se esperaban 4 valores de velocidad de motor')
            return

        # Cálculo de la velocidad del robot
        vx = (self.wheel_radius / 4) * (omega[0] + omega[1] + omega[2] + omega[3])
        vy = (self.wheel_radius / 4) * (-omega[0] + omega[1] - omega[2] + omega[3])
        wz = (self.wheel_radius / (4 * (self.lx + self.ly))) * (-omega[0] + omega[1] + omega[2] - omega[3])

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
