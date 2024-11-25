import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math
import numpy as np
from scipy.spatial.transform import Rotation as R

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')

        # Variables de estado del robot
        self.q = np.zeros((3, 1))  # Estado [x, y, theta]

        # Variables para las velocidades recibidas
        self.linear_velocity_x = 0.0
        self.linear_velocity_y = 0.0
        self.angular_velocity_z = 0.0

        # Suscriptor para el tópico 'vel_raw'
        self.create_subscription(Twist, 'vel_raw', self.cmd_vel_callback, 10)

        # Publicador de odometría
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)

        # Broadcaster para la transformación entre 'odom' y 'base_link'
        self.tf_broadcaster = TransformBroadcaster(self)

        # Intervalo de tiempo para la actualización de odometría
        self.dt = 0.01
        self.create_timer(self.dt, self.odometry_callback)

        # Último tiempo para calcular el delta
        self.last_time = self.get_clock().now()

    def cmd_vel_callback(self, msg):
        # Guardar las velocidades recibidas del tópico 'vel_raw'
        self.linear_velocity_x = msg.linear.x
        self.linear_velocity_y = msg.linear.y
        self.angular_velocity_z = msg.angular.z

        # Mensaje de log para verificar la recepción de datos
        self.get_logger().info(f'Recibido vel_raw - Lineal x: {self.linear_velocity_x}, y: {self.linear_velocity_y}, Angular z: {self.angular_velocity_z}')

    def odometry_callback(self):
        # Calcular el delta de tiempo
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Convertir a segundos
        self.last_time = current_time

        # Calcular el cambio en la posición y orientación
        delta_x = (self.linear_velocity_x * math.cos(self.q[2, 0]) - self.linear_velocity_y * math.sin(self.q[2, 0])) * dt
        delta_y = (self.linear_velocity_x * math.sin(self.q[2, 0]) + self.linear_velocity_y * math.cos(self.q[2, 0])) * dt
        delta_theta = self.angular_velocity_z * dt

        # Actualizar la posición y orientación
        self.q[0, 0] += delta_x
        self.q[1, 0] += delta_y
        self.q[2, 0] += delta_theta

        # Publicar la transformación TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.q[0, 0]
        t.transform.translation.y = self.q[1, 0]
        t.transform.translation.z = 0.0

        # Calcular cuaterniones para la orientación
        r = R.from_euler("z", self.q[2, 0])
        quat = r.as_quat()
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)

        # Publicar el mensaje de odometría
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.pose.pose.position.x = self.q[0, 0]
        odom_msg.pose.pose.position.y = self.q[1, 0]
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        # Velocidades
        odom_msg.twist.twist.linear.x = self.linear_velocity_x
        odom_msg.twist.twist.linear.y = self.linear_velocity_y
        odom_msg.twist.twist.angular.z = self.angular_velocity_z

        # Matriz de covarianza
        odom_msg.pose.covariance[0] = 0.01  # Variancia en x
        odom_msg.pose.covariance[7] = 0.01  # Variancia en y
        odom_msg.pose.covariance[35] = 0.01  # Variancia en yaw

        self.odom_publisher.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    odometry_publisher = OdometryPublisher()
    rclpy.spin(odometry_publisher)
    odometry_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
