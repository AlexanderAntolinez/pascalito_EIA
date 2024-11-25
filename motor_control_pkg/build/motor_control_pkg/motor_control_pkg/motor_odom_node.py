import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import numpy as np
from scipy.spatial.transform import Rotation as R
from Rosmaster_Lib import Rosmaster  # Importa la librería de Yahboom (asegúrate de usar el nombre correcto)

L_X = 0.09
L_Y = 0.096
WHEEL_RADIUS = 0.04

class OdometryPublisher(Node):

    def __init__(self):
        super().__init__('odometry_publisher')

        # Inicia la conexión con la placa de Yahboom y el hilo de recepción
        self.bot = Rosmaster(com="/dev/ttyUSB0")  # Ajusta el puerto si es necesario
        self.bot.create_receive_threading()  # Inicia el hilo de recepción de datos

        # Inicialización de variables de posición y velocidad
        self.q = np.zeros((3, 1))  # [x, y, theta]
        self.prev_pos_motors = np.zeros(4)  # Guarda la posición anterior de los encoders

        # Publicadores y transformadores
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 1)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Intervalo de tiempo para la actualización
        self.dt = 0.01
        self.create_timer(self.dt, self.odometry_callback)

    def get_robot_speed(self):
        # Obtiene los valores de los encoders directamente de la librería de Yahboom
        encoder_values = np.array(self.bot.get_motor_encoder())  # Posición de cada encoder
        # Calcula la velocidad de cada motor
        vel_motors = (encoder_values - self.prev_pos_motors) / self.dt
        self.prev_pos_motors = encoder_values  # Actualiza la posición previa

        # Matriz Jacobiana para transformar velocidades de las ruedas a velocidades del robot
        d = (L_X + L_Y)
        J = (WHEEL_RADIUS / 4) * np.array([[1, 1, 1, 1],
                                           [-1, 1, 1, -1],
                                           [-1/d, 1/d, -1/d, 1/d]])

        # Calcula q_dot aplicando la matriz de transformación
        q_dot = np.dot(J, vel_motors.reshape((4, 1)))

        return q_dot

    def odometry_callback(self):
        # Obtiene la velocidad del robot en el espacio de coordenadas del robot
        q_dot = self.get_robot_speed()
        dq = q_dot * self.dt

        # Actualiza la posición en el espacio global
        rot_m = np.array([[np.cos(self.q[2,0]), -np.sin(self.q[2,0]), 0],
                          [np.sin(self.q[2,0]), np.cos(self.q[2,0]), 0],
                          [0, 0, 1]])
        self.q = self.q + np.dot(rot_m, dq)

        ## Transformación TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.q[0,0]
        t.transform.translation.y = self.q[1,0]
        t.transform.translation.z = 0.0

        r = R.from_euler("xyz", [0, 0, self.q[2,0]])
        quat = r.as_quat()
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)

        ## Mensaje de odometría
        odometry = Odometry()
        odometry.header.stamp = self.get_clock().now().to_msg()
        odometry.header.frame_id = 'odom'
        odometry.child_frame_id = 'base_link'
        odometry.pose.pose.position.x = self.q[0,0]
        odometry.pose.pose.position.y = self.q[1,0]
        odometry.pose.pose.position.z = 0.0
        odometry.pose.pose.orientation.x = quat[0]
        odometry.pose.pose.orientation.y = quat[1]
        odometry.pose.pose.orientation.z = quat[2]
        odometry.pose.pose.orientation.w = quat[3]

        odometry.twist.twist.linear.x = q_dot[0,0]
        odometry.twist.twist.linear.y = q_dot[1,0]
        odometry.twist.twist.angular.z = q_dot[2,0]

        ## Covarianza
        odometry.pose.covariance[0] = 0.01
        odometry.pose.covariance[7] = 0.01
        odometry.pose.covariance[14] = 0.01
        odometry.pose.covariance[21] = 0.01
        odometry.pose.covariance[28] = 0.01
        odometry.pose.covariance[-1] = 0.01

        # Publica la odometría
        self.odom_publisher.publish(odometry)


def main(args=None):
    rclpy.init(args=args)
    odometry_publisher = OdometryPublisher()
    rclpy.spin(odometry_publisher)

    odometry_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
