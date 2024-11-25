#!/usr/bin/env python
# encoding: utf-8

# public lib
import sys
import math
import threading
from math import pi
from time import sleep
from Rosmaster_Lib import Rosmaster

# ros lib
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, Bool, Float32MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, MagneticField, JointState
from rclpy.clock import Clock

class MotorControl:
    def __init__(self, kp, ki, kd):
        # Inicializar las ganancias del PID
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # Variables del PID
        self.set_point = 0.0  # Valor objetivo de velocidad
        self.control_output = 0.0  # Salida de control (ej., PWM o voltaje)
        self.previous_error = 0.0  # Error anterior
        self.integral_error = 0.0  # Error acumulado
        
        # Encoder variables
        self.previous_pulses = 0.0
        self.pulses_per_revolution = 45
        self.gear_ratio = 30
        # Velocidad actual medida (inicialmente 0)
        self.current_velocity = 0.0

        # Estado inicial del PID
        self.state = "STOP"  # Estados posibles: "STOP", "TRASITION", "RUN"

        # Parámetros del filtro de paso bajo
        self.alpha = 0.05  # Factor de suavizado
        self.filtered_rpm = 0.0  # Valor de RPM filtrado previo

        # Parámetro de arranque
        self.offset = 20

    def update_gains(self, kp, ki, kd):
        """Actualizar las ganancias del PID en tiempo de ejecución."""
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def motor(self, dt):
        """Función principal para controlar el motor dado un set point de velocidad."""

        error = self.set_point - self.current_velocity

        # Máquina de estados para el control PID
        if self.set_point == 0:

            self.state = "STOP"
            
        else:
            self.state = "RUN"

        

        if self.state == "STOP":
            self.control_output = 0.0  # Valor inicial cuando el motor está parado
        
        elif self.state == "RUN":        
            self.control_output = self.pid_control(error, dt)  # Salida de control

        return self.control_output  # Retorna la salida de control

    def pid_control(self, error, dt):
        """Función de control PID que calcula el output basado en el error."""
        # Acumulación del error (integral)
        self.integral_error += error * dt

        # Derivada del error
        derivative_error = (error - self.previous_error) / dt if dt > 0 else 0.0

        # Calcular salida PID
        control_output = (self.kp * error) + (self.ki * self.integral_error) + (self.kd * derivative_error)
    
        control_output = control_output + self.offset if control_output > 0 else control_output-self.offset

        # Actualizar el error anterior
        self.previous_error = error
        control_output = max(min(100.0, control_output), -100.0)
        return control_output

    def calculate_velocity_from_pulses(self, current_pulses, time_interval):
        """Calcula la velocidad en función de los pulsos del encoder y aplica un filtro de paso bajo."""
        # Diferencia de pulsos
        pulse_difference = current_pulses - self.previous_pulses
        self.previous_pulses = current_pulses
        # Convertir pulsos a revoluciones
        revolutions = pulse_difference / self.pulses_per_revolution / self.gear_ratio
        
        # Calcular velocidad en revoluciones por segundo (rps)
        velocity_rps = revolutions / time_interval
    
        # Convertir a radianes por segundo (rad/s)
        raw_velocity = velocity_rps * (2 * 3.14159)  # Velocidad en rad/s
    
        # Aplicar filtro de paso bajo
        self.current_velocity = self.alpha * raw_velocity + (1.0 - self.alpha) * self.filtered_rpm
        self.filtered_rpm = self.current_velocity  # Actualizar el valor filtrado previo
        
        return self.current_velocity,raw_velocity



class yahboomcar_driver(Node):
    def __init__(self, name):
        super().__init__(name)
        self.RA2DE = 180 / pi
        self.car = Rosmaster()
        self.car.create_receive_threading()
        self.car.set_car_type(1)

        # Inicializar controladores PID para cada motor
        self.motors = [
            MotorControl(kp=5.0, ki=0.5, kd=0.0),
            MotorControl(kp=5.0, ki=0.5, kd=0.0),
            MotorControl(kp=5.0, ki=0.5, kd=0.0),
            MotorControl(kp=5.0, ki=0.5, kd=0.0)
        ]
        self.motors[0].previous_pulses=self.car.get_motor_encoder()[0]
        self.motors[1].previous_pulses=self.car.get_motor_encoder()[1]
        self.motors[2].previous_pulses=self.car.get_motor_encoder()[2]
        self.motors[3].previous_pulses=self.car.get_motor_encoder()[3]
        # Obtener parámetros
        self.declare_parameter('car_type', 'X3')
        self.car_type = self.get_parameter('car_type').get_parameter_value().string_value
        print(self.car_type)

        self.declare_parameter('imu_link', 'imu_link')
        self.imu_link = self.get_parameter('imu_link').get_parameter_value().string_value
        print(self.imu_link)

        self.declare_parameter('Prefix', "")
        self.Prefix = self.get_parameter('Prefix').get_parameter_value().string_value
        print(self.Prefix)

        self.declare_parameter('xlinear_limit', 1.0)
        self.xlinear_limit = self.get_parameter('xlinear_limit').get_parameter_value().double_value
        print(self.xlinear_limit)

        self.declare_parameter('ylinear_limit', 1.0)
        self.ylinear_limit = self.get_parameter('ylinear_limit').get_parameter_value().double_value
        print(self.ylinear_limit)

        self.declare_parameter('angular_limit', 5.0)
        self.angular_limit = self.get_parameter('angular_limit').get_parameter_value().double_value
        print(self.angular_limit)

        # Crear suscriptores
        self.sub_cmd_vel = self.create_subscription(Float32MultiArray, "/set_points", self.cmd_vel_callback, 10)
        self.sub_RGBLight = self.create_subscription(Int32, "RGBLight", self.RGBLightcallback, 100)
        self.sub_BUzzer = self.create_subscription(Bool, "Buzzer", self.Buzzercallback, 100)

        # Crear publicadores
        self.output_publisher = self.create_publisher(Float32MultiArray, '/outputs', 10)
        self.EdiPublisher = self.create_publisher(Float32, "edition", 100)
        self.volPublisher = self.create_publisher(Float32, "voltage", 100)
        self.staPublisher = self.create_publisher(JointState, "joint_states", 100)
        self.velPublisher = self.create_publisher(Twist, "vel_raw", 50)
        self.imuPublisher = self.create_publisher(Imu, "/imu/data_raw", 100)
        self.magPublisher = self.create_publisher(MagneticField, "/imu/mag", 100)

        # Crear temporizador
        self.timer = self.create_timer(0.1, self.pub_data)

        # Crear un timer que se dispare cada 0.02 segundos (50 Hz)
        self.timer = self.create_timer(0.02, self.control_motors)

        # Variables iniciales
        self.edition = Float32()
        self.edition.data = 1.0

        # Variables de tiempo para el control PID
        self.previous_time = self.get_clock().now()

    def setpoint_callback(self, msg):
        # Lee los valores de los set points del mensaje
        for i, motor in enumerate(self.motors):
            if i < len(msg.data):
                motor.set_point = msg.data[i]

    # Función de callback para cmd_vel
    def cmd_vel_callback(self, msg):

        self.setpoint_callback(msg)

 
    def control_motors(self):
        # Obtener datos de movimiento del robot
        timepo_debug = self.get_clock().now()
        current_time = self.get_clock().now()
        dt = (current_time - self.previous_time).nanoseconds / 1e9  # Convertir nanosegundos a segundos
        self.previous_time = current_time

        feedback_data = []
        control_outputs = []
    

        # Leer pulsos del encoder y aplicar control a cada motor
        for i, motor in enumerate(self.motors):
            current_pulses = self.car.get_motor_encoder()[i]  # Obtener los pulsos del encoder del motor `i`
            filtered_velocity,raw_vel = motor.calculate_velocity_from_pulses(current_pulses, dt)
            control_output = motor.motor(dt)
            

            # Guardar el control output y la velocidad filtrada
            control_outputs.append(control_output)
            feedback_data.append(raw_vel)
            rpm = filtered_velocity*(30/3.141516)
            # Log de depuración para cada motor  filtered_velocity
            # self.get_logger().info(f"Motor {i+1} - SP {motor.set_point} CO {control_output} Vel {rpm}")

        # Aplicar los valores de control a cada motor usando el método `set_motor`
        self.car.set_motor(-control_outputs[0], -control_outputs[1], -control_outputs[2], -control_outputs[3])

        # Publicar el feedback de los 4 motores
        feedback_msg = Float32MultiArray()
        feedback_msg.data = feedback_data
        deltaT= (self.get_clock().now()- timepo_debug).nanoseconds/10e9
        # self.get_logger().info(f"tiempo {deltaT}")
        self.output_publisher.publish(feedback_msg)

    def RGBLightcallback(self, msg):
        if not isinstance(msg, Int32): return
        for i in range(3):
            self.car.set_colorful_effect(msg.data, 6, parm=1)

    def Buzzercallback(self, msg):
        if not isinstance(msg, Bool): return
        if msg.data:
            for i in range(3):
                self.car.set_beep(1)
        else:
            for i in range(3):
                self.car.set_beep(0)

    # Función para publicar datos como voltaje, IMU y otros
    def pub_data(self):
        time_stamp = Clock().now()
        imu = Imu()
        twist = Twist()
        battery = Float32()
        edition = Float32()
        mag = MagneticField()
        state = JointState()
        state.header.stamp = time_stamp.to_msg()
        state.header.frame_id = "joint_states"

        if len(self.Prefix) == 0:
            state.name = ["back_right_joint", "back_left_joint", "front_left_steer_joint", "front_left_wheel_joint",
                            "front_right_steer_joint", "front_right_wheel_joint"]
        else:
            state.name = [self.Prefix + "back_right_joint", self.Prefix + "back_left_joint",
                            self.Prefix + "front_left_steer_joint", self.Prefix + "front_left_wheel_joint",
                            self.Prefix + "front_right_steer_joint", self.Prefix + "front_right_wheel_joint"]

        edition.data = self.car.get_version() * 1.0
        battery.data = self.car.get_battery_voltage() * 1.0
        ax, ay, az = self.car.get_accelerometer_data()
        gx, gy, gz = self.car.get_gyroscope_data()
        mx, my, mz = self.car.get_magnetometer_data()
        mx = mx * 1.0
        my = my * 1.0
        mz = mz * 1.0







        vx, vy, angular = self.car.get_motion_data()

        # Publicar datos de IMU
        imu.header.stamp = time_stamp.to_msg()
        imu.header.frame_id = self.imu_link
        imu.linear_acceleration.x = ax * 1.0
        imu.linear_acceleration.y = ay * 1.0
        imu.linear_acceleration.z = az * 1.0
        imu.angular_velocity.x = gx * 1.0
        imu.angular_velocity.y = gy * 1.0
        imu.angular_velocity.z = gz * 1.0

        mag.header.stamp = time_stamp.to_msg()
        mag.header.frame_id = self.imu_link
        mag.magnetic_field.x = mx * 1.0
        mag.magnetic_field.y = my * 1.0
        mag.magnetic_field.z = mz * 1.0

        # Publicar velocidades actuales
        twist.linear.x = vx * 1.0
        twist.linear.y = vy * 1.0
        twist.angular.z = angular * 1.0
        #self.velPublisher.publish(twist)

        # Publicar datos
        self.imuPublisher.publish(imu)
        self.magPublisher.publish(mag)
        self.volPublisher.publish(battery)
        self.EdiPublisher.publish(edition)

def main():
    rclpy.init()
    driver = yahboomcar_driver('driver_node')
    rclpy.spin(driver)


    # Detener el movimiento del robot al cerrar el nodo
    driver.car.set_car_motion(0, 0, 0, 0)
    driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
