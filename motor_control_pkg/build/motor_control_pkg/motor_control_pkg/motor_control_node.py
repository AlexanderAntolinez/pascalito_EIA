import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from Rosmaster_Lib import Rosmaster  # Asegúrate de que el archivo de la biblioteca esté disponible en tu entorno
from rcl_interfaces.msg import SetParametersResult

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
        self.pulses_per_revolution = 90
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
        error = self.set_point - self.current_velocity  # Cálculo del error

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
        
        return self.current_velocity


class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        self.bot = Rosmaster(com="/dev/ttyUSB0", debug=True)  # Cambia "/dev/ttyUSB0" según sea necesario
        self.bot.create_receive_threading()

        # Suscriptor para los set points como un vector de flotantes
        self.setpoint_subscriber = self.create_subscription(
            Float32MultiArray,
            '/set_points',
            self.setpoint_callback,
            10
        )

        # Publicador para el feedback como un vector de flotantes
        self.output_publisher = self.create_publisher(Float32MultiArray, '/outputs', 10)
        self.timer = self.create_timer(0.025, self.publish_feedback)
        self.previous_time = self.get_clock().now()

        # Inicializar el controlador de 4 motores
        self.motors = [MotorControl(kp=5.0, ki=0.5, kd=0.0) for _ in range(4)]

        # Definir los parámetros de ROS para las ganancias del PID
        self.declare_parameter('kp', 5.0)
        self.declare_parameter('ki', 0.5)
        self.declare_parameter('kd', 0.0)
        
        # Configurar la callback para cambios en los parámetros
        self.add_on_set_parameters_callback(self.update_pid_gains)

    def update_pid_gains(self, params):
        """Callback para actualizar las ganancias PID en tiempo de ejecución."""
        for param in params:
            if param.name == 'kp':
                for motor in self.motors:
                    motor.kp = param.value
            elif param.name == 'ki':
                for motor in self.motors:
                    motor.ki = param.value
            elif param.name == 'kd':
                for motor in self.motors:
                    motor.kd = param.value

        self.get_logger().info(f"PID gains updated for all motors: kp={self.motors[0].kp}, ki={self.motors[0].ki}, kd={self.motors[0].kd}")

        # Devolver un SetParametersResult exitoso
        return SetParametersResult(successful=True)

    def setpoint_callback(self, msg):
        # Lee los valores de los set points del mensaje
        for i, motor in enumerate(self.motors):
            if i < len(msg.data):
                motor.set_point = msg.data[i]

    def publish_feedback(self):
        # Obtener datos de movimiento del robot
        timepo_debug = self.get_clock().now()
        current_time = self.get_clock().now()
        dt = (current_time - self.previous_time).nanoseconds / 1e9  # Convertir nanosegundos a segundos
        self.previous_time = current_time

        feedback_data = []
        control_outputs = []

        # Leer pulsos del encoder y aplicar control a cada motor
        for i, motor in enumerate(self.motors):
            current_pulses = self.bot.get_motor_encoder()[i]  # Obtener los pulsos del encoder del motor `i`
            filtered_velocity = motor.calculate_velocity_from_pulses(current_pulses, dt)
            control_output = motor.motor(dt)
            
            # Guardar el control output y la velocidad filtrada
            control_outputs.append(control_output)
            feedback_data.append(filtered_velocity)

            # Log de depuración para cada motor
            self.get_logger().info(f"Motor {i+1} - SP {motor.set_point} CO {control_output} Vel {filtered_velocity}")

        # Aplicar los valores de control a cada motor usando el método `set_motor`
        self.bot.set_motor(-control_outputs[0], -control_outputs[1], -control_outputs[2], -control_outputs[3])

        # Publicar el feedback de los 4 motores
        feedback_msg = Float32MultiArray()
        feedback_msg.data = feedback_data
        deltaT= (self.get_clock().now()- timepo_debug).nanoseconds/10e9
        self.get_logger().info(f"tiempo {deltaT}")
        self.output_publisher.publish(feedback_msg)

def main(args=None):
    rclpy.init(args=args)
    motor_control_node = MotorControlNode()
    rclpy.spin(motor_control_node)

    # Detener el movimiento del robot al cerrar el nodo
    motor_control_node.bot.set_car_motion(0, 0, 0, 0)
    motor_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
