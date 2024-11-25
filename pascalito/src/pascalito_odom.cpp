#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <chrono>
#include <array>

class OdomPublisherNode : public rclcpp::Node {
public:
    bool debug;
    OdomPublisherNode() : Node("odom_publisher_node"), x_(0.0), y_(0.0), theta_(0.0) {
        // Inicializar el suscriptor y el publicador
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "vel_raw", 10, std::bind(&OdomPublisherNode::cmdVelCallback, this, std::placeholders::_1));
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        vel_raw_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/vel_raw", 10);

        // Suscriptor al tópico /outputs (velocidades de los motores)
        outputs_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/outputs", 10, std::bind(&OdomPublisherNode::outputsCallback, this, std::placeholders::_1));

        // Inicializar el TransformBroadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Configurar un timer para la publicación de odometría a intervalos constantes
        odom_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&OdomPublisherNode::publishOdometry, this));
        last_time_ = this->now();
        debug=false;
    }
private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Guardar velocidades lineal y angular
        linear_velocity_ = msg->linear.x;
        angular_velocity_ = msg->angular.z;
    }

    void outputsCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        // Verificar que el mensaje tenga 4 elementos (velocidades de los motores)
        if (msg->data.size() != 4) {
            RCLCPP_ERROR(this->get_logger(), "Se esperaban 4 valores de velocidad de motor");
            return;
        }

        // Convertir los datos del mensaje a un array de tipo `double`
        std::array<double, 4> omega;
        omega[0] = msg->data[0] ;  // *2 Multiplicar por 2.0 como en tu código original de Python
        omega[1] = msg->data[2] ;
        omega[2] = msg->data[1] ;
        omega[3] = msg->data[3] ;


        // Calcular la velocidad del robot usando la cinemática directa
        std::array<double, 3> q_dot = get_robot_speed(omega);

        // Publicar el mensaje Twist con la velocidad calculada
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = q_dot[0];
        twist_msg.linear.y = q_dot[1];
        twist_msg.angular.z = q_dot[2];

        vel_raw_pub_->publish(twist_msg);
        if(debug){
            RCLCPP_INFO(this->get_logger(), "Publicado cmd_vel_feedback: [vx: %f, vy: %f, wz: %f]", 
                    q_dot[0], q_dot[1], q_dot[2]);
        }
        
        
        //Actualizar las velocidades lineal y angular para la odometría
        linear_velocity_ = q_dot[0];
        angular_velocity_ = q_dot[2];
    }

    std::array<double, 3> get_robot_speed(const std::array<double, 4>& vel_motors) {
        // Constantes del robot
        double wheel_radius = 0.04;  // Radio de las ruedas (m)
        double lx = 0.09;            // Distancia en el eje X (m)
        double ly = 0.096;           // Distancia en el eje Y (m)
        double d = lx + ly;

        // Matriz Jacobiana
        std::array<std::array<double, 4>, 3> J = {{
            { wheel_radius / 4, wheel_radius / 4, wheel_radius / 4, wheel_radius / 4 },
            { -wheel_radius / 4, wheel_radius / 4, wheel_radius / 4, -wheel_radius / 4 },
            { -wheel_radius / (4 * d), wheel_radius / (4 * d), -wheel_radius / (4 * d), wheel_radius / (4 * d) }
        }};

        // Multiplicación de la matriz J por el vector de velocidades de las ruedas (vel_motors)
        std::array<double, 3> q_dot = {0.0, 0.0, 0.0};
        for (size_t i = 0; i < 3; ++i) {
            for (size_t j = 0; j < 4; ++j) {
                q_dot[i] += J[i][j] * vel_motors[j];
            }
        }

        return q_dot;
    }

void publishOdometry() {
    // Obtener el tiempo actual y calcular el delta de tiempo
    auto current_time = this->now();
    double dt = (current_time - last_time_).seconds();

    // Actualizar la posición del robot (modelo de movimiento diferencial)
    double delta_x = linear_velocity_ * cos(theta_) * dt;
    double delta_y = linear_velocity_ * sin(theta_) * dt;
    double delta_theta = angular_velocity_ * dt;

    x_ += delta_x;
    y_ += delta_y;
    theta_ += delta_theta;

    // Publicar la odometría
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_footprint";

    // Posición
    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.z = sin(theta_ / 2.0);
    odom_msg.pose.pose.orientation.w = cos(theta_ / 2.0);

    // Configurar la covarianza de la pose
    odom_msg.pose.covariance = {
        0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.1
    };

    // Velocidad
    odom_msg.twist.twist.linear.x = linear_velocity_;
    odom_msg.twist.twist.angular.z = angular_velocity_;
    odom_msg.twist.covariance = odom_msg.pose.covariance;

    odom_pub_->publish(odom_msg);

    // Publicar la transformación entre `odom` y `base_footprint`
    geometry_msgs::msg::TransformStamped odom_transform;
    odom_transform.header.stamp = current_time;
    odom_transform.header.frame_id = "odom";
    odom_transform.child_frame_id = "base_footprint";

    odom_transform.transform.translation.x = x_;
    odom_transform.transform.translation.y = y_;
    odom_transform.transform.translation.z = 0.0;
    odom_transform.transform.rotation = odom_msg.pose.pose.orientation;

    tf_broadcaster_->sendTransform(odom_transform);

    last_time_ = current_time;
}


    // Variables de posición y orientación
    double x_, y_, theta_;
    double linear_velocity_, angular_velocity_;

    // ROS 2 elementos
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr outputs_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_raw_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr odom_timer_;
    rclcpp::Time last_time_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
