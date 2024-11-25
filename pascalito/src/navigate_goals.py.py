import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.duration import Duration

class GoalNavigator(Node):
    def __init__(self):
        super().__init__('goal_navigator')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, pose):
        self.get_logger().info(f'Enviando meta: x={pose.pose.position.x}, y={pose.pose.position.y}')
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('La meta fue rechazada.')
            rclpy.shutdown()
            return

        self.get_logger().info('Meta aceptada, esperando el resultado...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        status = future.result().status

        if status == 4:  # Estado de la acción: ABORTADO
            self.get_logger().error('La navegación fue abortada. Deteniendo...')
            rclpy.shutdown()
        elif status == 3:  # Estado de la acción: EXITOSO
            self.get_logger().info('Meta alcanzada con éxito.')

        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    navigator = GoalNavigator()

    # Lista de metas
    goals = [
        {'x': -0.213, 'y': -0.547, 'z': 0.0, 'qz': -0.967331, 'qw': 0.253516},
        {'x': 1.12, 'y': -0.0509, 'z': 0.0, 'qz': 0.0, 'qw': 1.0},
        {'x': 0.284, 'y': 0.911, 'z': 0.0, 'qz': 0.7071, 'qw': 0.7071}
    ]

    goal = goals[0]
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()

    # Establecer la posición
    pose.pose.position.x = goal['x']
    pose.pose.position.y = goal['y']
    pose.pose.position.z = goal['z']

    # Establecer la orientación (cuaterniones)
    pose.pose.orientation.z = goal['qz']
    pose.pose.orientation.w = goal['qw']

    navigator.send_goal(pose)

    # Mantener el nodo activo mientras se espera el resultado


if __name__ == '__main__':
    main()
