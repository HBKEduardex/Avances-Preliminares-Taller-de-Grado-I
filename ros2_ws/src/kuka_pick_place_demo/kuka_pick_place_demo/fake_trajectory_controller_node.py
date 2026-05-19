#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
fake_trajectory_controller_node.py

Nodo que provee un Action Server falso para /joint_trajectory_controller/follow_joint_trajectory.
Recibe la trayectoria de MoveIt2 y simplemente publica los puntos a /fake_joint_states
con la temporización adecuada, permitiendo probar la ejecución ('execute:=true')
sin necesidad de ros2_control ni hardware real.
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState


class FakeTrajectoryControllerNode(Node):
    def __init__(self):
        super().__init__('fake_trajectory_controller_node')
        
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory',
            self.execute_callback
        )
        
        self._publisher = self.create_publisher(JointState, '/fake_joint_states', 10)
        self.get_logger().info('Fake trajectory controller action server is ready en /joint_trajectory_controller/follow_joint_trajectory')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Recibida petición de ejecución de trayectoria (FAKE).')
        
        trajectory = goal_handle.request.trajectory
        joint_names = trajectory.joint_names
        
        if not trajectory.points:
            self.get_logger().warn('La trayectoria está vacía.')
            goal_handle.succeed()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return result

        start_time = self.get_clock().now()
        
        for point in trajectory.points:
            # Calcular tiempo objetivo desde el inicio
            time_from_start_sec = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            
            # Esperar hasta que se alcance el tiempo objetivo (simulación simple)
            while True:
                elapsed_sec = (self.get_clock().now() - start_time).nanoseconds * 1e-9
                if elapsed_sec >= time_from_start_sec:
                    break
                time.sleep(0.01)
            
            # Publicar el estado articular
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = joint_names
            msg.position = point.positions
            self._publisher.publish(msg)
            
        self.get_logger().info('Trayectoria ejecutada exitosamente.')
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result


def main(args=None):
    rclpy.init(args=args)
    node = FakeTrajectoryControllerNode()
    
    # Usar un MultiThreadedExecutor no es estrictamente necesario aquí,
    # pero es buena práctica para Action Servers en Python
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
