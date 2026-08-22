"""Executable entry point for the grasp pose provider node.

Initializes the node via :mod:`grasp_pose_provider.node_initializer` and spins
it. A multi-threaded executor is used so the action callback can wait on the
GPD service response while the executor keeps servicing the node.
"""

import rclpy
from rclpy.executors import MultiThreadedExecutor

from grasp_pose_provider import node_initializer


def main(args=None):
    rclpy.init(args=args)
    node = node_initializer.create_node()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
