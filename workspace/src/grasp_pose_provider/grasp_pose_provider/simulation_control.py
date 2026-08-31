"""Helpers for pausing and resuming the synchronized physics loop."""

import time

import rclpy
from std_srvs.srv import Trigger


PHYSICS_SYNC_NODE_NAME = 'physics_sync_node'
PAUSE_SIMULATION_SERVICE = (
    f'/{PHYSICS_SYNC_NODE_NAME}/pause_simulation'
)
RESUME_SIMULATION_SERVICE = (
    f'/{PHYSICS_SYNC_NODE_NAME}/resume_simulation'
)


def pause_simulation(node, parameters):
    """Pause physics advancement through the physics-sync node.

    ``parameters`` is the node's
    :class:`~grasp_pose_provider.node_parameters.GraspPoseProviderParameters`;
    the service timeout comes from it.
    """
    _call_trigger_service(
        node,
        PAUSE_SIMULATION_SERVICE,
        parameters.simulation_service_timeout_sec,
    )


def resume_simulation(node, parameters):
    """Resume physics advancement through the physics-sync node."""
    _call_trigger_service(
        node,
        RESUME_SIMULATION_SERVICE,
        parameters.simulation_service_timeout_sec,
    )


def _call_trigger_service(node, service_name, timeout_sec):
    client = node.create_client(Trigger, service_name)
    try:
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(
                f"Simulation service '{service_name}' not available after "
                f'{timeout_sec}s.'
            )

        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)

        response = future.result() if future.done() else None
        if response is None:
            raise RuntimeError(
                f"Simulation service '{service_name}' failed or timed out."
            )
        if not response.success:
            raise RuntimeError(
                response.message
                or f"Simulation service '{service_name}' rejected the request."
            )
    finally:
        node.destroy_client(client)
