import time
import heapq
import numpy as np
from typing import List, Union, Callable

from .packets import IMUPacket, ImagePacket
from .vio_interface import VIOInterface
from core.state import VIOState


class MockROS2Player:
    """
    Simulates a ROS 2 executor event loop with multi-topic priority message queues.
    Replays IMUPacket (sensor_msgs/Imu) and ImagePacket (sensor_msgs/Image)
    in strict chronological order through VIOInterface callbacks.
    """

    def __init__(self, vio_interface: VIOInterface):
        self.vio = vio_interface
        self.queue = [] # Min-heap of (timestamp, seq, packet)
        self.seq_counter = 0

    def publish_imu(self, packet: IMUPacket):
        """Enqueues an incoming IMU message on simulated topic /imu/data."""
        heapq.heappush(self.queue, (packet.timestamp, self.seq_counter, packet))
        self.seq_counter += 1

    def publish_image(self, packet: ImagePacket):
        """Enqueues an incoming Camera message on simulated topic /camera/image_raw."""
        heapq.heappush(self.queue, (packet.timestamp, self.seq_counter, packet))
        self.seq_counter += 1

    def spin_all(self) -> List[VIOState]:
        """
        Executes all queued messages chronologically, simulating rclpy.spin().
        Returns all generated VIO state estimates.
        """
        produced_states: List[VIOState] = []

        while len(self.queue) > 0:
            timestamp, _, packet = heapq.heappop(self.queue)

            if isinstance(packet, IMUPacket):
                self.vio.push_imu_packet(packet)
            elif isinstance(packet, ImagePacket):
                state = self.vio.push_camera_packet(packet)
                if state is not None:
                    produced_states.append(state)

        return produced_states
