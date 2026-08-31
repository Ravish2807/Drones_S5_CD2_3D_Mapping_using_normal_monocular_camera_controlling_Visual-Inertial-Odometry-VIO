import os
import sys
import yaml
import unittest
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from interfaces.packets import IMUPacket, ImagePacket
from interfaces.vio_interface import VIOInterface
from interfaces.mock_ros2_player import MockROS2Player
from datasets.synthetic.synthetic_generator import SyntheticDatasetGenerator


class TestInterfaceAndMockROS(unittest.TestCase):
    """
    Validates the standardized Adapter Boundary and Simulated ROS 2 event loop.
    Ensures zero coupling between the VIO mathematical core and ROS 2 middleware.
    """

    def setUp(self):
        with open("config/synthetic.yaml", "r") as f:
            self.config = yaml.safe_load(f)
        self.config["simulation"]["duration_sec"] = 2.0

        gen = SyntheticDatasetGenerator(self.config)
        self.ds = gen.generate_dataset("hover")

    def test_direct_interface_feed(self):
        """Tests standard push_imu and push_camera_frame calls."""
        vio = VIOInterface(self.config)
        gt0 = self.ds["ground_truth"][0]
        vio.estimator.p_WB = gt0["position"].copy()
        vio.estimator.v_WB = gt0["velocity"].copy()
        vio.estimator.R_WB = gt0["R_WB"].copy()
        vio.estimator.is_initialized = True

        received_callbacks = []
        vio.register_callback(lambda state: received_callbacks.append(state))

        imu_stream = self.ds["imu"]
        cam_stream = self.ds["camera"]

        imu_idx = 0
        for cam in cam_stream[:10]:
            t_cam = cam["timestamp"]
            while imu_idx < len(imu_stream) and imu_stream[imu_idx]["timestamp"] <= t_cam:
                s = imu_stream[imu_idx]
                vio.push_imu(s["timestamp"], *s["linear_accel"], *s["angular_vel"])
                imu_idx += 1
            vio.push_camera_frame(t_cam, cam["image"])

        self.assertGreater(len(received_callbacks), 0)
        self.assertEqual(len(vio.state_history), 10)
        
        curr_state = vio.get_current_state()
        self.assertIsNotNone(curr_state)
        self.assertEqual(len(curr_state.position), 3)
        self.assertEqual(len(curr_state.orientation_q), 4)

        # Test CSV export
        test_csv = "results/trajectories/test_vio_estimate.csv"
        vio.export_state_csv(test_csv)
        self.assertTrue(os.path.exists(test_csv))

    def test_mock_ros2_player_queue(self):
        """Tests MockROS2Player chronological min-heap queue execution."""
        vio = VIOInterface(self.config)
        gt0 = self.ds["ground_truth"][0]
        vio.estimator.p_WB = gt0["position"].copy()
        vio.estimator.v_WB = gt0["velocity"].copy()
        vio.estimator.R_WB = gt0["R_WB"].copy()
        vio.estimator.is_initialized = True

        player = MockROS2Player(vio)

        # Feed packets in interleaved / non-sorted order to test queue sorting
        for cam in self.ds["camera"][:10]:
            player.publish_image(ImagePacket(cam["timestamp"], cam["image"]))

        for s in self.ds["imu"][:100]:
            player.publish_imu(IMUPacket(s["timestamp"], s["linear_accel"], s["angular_vel"]))

        # Spin player
        states = player.spin_all()
        self.assertGreater(len(states), 0)
        self.assertEqual(len(states), 10)
        print(f"\n[PASS] MockROS2Player successfully executed {len(states)} frame updates.")


if __name__ == "__main__":
    unittest.main()
