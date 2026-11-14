"""Check the link-frame twist contract, including rotated/off-center inertias."""

from pathlib import Path
import re
import unittest

import mujoco
import numpy as np


class BodyTwistFrameTest(unittest.TestCase):
    def setUp(self):
        self.model = mujoco.MjModel.from_xml_string('''
          <mujoco><worldbody><body name="base" pos="0.2 -0.1 0.5" quat="0.9 0.1 -0.2 0.3">
            <freejoint/>
            <inertial pos="0.02 -0.03 0.12" mass="2"
                      fullinertia="0.02 0.01 0.015 0.001 0.002 -0.0003"/>
            <geom type="box" size="0.1 0.08 0.15"/>
          </body></worldbody></mujoco>''')
        self.data = mujoco.MjData(self.model)
        self.body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "base")

    def test_link_twist_matches_link_origin_jacobian(self):
        cases = ([0, 0, 0, 0, 0, .3], [0, 0, 0, 0, .4, 0],
                 [.2, -.1, .3, 0, 0, 0], [.2, -.1, .3, .4, -.5, .6])
        for velocity in cases:
            with self.subTest(velocity=velocity):
                self.data.qvel[:] = velocity
                mujoco.mj_forward(self.model, self.data)
                jp = np.zeros((3, self.model.nv))
                jr = np.zeros_like(jp)
                mujoco.mj_jacBody(self.model, self.data, jp, jr, self.body)
                world_to_link = self.data.xmat[self.body].reshape(3, 3).T
                expected = np.r_[world_to_link @ jr @ self.data.qvel, world_to_link @ jp @ self.data.qvel]
                actual = np.zeros(6)
                mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_XBODY, self.body, actual, 1)
                np.testing.assert_allclose(actual, expected, atol=1.0e-12)
                inertial = np.zeros(6)
                mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY, self.body, inertial, 1)
                self.assertGreater(np.linalg.norm(inertial - expected), 0.05)

    def test_both_feedback_paths_use_link_frame(self):
        source = Path(__file__).resolve().parents[1] / "src"
        call = r"mj_objectVelocity\(mj_model_,\s*mj_data_,\s*mjOBJ_XBODY,\s*body_id,"
        for name in ("mujoco_system.cpp", "mujoco_ros2_control.cpp"):
            self.assertEqual(len(re.findall(call, (source / name).read_text())), 1, name)


if __name__ == "__main__":
    unittest.main()
