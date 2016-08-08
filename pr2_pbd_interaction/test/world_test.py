#! /usr/bin/env python
"""Tests the functionality of the world module.

Currently, it tests the transformation-related code, not the World object
itself.
"""

import os.path, sys
sys.path = [os.path.abspath(os.path.dirname(__file__))] + sys.path

import numpy as np
import unittest
from pr2_pbd_interaction import world
from pr2_pbd_interaction.msg import ArmState
from pr2_pbd_interaction.msg import Landmark
from geometry_msgs.msg import Pose


class TestTransforms(unittest.TestCase):
    def testConvertFromBaseToObj(self):
        arm_state = ArmState()
        arm_state.ee_pose.position.x = 0.2
        arm_state.ee_pose.position.y = 0
        arm_state.ee_pose.position.z = 0
        arm_state.ee_pose.orientation.w = 1
        arm_state.refFrame = ArmState.ROBOT_BASE
        landmark = Landmark()
        landmark.name = 'landmark1'
        landmark.pose.position.x = 0.15
        landmark.pose.position.y = 0
        landmark.pose.position.z = 0
        landmark.pose.orientation.w = 1

        rel_arm_state = world.convert_ref_frame(arm_state, ArmState.OBJECT,
                                                landmark)

        self.assertAlmostEqual(rel_arm_state.ee_pose.position.x, 0.05)
        self.assertEqual(rel_arm_state.refFrame, ArmState.OBJECT)
        self.assertEqual(rel_arm_state.refFrameLandmark.pose.position.x, 0.15)

    def testConvertObjToBase(self):
        arm_state = ArmState()
        arm_state.ee_pose.position.x = 0.05
        arm_state.ee_pose.position.y = 0
        arm_state.ee_pose.position.z = 0
        arm_state.ee_pose.orientation.w = 1
        arm_state.refFrame = ArmState.OBJECT
        arm_state.refFrameLandmark.name = 'landmark1'
        arm_state.refFrameLandmark.pose.position.x = 0.15
        arm_state.refFrameLandmark.pose.position.y = 0
        arm_state.refFrameLandmark.pose.position.z = 0
        arm_state.refFrameLandmark.pose.orientation.w = 1

        abs_arm_state = world.convert_ref_frame(arm_state, ArmState.ROBOT_BASE)

        self.assertAlmostEqual(abs_arm_state.ee_pose.position.x, 0.2)
        self.assertEqual(abs_arm_state.refFrame, ArmState.ROBOT_BASE)

        # Check that the input arg is unchanged
        self.assertEqual(arm_state.refFrame, ArmState.OBJECT)
        self.assertEqual(arm_state.ee_pose.position.x, 0.05)

    def testConvertFromObjToAnotherObj(self):
        arm_state = ArmState()
        arm_state.ee_pose.position.x = 0.05
        arm_state.ee_pose.position.y = 0
        arm_state.ee_pose.position.z = 0
        arm_state.ee_pose.orientation.w = 1
        arm_state.refFrame = ArmState.OBJECT
        arm_state.refFrameLandmark.name = 'landmark1'
        arm_state.refFrameLandmark.pose.position.x = 0.15
        arm_state.refFrameLandmark.pose.position.y = 0
        arm_state.refFrameLandmark.pose.position.z = 0
        arm_state.refFrameLandmark.pose.orientation.w = 1

        landmark = Landmark()
        landmark.name = 'landmark2'
        landmark.pose.position.x = 0.2
        landmark.pose.position.y = 0.1
        landmark.pose.position.z = 0
        landmark.pose.orientation.w = 1

        rel_arm_state = world.convert_ref_frame(arm_state, ArmState.OBJECT,
                                                landmark)

        self.assertAlmostEqual(rel_arm_state.ee_pose.position.x, 0)
        self.assertAlmostEqual(rel_arm_state.ee_pose.position.y, -0.1)

    def testGetAbsoluteMakesCopy(self):
        arm_state = ArmState()
        arm_state.ee_pose.position.x = 0.05
        arm_state.ee_pose.position.y = 0
        arm_state.ee_pose.position.z = 0
        arm_state.ee_pose.orientation.w = 1
        arm_state.refFrame = ArmState.OBJECT
        arm_state.refFrameLandmark.name = 'landmark1'
        arm_state.refFrameLandmark.pose.position.x = 0.15
        arm_state.refFrameLandmark.pose.position.y = 0
        arm_state.refFrameLandmark.pose.position.z = 0
        arm_state.refFrameLandmark.pose.orientation.w = 1

        pose = world.get_absolute_pose(arm_state)

        self.assertEqual(pose.position.x, 0.2)
        self.assertEqual(arm_state.refFrame,  ArmState.OBJECT)
        self.assertEqual(arm_state.ee_pose.position.x,  0.05)


if __name__ == '__main__':
    unittest.main()
