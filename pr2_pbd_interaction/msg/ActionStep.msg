# ActionStep represents a single step in a PbD action.
# A step consists of the positions of the two arms, provided in armTarget, and the gripper states.
# A step could also be a recorded arm trajectory, but that has not been enabled or tested recently.
uint8 ARM_TARGET=0
uint8 ARM_TRAJECTORY=1 # Unsupported
uint8 type # The type of step, only ARM_TARGET is supported.
pr2_pbd_interaction/ArmTarget armTarget # the position of the two arms.
pr2_pbd_interaction/ArmTrajectory armTrajectory # Unsupported
pr2_pbd_interaction/Condition preCond # Unused
pr2_pbd_interaction/Condition postCond # Unused
pr2_pbd_interaction/GripperAction gripperAction # Whether each gripper is opened or closed.
