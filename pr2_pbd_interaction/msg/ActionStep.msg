# ActionStep represents a single step in a PbD action.
# A step consists of the positions of the two arms, provided in armTarget, and the gripper states.
# A step could also be a recorded arm trajectory, but that has not been enabled or tested recently.
uint8 ARM_TARGET=0
uint8 ARM_TRAJECTORY=1 # Unsupported
uint8 type # The type of step, only ARM_TARGET is supported.
ArmTarget armTarget # the position of the two arms.
ArmTrajectory armTrajectory # Unsupported
Condition preCond # Unused
Condition postCond # Unused
GripperAction gripperAction # Whether each gripper is opened or closed.
