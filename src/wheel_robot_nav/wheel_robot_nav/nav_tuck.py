"""Joint targets that fold both arms into the same pose.

The zero pose leaves each wrist about 1 m to the side. These targets drop the
upper arms and bring the forearms forward, and the right-arm angles are the
mirror of the left. Joints 1, 3, 5 and 7 flip sign; joints 2, 4 and 6 do not,
because those axes are already mirrored in the URDF.
"""

LEFT_JOINTS = [
    "Link1_L_joint",
    "Link2_L_joint",
    "Link3_L_joint",
    "Link4_L_joint",
    "Link5_L_joint",
    "Link6_L_joint",
    "Link7_L_joint",
]

RIGHT_JOINTS = [
    "Link1_R_joint",
    "Link2_R_joint",
    "Link3_R_joint",
    "Link4_R_joint",
    "Link5_R_joint",
    "Link6_R_joint",
    "Link7_R_joint",
]

TUCK_LEFT = [2.460, -1.616, -1.972, -2.400, -0.600, 0.175, -0.841]
TUCK_RIGHT = [-2.460, -1.616, 1.972, -2.400, 0.600, 0.175, 0.841]

GRIPPER_CLOSED = 0.7
