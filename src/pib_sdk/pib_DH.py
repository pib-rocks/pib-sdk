import numpy as np
import pinocchio as pin
from spatialmath import SE3


camera = SE3.Tx(-91) * SE3.Tz(643.6) * SE3.Rz(-np.pi/2) * SE3.Rx(np.pi/2)  # camera link to base


ARM_JOINTS_RIGHT = [
    "shoulder_vertical_right",
    "shoulder_horizontal_right",
    "upper_arm_right",
    "elbow_right",
    "forearm_right",
    "wrist_right",
]

ARM_JOINTS_LEFT = [
    "shoulder_vertical_left",
    "shoulder_horizontal_left",
    "upper_arm_left",
    "elbow_left",
    "forearm_left",
    "wrist_left",
]


def _build_reduced_model(arm_joints: list[str], ee_frame_name: str):
    full_model = pin.buildModelFromUrdf("/home/pib/pib-sdk/src/pib_sdk/robot.urdf")

    # Alle Joints die NICHT zum Arm gehören werden auf 0 fixiert
    joints_to_lock = [
        full_model.getJointId(name)
        for name in full_model.names[1:]  
        if name not in arm_joints
    ]

    q_ref = pin.neutral(full_model)
    model = pin.buildReducedModel(full_model, joints_to_lock, q_ref)
    data  = model.createData()
    return model, data


class pib_right:
    """
    Class that models pib's right arm.
    Loaded and reduced from robot.urdf via pinocchio.
    """

    def __init__(self):
        self.model, self.data = _build_reduced_model(
            ARM_JOINTS_RIGHT, "urdf_palm_right"
        )
        self.n             = self.model.nq           # = 6
        self.ee_frame_name = "urdf_palm_right"

        self.qz        = np.zeros(self.n)
        self.q_observe = np.radians([40, -90, -30, 40, -90, 0])
        self.q_rest    = np.radians([90, -90,   0, -40,  0, 0])

    def __repr__(self):
        return f"pib_right(nq={self.n}, ee_frame='{self.ee_frame_name}')"


class pib_left:
    """
    Class that models pib's left arm.
    Loaded and reduced from robot.urdf via pinocchio.
    """

    def __init__(self):
        self.model, self.data = _build_reduced_model(
            ARM_JOINTS_LEFT, "urdf_palm_left"
        )
        self.n             = self.model.nq           # = 6
        self.ee_frame_name = "urdf_palm_left"

        self.qz        = np.zeros(self.n)
        self.q_observe = np.radians([-40,  90,  30,  40, 90, 0])
        self.q_rest    = np.radians([-90,  90,   0, -40,  0, 0])

    def __repr__(self):
        return f"pib_left(nq={self.n}, ee_frame='{self.ee_frame_name}')"


if __name__ == "__main__":
    Pib_right = pib_right()
    print(Pib_right)
    Pib_left = pib_left()
    print(Pib_left)
    print(camera)