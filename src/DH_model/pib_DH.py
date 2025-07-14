import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3

camera=SE3.Tx(-91) * SE3.Tz(643.6) * SE3.Rz(-np.pi/2) * SE3.Rx(np.pi/2) #camera link to base

class pib_right(DHRobot):
    """
    Class that models pib's right arm with specified DH parameters
    This description has θ₁ and θ₂ fixed at 0 degrees and θ₉ fixed at -90 degrees.
    The remaining joints have specified joint limits.
    """
    def __init__(self):
        pi = np.pi
        d = [-59, 0, 216.3, 0, -242, 2.7]
        a = [0, 0, -21, 21, 15, -45.2]
        alpha = [pi/2, pi/2, pi/2, pi/2, pi/2, -pi/2]
        offset = [0, 0, pi/2, pi/4, pi/2, 0]
        joint_limits_lower = ([-pi/2, -pi/2, -pi/2, -pi/4, -pi/2, -pi/4])
        joint_limits_upper = ([pi/2, pi/2, pi/2, pi/2, pi/2, pi/4])
        links = []
        for i in range(6):
            link = RevoluteDH(
                d=d[i],
                a=a[i],
                alpha=alpha[i],
                offset=offset[i],
                qlim=[joint_limits_lower[i], joint_limits_upper[i]],
            )
            links.append(link)

        T_tool = SE3.Rz(-pi/2) * SE3.Tz(-42.3) * SE3.Rx(pi/2)
        super().__init__(
            links,
            name="pib_right",
            manufacturer="isento GmbH",
            tool=T_tool,
        )
        self.qz = np.zeros(6)
        self.addconfiguration("qz", self.qz)
        self.q_observe = np.radians([40, -90, -30, 40, -90, 0])
        self.addconfiguration("q_observe", self.q_observe)
        self.q_rest = np.radians([90, -90, 0, -40, 0, 0])
        self.addconfiguration("q_rest", self.q_rest)
        self.base = SE3.Tz(480.5) * SE3.Tx(7.3) * SE3.Rx(pi/2) * SE3.Tz(-160)

class pib_left(DHRobot):
    """
    Class that models pib's left arm with specified DH parameters
    This robot has θ₁ and θ₂ fixed at 0 degrees and θ₉ fixed at -90 degrees.
    The remaining joints have specified joint limits.
    """

    def __init__(self):
        pi = np.pi
        d = [-59, 0, 216.3, 0, -242, -2.7]
        a = [0, 0, -21, 21, 15, -45.2]
        alpha = [pi/2, pi/2, pi/2, pi/2, pi/2, -pi/2]
        offset = [0, 0, pi/2, pi/4, (pi/2+pi), 0]
        joint_limits_lower = ([-pi/2, -pi/2, -pi/2, -pi/4, -pi/2, -pi/4])
        joint_limits_upper = ([pi/2, pi/2, pi/2, pi/2, pi/2, pi/4])

        links = []
        for i in range(6):
            link = RevoluteDH(
                d=d[i],
                a=a[i],
                alpha=alpha[i],
                offset=offset[i],
                qlim=[joint_limits_lower[i], joint_limits_upper[i]],
            )
            links.append(link)

        T_tool = SE3.Rz(-pi/2) * SE3.Tz(-42.3) * SE3.Rx(pi/2)
        super().__init__(
            links,
            name="pib_left",
            manufacturer="isento GmbH",
            tool=T_tool,
        )
        self.qz = np.zeros(6)
        self.addconfiguration("qz", self.qz)
        self.q_observe = np.radians([-40, 90, 30, 40, 90, 0])
        self.addconfiguration("q_observe", self.q_observe)
        self.q_rest = np.radians([-90, 90, 0, -40, 0, 0])
        self.addconfiguration("q_rest", self.q_rest)
        self.base = SE3.Tz(480.5) * SE3.Tx(-7.3) * SE3.Rx(np.pi/2) * SE3.Tz(-160)
        self.base= SE3.Rz(np.pi) * self.base

if __name__ == "__main__":
    Pib_right = pib_right()
    print(Pib_right)
    Pib_left = pib_left()
    print(Pib_left)
    print(camera)