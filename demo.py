from pib_sdk.control import *
from pib_sdk.kinematics import fk, ik

ik_angles = ik('right', xyz=[150,0,350])

w = Write(debug=True)
w.move(right_arm, *ik_angles)

