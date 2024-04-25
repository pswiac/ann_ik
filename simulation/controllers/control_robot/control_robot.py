from joint import Joint
from ur5e import UR5e
from ur5_controller import Ur5Controller

import math

pi = math.pi

joint1 = Joint(pi, 0, 0.1625, pi/2, 3.761, [[0, -0.02561, 0.00193]])
joint2 = Joint(pi, -0.425, 0, 0, 8.058, [0.2125, 0, 0.11336])
joint3 = Joint(pi, -0.3922, 0, 0, 2.846, [0.15, 0.0, 0.0265])
joint4 = Joint(pi, 0, 0.1333, pi/2, 1.37, [0, 0, 0.01])
joint5 = Joint(pi, 0, 0.0997, -pi/2, 1.3, [0, 0, 0.01])
joint6 = Joint(pi, 0, 0.0996, 0, 0.365, [0, 0, -0.001159])


ur5 = UR5e(joint1, joint2, joint3, joint4, joint5, joint6)

controller = Ur5Controller(ur5)

if __name__ == '__main__':
    controller.initUr5()
    controller.moveToTarget([2.8281939856595333, 2.4228635353386743, 1.539517377222528])
    
