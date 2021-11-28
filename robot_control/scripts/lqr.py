#!/usr/bin/env python3
from control import ss, lqr, c2d
import numpy as np
import rospy
from robot_control.srv import Gain, GainResponse
# import time

class LQRNode:
    def __init__(self, dt):
        self.dt = dt

        self.m = 0.30465
        self.M = 5.778
        self.r = 0.05
        self.d = 0.19
        self.j_p = None
        self.j_w = 0.5 * self.m * self.r**2
        self.j_d = self.M * self.d**2 / 12
        self.sys1 = None
        self.sys2 = None

        self.Q1 = np.diag([1, 1, 10, 100])
        self.R1 = np.eye(1)
        self.Q2 = np.diag([10, 100])
        self.R2 = np.eye(1)

        rospy.init_node('lqr_optimizer')
        self.server = rospy.Service('/lqr_gain', Gain, self.gainCallBack)
        rospy.loginfo("LQR Optimizer Node is Online...")
        pass

    def gainCallBack(self, req):
        # start_time = time.time()
        self.updateModel(req.l)
        k1, _, _ = lqr(self.sys1, self.Q1, self.R1)
        k2, _, _ = lqr(self.sys2, self.Q2, self.R2)
        res = GainResponse(k1.flatten().tolist() + k2.flatten().tolist())
        # rospy.loginfo("--- %s seconds ---" % (time.time() - start_time))
        return res
    
    def updateModel(self, l):
        g = 9.81
        self.j_p = self.M * l**2 / 3
        a1 = np.zeros((4, 4))
        b1 = np.zeros((4,1))
        a2 = np.array([[0,1],[0,0]])
        b2 = np.zeros((2,1))
        x = 2 * self.j_p * self.j_w + 2 * self.M * l**2 * self.j_w + self.M * self.r**2 * self.j_p + \
                    2 * self.m * self.r**2 * self.j_p + 2 * self.m * self.M * (l * self.r)**2
        a1[0,1] = 1
        a1[2,3] = 1
        a1[1,2] = -g * (l * self.M * self.r)**2 / x
        a1[3,2] = l * self.M * g * (2 * self.j_w + self.M * self.r**2 + 2 * self.m * self.r**2) / x
        b1[1,0] = self.r * (self.M * l**2 + self.M * l * self.r + self.j_p) / x
        b1[3,0] = -(2 * self.j_w + self.M * self.r**2 + 2 * self.m * self.r**2 + l * self.M * self.r) / x
        b2[1,0] = self.d * self.r / ((self.m * self.r**2 + self.j_w) * self.d**2 + 2 * self.j_d * self.r**2)
        self.sys1 = c2d(ss(a1, b1, np.eye(4), np.zeros((4,1))), self.dt)
        self.sys2 = c2d(ss(a2, b2, np.eye(2), np.zeros((2,1))), self.dt)

        rospy.loginfo("State Space Model Updated.")
        # print(self.sys1)
        # print("----------------")
        # print(self.sys2)
        return True

if __name__ == "__main__":
    lqr_optim = LQRNode(0.001)
    rospy.spin()
    pass