#!/usr/bin/env python3
from control import ss, lqr
import numpy as np
import rospy
from robot_control.srv import Gain, GainResponse

class LQRNode:
    def __init__(self):
        self.A = None
        self.B = None
        self.Q = None
        self.R = None

        rospy.init_node('lqr_optimizer')
        self.server = rospy.Service('lqr_gain', Gain, self.gainCallBack)

        pass

    def gainCallBack(self, req):
        res = GainResponse()
        A = np.array(req.A).reshape(5,5)
        B = np.array(req.A).reshape(5,2)
        Q = np.diag(req.Q)
        R = np.diag(req.R)
        K,_, _ = lqr(A, B, R, Q)
        print(K.type)
        return True

if __name__ == "__main__":
    lqr_optim = LQRNode()
    rospy.spin()
    pass