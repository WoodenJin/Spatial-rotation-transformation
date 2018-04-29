"""
mpu_glove.py is a python api for space rotation
The ZJU license
Copyright @ Apr.27 2018, Wooden Jin
e-mail : yongbinjin@zju.edu.cn

the qua is [w x y z], means w+[x,y,z]
the Euler is [Yaw Pitch Roll]
"""
import numpy as np
from math import atan2, asin, pi, cos, sin


class Euler:
    def __init__(self):
        self.Yaw = 0
        self.Pitch = 0
        self.Roll = 0


class Rot(object):
    """
    transform the express of rotation
    """
    """
    def __init__(self):
        # the default value is no rotation
        self.qua = [1, 0, 0, 0]
        self.euler = Euler()
        self.rotMat = np.eye(3)
    """
    def rotMat2quatern(R):
        # this function can transform the rotation matrix into quatern
        q = np.zeros(4)
        K = np.zeros([4, 4])
        K[0, 0] = 1 / 3 * (R[0, 0] - R[1, 1] - R[2, 2])
        K[0, 1] = 1 / 3 * (R[1, 0] + R[0, 1])
        K[0, 2] = 1 / 3 * (R[2, 0] + R[0, 2])
        K[0, 3] = 1 / 3 * (R[1, 2] - R[2, 1])
        K[1, 0] = 1 / 3 * (R[1, 0] + R[0, 1])
        K[1, 1] = 1 / 3 * (R[1, 1] - R[0, 0] - R[2, 2])
        K[1, 2] = 1 / 3 * (R[2, 1] + R[1, 2])
        K[1, 3] = 1 / 3 * (R[2, 0] - R[0, 2])
        K[2, 0] = 1 / 3 * (R[2, 0] + R[0, 2])
        K[2, 1] = 1 / 3 * (R[2, 1] + R[1, 2])
        K[2, 2] = 1 / 3 * (R[2, 2] - R[0, 0] - R[1, 1])
        K[2, 3] = 1 / 3 * (R[0, 1] - R[1, 0])
        K[3, 0] = 1 / 3 * (R[1, 2] - R[2, 1])
        K[3, 1] = 1 / 3 * (R[2, 0] - R[0, 2])
        K[3, 2] = 1 / 3 * (R[0, 1] - R[1, 0])
        K[3, 3] = 1 / 3 * (R[0, 0] + R[1, 1] + R[2, 2])
        # print(R)
        # print("***********")
        # print(K)
        D, V = np.linalg.eig(K)
        # print(K)
        pp = 0
        for i in range(1, 4):
            if(D[i] > D[pp]):
                pp = i
        # print(D[pp])
        # print(D)
        q = V[:, pp]
        q = np.array([q[3], q[0], q[1], q[2]])
        return q

    def quatern2rotMat(qua):
        # get the rotation matrix from quatern
        # temp = [qua[3], qua[0], qua[1], qua[2]]
        # matrix = p.getMatrixFromQuaternion(qua)
        # matrix = np.asarray(matrix)
        # matrix = matrix.reshape(3, 3)
        R = np.zeros([3, 3])
        w = qua[0]
        x = qua[1]
        y = qua[2]
        z = qua[3]
        R[0, 0] = w**2 + x**2 - y**2 - z**2
        R[0, 1] = 2 * (x * y + z * w)
        R[0, 2] = 2 * (x * z - y * w)
        R[1, 0] = 2 * (x * y - z * w)
        R[1, 1] = w**2 + y**2 - x**2 - z**2
        R[1, 2] = 2 * (y * z + x * w)
        R[2, 0] = 2 * (x * z + y * w)
        R[2, 1] = 2 * (y * z - x * w)
        R[2, 2] = w**2 + z**2 - x**2 - y**2
        return R

    def quatern2Euler(qua):
        euler = Euler()
        L = (qua[0]**2 + qua[1]**2 + qua[2]**2 + qua[3]**2)**0.5
        w = qua[0] / L
        x = qua[1] / L
        y = qua[2] / L
        z = qua[3] / L
        euler.Roll = atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
        if euler.Roll < 0:
            euler.Roll += 2 * pi
        # print("*")
        # print(w * y - z * x)
        temp = w * y - z * x
        if temp >= 0.5:
            temp = 0.5
        elif temp <= -0.5:
            temp = -0.5
        else:
            pass
        euler.Pitch = asin(2 * temp)
        euler.Yaw = atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
        if euler.Yaw < 0:
            euler.Yaw += 2 * pi
        return euler

    def Euler2quatern(euler):
        roll = euler.Roll / 2
        pitch = euler.Pitch / 2
        yaw = euler.Yaw / 2
        w = cos(roll) * cos(pitch) * cos(yaw) + \
            sin(roll) * sin(pitch) * sin(yaw)
        x = sin(roll) * cos(pitch) * cos(yaw) - \
            cos(roll) * sin(pitch) * sin(yaw)
        y = cos(roll) * sin(pitch) * cos(yaw) + \
            sin(roll) * cos(pitch) * sin(yaw)
        z = cos(roll) * cos(pitch) * sin(yaw) + \
            sin(roll) * sin(pitch) * cos(yaw)
        qua = [w, x, y, z]
        return qua

    def Euler2rotMat(euler):
        qua = Rot.Euler2quatern(euler)
        R = Rot.quatern2rotMat(qua)
        return R

    def rotMat2Euler(R):
        qua = Rot.rotMat2quatern(R)
        euler = Rot.quatern2Euler(qua)
        return(euler)


def main():
    # qua = [1, 0, 0, 0]
    # R = Rot.quatern2rotMat(qua)
    # euler = Rot.quatern2Euler(qua)
    euler = Euler()
    euler.Yaw = pi / 2
    R = Rot.Euler2rotMat(euler)
    qua = Rot.Euler2quatern(euler)
    print(R)
    print(qua)
    print(euler.Yaw, euler.Pitch, euler.Roll)
    print(Rot.quatern2rotMat(qua))
    print(Rot.quatern2Euler(qua))


if __name__ == "__main__":
    main()
