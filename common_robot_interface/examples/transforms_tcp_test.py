# -*- coding: utf-8 -*-
"""Simple test script for transforms.
"""

import time

import numpy as np

from cri.transforms import euler2quat, quat2euler, transform, inv_transform, frame, transform_euler, inv_transform_euler, frame

np.set_printoptions(precision=2, suppress=True)


def main():
    axes = 'sxyz'

    x1 = (0, 0, 300, -180, 0, 0)
    x2 = (0, 0, 300, -180, 45, 0)
    t = (0, 0, 100, 0, 0, 0)
    print("x1:", x1)
    print("x2:", x2)
    print("t:", t)

    x1_q = euler2quat(x1, axes)
    x2_q = euler2quat(x2, axes)
    t_q = euler2quat(t, axes)
    print("x1_q:", x1_q)
    print("x2_q:", x2_q)
    print("t_q:", t_q)

    x1_t_q = transform(x1_q, t_q)
    x2_t_q = transform(x2_q, t_q)
    x1_t = quat2euler(x1_t_q, axes)
    x2_t = quat2euler(x2_t_q, axes)
    print("x1 transformed to tcp frame:", x1_t)
    print("x2 transformed to tcp frame:", x2_t)

    x1_t_q = inv_transform(x1_q, t_q)
    x2_t_q = inv_transform(x2_q, t_q)
    x1_t = quat2euler(x1_t_q, axes)
    x2_t = quat2euler(x2_t_q, axes)
    print("x1 transformed back from tcp to base frame:", x1_t)
    print("x2 transformed back from tcp to base frame:", x2_t)

    t_x1_q = transform(t_q, x1_q)
    t_x2_q = transform(t_q, x2_q)
    t_x1 = quat2euler(t_x1_q, axes)
    t_x2 = quat2euler(t_x2_q, axes)
    print("tcp transformed to x1 frame:", t_x1)
    print("tcp transformed to x2 frame:", t_x2)

    t_x1_q = inv_transform(t_q, x1_q)
    t_x2_q = inv_transform(t_q, x2_q)
    t_x1 = quat2euler(t_x1_q, axes)
    t_x2 = quat2euler(t_x2_q, axes)
    print("tcp transformed back from x1 to base frame:", t_x1)
    print("tcp transformed back from x2 to base frame:", t_x2)

    print("\n-----------------------------------------\n")

    w = (0, 0, 300, -180, 0, 0)
    x = (0, 0, 210, -180, 45, 0)
    t = (0, 0, 100, 0, 0, 0)
    print("w in base frame:", w)
    print("x in base frame:", x)
    print("t:", t)

    print("\nReturn the TCP pose in the work frame")
    x_w_q = transform(euler2quat(x, axes), euler2quat(w, axes))
    print("tcp transformed from base frame to work frame:", quat2euler(x_w_q, axes))
    t_x_w_q = frame(x_w_q, euler2quat(t, axes))
    print("... then frame extracted from original tcp", quat2euler(t_x_w_q, axes))

    print("\n-----------------------------------------\n")

    w = (0, 0, 300, -180, 0, 0)
    x = (0, 0, 10,  0, 10, 0)
    t = (0, 0, 100, 0, 0, 0)
    print("w in base frame:", w)
    print("x:", x)
    print("t:", t)

    print("\nFind the TCP move in the base frame - close")
    t_x_q = transform(euler2quat(t, axes), euler2quat(x, axes))
    print("tcp transformed from base frame to x frame:", quat2euler(t_x_q, axes))
    t_x_w_q = inv_transform(t_x_q, euler2quat(w, axes))
    print("... then transformed back to base frame:", quat2euler(t_x_w_q, axes))

    print("\n-----------------------------------------\n")

    w = (0, 0, 300, -180, 0, 0)
    x = (0, 0, 0,  0, 10, 90)
    t = (0, 0, 100, 0, 0, 0)
    print("w in base frame:", w)
    print("x:", x)
    print("t:", t)

    print("\nFind the TCP in the pose frame - separate angles and positions")
    t_x_q_rot = transform(euler2quat(t, axes), euler2quat((0,0,0)+x[3:], axes))
    t_x_q = transform(t_x_q_rot, euler2quat(x[:3]+(0,0,0), axes))
    print("tcp transformed from base frame to x frame:", quat2euler(t_x_q, axes))
    t_x_w_q = inv_transform(t_x_q, euler2quat(w, axes))
    print("... then transformed back to base frame:", quat2euler(t_x_w_q, axes))

    print("\n-----------------------------------------\n")

    w = (0, 0, 300, -180, 0, 0)
    x = (0, 0, 0,  0, 10, 90)
    t = (0, 0, 100, 0, 0, 0)
    print("w in base frame:", w)
    print("x:", x)
    print("t:", t)

    print("\nFind the TCP in the pose frame - simpler separate angles and positions")
    t_x_q_rot = transform(euler2quat(t, axes), euler2quat((0,0,0)+x[3:], axes))
    t_x_q = transform(t_x_q, euler2quat(x[:3]+(0,0,0), axes))
    print("tcp transformed from base frame to x frame:", quat2euler(t_x_q, axes))
    t_x_w_q = inv_transform(t_x_q, euler2quat(w, axes))
    print("... then transformed back to base frame:", quat2euler(t_x_w_q, axes))

if __name__ == '__main__':
    main()
