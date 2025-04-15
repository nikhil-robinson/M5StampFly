# ref: https://github.com/juancamilog/calibrate_imu/blob/master/nodes/calibrate_imu.py

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import csv

def mag_least_squares_method(xyz):

    print('Starting least-squared based magnetometer calibration with %d samples'%(xyz.shape[0]))

    #compute the vectors [ x^2 y^2 z^2 2*x*y 2*y*z 2*x*z x y z 1] for every sample
    # the result for the x*y y*z and x*z components should be divided by 2
    xyz2 = np.power(xyz,2)
    xy = np.multiply(xyz[:,0],xyz[:,1])
    xz = np.multiply(xyz[:,0],xyz[:,2])
    yz = np.multiply(xyz[:,1],xyz[:,2])

    # build the data matrix
    A = np.bmat('xyz2 xy xz yz xyz')

    b = 1.0*np.ones((xyz.shape[0],1))

    # solve the system Ax = b
    q,res,rank,sing = np.linalg.lstsq(A,b)

    # build scaled ellipsoid quadric matrix (in homogeneous coordinates)
    A = np.matrix([[q[0][0],0.5*q[3][0],0.5*q[4][0],0.5*q[6][0]],
                [0.5*q[3][0],q[1][0],0.5*q[5][0],0.5*q[7][0]],
                [0.5*q[4][0],0.5*q[5][0],q[2][0],0.5*q[8][0]],
                [0.5*q[6][0],0.5*q[7][0],0.5*q[8][0],-1]])

    # build scaled ellipsoid quadric matrix (in regular coordinates)
    Q = np.matrix([[q[0][0],0.5*q[3][0],0.5*q[4][0]],
                [0.5*q[3][0],q[1][0],0.5*q[5][0]],
                [0.5*q[4][0],0.5*q[5][0],q[2][0]]])

    # obtain the centroid of the ellipsoid
    x0 = np.linalg.inv(-1.0*Q) * np.matrix([0.5*q[6][0],0.5*q[7][0],0.5*q[8][0]]).T

    # translate the ellipsoid in homogeneous coordinates to the center
    T_x0 = np.matrix(np.eye(4))
    T_x0[0,3] = x0[0]; T_x0[1,3] = x0[1]; T_x0[2,3] = x0[2];
    A = T_x0.T*A*T_x0

    # rescale the ellipsoid quadric matrix (in regular coordinates)
    Q = Q*(-1.0/A[3,3])

    # take the cholesky decomposition of Q. this will be the matrix to transform
    # points from the ellipsoid to a sphere, after correcting for the offset x0
    L = np.eye(3)
    try:
        L = np.linalg.cholesky(Q).transpose()
    except Exception as e:
        print(e)
        L = np.eye(3)

    print("Magnetometer offset:\n", x0)
    print("Magnetometer Calibration Matrix:\n",L)

    return x0, L


def plot_data(xyz):
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1, projection='3d')
    ax.scatter(xyz[:, 0], xyz[:, 1], xyz[:, 2], c='b')
    plt.show()


if __name__ == "__main__":
    csv_file = "magnetic_data2.csv"
    # data = np.loadtxt(csv_file, delimiter=" ")
    data = np.genfromtxt(csv_file, delimiter=None, dtype=float)
    xyz = np.matrix(data)
    x0 , L = mag_least_squares_method(xyz)
    print(xyz)
    plot_data(xyz)

    fixed_data = (L * (xyz - x0.T).T).T
    print(fixed_data)
    plot_data(fixed_data)

