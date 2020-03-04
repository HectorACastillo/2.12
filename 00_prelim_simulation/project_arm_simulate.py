from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import

import matplotlib.pyplot as plt
import numpy as np
import time


def set_axes_equal(ax):
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    '''

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5 * max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


def Rn(a):
    return np.array([[1, 0, 0],
                     [0, np.cos(a), -np.sin(a)],
                     [0, np.sin(a), np.cos(a)]])


def Rt(a):
    return np.array([[np.cos(a), 0, np.sin(a)],
                     [0, 1, 0],
                     [-np.sin(a), 0, np.cos(a)]])


def Rb(a):
    return np.array([[np.cos(a), -np.sin(a), 0],
                     [np.sin(a), np.cos(a), 0],
                     [0, 0, 1]])


def a_rand(min, max, N=1):
    return (min + (max - min) * np.random.rand(N))[0]


def show_rot(ax, R, l):
    n0 = np.array([[l, 0, 0]]).T
    t0 = np.array([[0, l, 0]]).T
    b0 = np.array([[0, 0, l]]).T
    n = R @ n0
    t = R @ t0
    b = R @ b0
    nl = np.concatenate((np.zeros((3, 1)), n), axis=1)
    tl = np.concatenate((np.zeros((3, 1)), t), axis=1)
    bl = np.concatenate((np.zeros((3, 1)), b), axis=1)
    ax.plot(nl[0, :], nl[1, :], nl[2, :], 'r')
    ax.plot(tl[0, :], tl[1, :], tl[2, :], 'g')
    ax.plot(bl[0, :], bl[1, :], bl[2, :], 'b')

# setup figures, plots
fig = plt.figure()
# ax = fig.add_subplot(111, aspect='equal', projection='3d')
# fig2 = plt.figure()
# ax2 = fig2.add_subplot(111, aspect='equal')
ax = fig.add_subplot(211, aspect='equal', projection='3d')
ax2 = fig.add_subplot(212, aspect='equal')

# draw axes
show_rot(ax, np.identity(3), .1)

# draw table, basket
x_table = [.30, .30, -.30, -.30, .30]
y_table = [0, .50, .50, 0, 0]
x_basket = [-.30, -.30, -.55, -.55, -.30]
y_basket = [0, .35, .35, 0, 0]
ax.plot(x_table, y_table, np.zeros(5), '--k')
ax.plot(x_basket, y_basket, np.zeros(5), '--k')
ax2.plot(x_table, y_table, '--k')
ax2.plot(x_basket, y_basket, '--k')

# set target reach
target_A = np.array([[.3, .5, 0]]).T
target_B = np.array([[.3, 0, 0]]).T
target_C = np.array([[-.55, 0, 0]]).T
target_D = np.array([[-.55, .35, 0]]).T
target_E = np.array([[-.3, .5, 0]]).T
target_origin = np.array([[0, 0, 0]]).T

## THINGS YOU CARE ABOUT ------------------------------------
# set target for robot
target = target_D

# relative positions, lengths of robot
scale = .35 # scale all links at once
x01 = np.array([[0, -.1, .1]]).T # position of initial joint
x12 = np.array([[0, 0, 1*scale]]).T # length of link 1
x23 = np.array([[0, 0, 1*scale]]).T # length of link 2
x3e = np.array([[0, 0, -0.2*scale]]).T # length of link 3

# number of iterations to try to reach target
N = 100

# show every iteration (only use with N<500)
show_all = True
## THINGS YOU CARE ABOUT ------------------------------------

# initialize for loop
x0e_l = np.zeros((3, N))
norm_best = 100

for i in range(N):

    # angles
    p1 = a_rand(0, np.pi)
    t1 = a_rand(0, np.pi * 3 / 4)
    t2 = a_rand(0, np.pi * 7 / 8)

    # manual set angles
    # p1 = np.pi/2
    # t1 = np.pi/4
    # t2 = np.pi/8

    # rotation matrices
    R01 = Rb(p1) @ Rt(t1)
    R12 = Rt(t2)

    # postition of joints
    x02 = (x01 +
           R01 @ x12)
    x03 = (x01 +
           R01 @ x12 +
           R01 @ R12 @ x23)
    x0e = (x01 +
           R01 @ x12 +
           R01 @ R12 @ x23 +
           x3e)

    # all joint positions
    bot = np.concatenate((x01, x02, x03, x0e), axis=1)
    for z in bot[2, :]:
        if z < 0:
            viable = False
        else:
            viable = True

    # add to list of all
    x0e_l[:, [i]] = x0e

    # calculate, test dist to target
    norm = np.linalg.norm(x0e - target)
    if (norm < norm_best) and viable:
        norm_best = norm
        i_best = i
        bot_best = bot
        angles_best = [p1, t1, t2]

    # draw robot
    if show_all:
        b_alpha = 1;
        if x0e[2] < 0.25 and x0e[2] > 0:
            e_color = 'palegreen'
        else:
            e_color = 'skyblue'
        if viable:
            b_color = 'slategrey'
        else:
            e_color = 'rosybrown'
            b_color = 'rosybrown'
            b_alpha = 0.2

        ax.scatter(x0e[0], x0e[1], x0e[2], c=e_color, alpha = b_alpha)
        ax2.scatter(x0e[0], x0e[1], c=e_color, alpha = b_alpha)
        ax.plot(bot[0,:], bot[1,:], bot[2,:], c=b_color, alpha = b_alpha)

print('x_base: ', x01.T)
print('\nlengths:')
print('l_1: ', x12[2])
print('l_2: ', x23[2])
print('l_3: ', x3e[2])
print('\nangles:')
print('p_1 = rad: ', angles_best[0], '  deg: ', angles_best[0] * 180 / np.pi)
print('t_1 = rad: ', angles_best[1], '  deg: ', angles_best[1] * 180 / np.pi)
print('t_2 = rad: ', angles_best[2], '  deg: ', angles_best[2] * 180 / np.pi)
ax.scatter(x0e_l[0, i_best], x0e_l[1, i_best], x0e_l[2, i_best], c='orange')
ax2.scatter(x0e_l[0, i_best], x0e_l[1, i_best], c='orange')
ax.plot(bot_best[0, :], bot_best[1, :], bot_best[2, :], c='black')

set_axes_equal(ax)
plt.show()

# ax = fig.add_subplot(212)
# ax.scatter(x, y)

# ax.plot(x_table, y_table, '--y')
# ax.plot(x_basket, y_basket, '--r')

# plt.axis('equal')
