import matplotlib.pyplot as plt
import numpy as np
import matplotlib

def plot_3d(data: np.array, newfigure=True, plot_grid = True, suppress = False, axes_equal = True, plotfn = "scatter", plot_args = {}):
    if newfigure:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection = "3d", proj_type = 'ortho')
    else:
        fig = plt.gcf()
        ax = plt.gca()
    if plotfn == "scatter":
        ax.scatter(data[:, 0], data[:, 1], data[:, 2], **plot_args)
    elif plotfn == "line":
        ax.plot(data[:, 0], data[:, 1], data[:, 2], **plot_args)

    if plot_grid:
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
    if axes_equal:
        set_axes_equal_3d(ax)
    if not suppress:
        plt.show()

    return True, (fig, ax)

def plot_2d(data: np.array, newfigure=True, plot_grid = True, suppress = False, axes_equal = True, plotfn = "scatter", plot_args = {}):
    if newfigure:
        fig = plt.figure()
        ax = fig.add_subplot(111)
    else:
        fig = plt.gcf()
        ax = plt.gca()
    if plotfn == "scatter":
        ax.scatter(data[:, 0], data[:, 1], **plot_args)
    elif plotfn == "line":
        ax.plot(data[:, 0], data[:, 1], **plot_args)
    
    if plot_grid:
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
    if axes_equal:
        set_axes_equal_2d(ax)
    if not suppress:
        plt.show()

    return True, (fig, ax)

def set_axes_equal_3d(ax):
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
    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])    


def set_axes_equal_2d(ax: matplotlib.axes.Axes):
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    '''

    x_limits = ax.get_xlim()
    y_limits = ax.get_ylim()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5*max([x_range, y_range])

    ax.set_xlim([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim([y_middle - plot_radius, y_middle + plot_radius])