import matplotlib
# use one of the following backends if matplotlib does not
# render correctly.  See https://matplotlib.org/stable/users/explain/backends.html
# matplotlib.use('qtagg')
# matplotlib.use('ipympl')
# matplotlib.use('gkt3agg')
# matplotlib.use('gkt4agg')
# matplotlib.use('macosx')
# matplotlib.use('tkagg')
# matplotlib.use('nbagg')
# matplotlib.use('gkt3cairo')
# matplotlib.use('gkt4cairo')
# matplotlib.use('wxagg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import VTOLParam as P


class VTOLAnimation:
    '''
        Create VTOL animation
    '''
    def __init__(self):
        self.flagInit = True                  # Used to indicate initialization
        self.fig, self.ax = plt.subplots()    # Initializes a figure and axes object
        self.handle = []                      # Initializes a list object that will
                                              # be used to contain handles to the
                                              # patches and line objects.
        plt.plot([0.0, P.length], [0.0, 0.0], 'k')    # Draw a base line
        plt.axis([-P.length / 5, P.length + P.length / 5, -P.length/5, P.length+P.length/5])  # Change the x,y axis limits

    def update(self, x, target=0.0):
        # Process inputs to function
        z = x[0,0]  # lateral position of VTOL (m)
        h = x[1,0]  # altitude of VTOL (m)
        theta = x[2,0]   # Angle of VTOL (rad)

        self.drawVehicle(z, h, theta)
        self.drawTarget(target)

        # After each function has been called, initialization is over.
        if self.flagInit == True:
            self.flagInit = False

    def drawVehicle(self, z, h, theta):
        x1 = 0.1
        x2 = 0.3
        x3 = 0.4
        y1 = 0.05
        y2 = 0.01
        pts = np.array([
            [x1, y1],
            [x1, 0],
            [x2, 0],
            [x2, y2],
            [x3, y2],
            [x3, -y2],
            [x2, -y2],
            [x2, 0],
            [x1, 0],
            [x1, -y1],
            [-x1, -y1],
            [-x1, 0],
            [-x2, 0],
            [-x2, -y2],
            [-x3, -y2],
            [-x3, y2],
            [-x2, y2],
            [-x2, 0],
            [-x1, 0],
            [-x1, y1],
            [x1, y1]]).T
        R = np.array([
            [np.cos(theta), np.sin(theta)],
            [-np.sin(theta), np.cos(theta)],
        ])
        pts = R.T @ pts
        pts = pts + np.repmat(np.array([[z],[h]]), 1, pts.shape[1])
        xy = np.array(pts.T)

        # When the class is initialized, a polygon patch object will be
        # created and added to the axes. After initialization, the polygon
        # patch object will only be updated.
        if self.flagInit == True:
            # Create the Rectangle patch and append its handle
            # to the handle list
            self.handle.append(mpatches.Polygon(xy, facecolor='blue', edgecolor='black'))
            self.ax.add_patch(self.handle[0]) # Add the patch to the axes
        else:
            self.handle[0].set_xy(xy)         # Update polygon

    def drawTarget(self, target=0.0):
        w = 0.1
        h = 0.05
        pts = np.array([
               [target+w/2.0, h],
               [target+w/2.0, 0],
               [target-w/2.0, 0],
               [target-w/2.0, h],
               [target+w/2.0, h]])

        if self.flagInit == True:
            # Create the Rectangle patch and append its handle
            # to the handle list
            self.handle.append(mpatches.Polygon(pts, facecolor='blue', edgecolor='black'))
            self.ax.add_patch(self.handle[1]) # Add the patch to the axes
        else:
            self.handle[1].set_xy(pts)         # Update polygon
