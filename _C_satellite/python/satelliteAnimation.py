import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np 
import satelliteParam as P
# if you are having difficulty with the graphics, 
# try using one of the following backends.  
# See https://matplotlib.org/stable/users/explain/backends.html
# import matplotlib
# matplotlib.use('qtagg')  # requires pyqt or pyside
# matplotlib.use('ipympl')  # requires ipympl
# matplotlib.use('gtk3agg')  # requires pyGObject and pycairo
# matplotlib.use('gtk4agg')  # requires pyGObject and pycairo
# matplotlib.use('gtk3cairo')  # requires pyGObject and pycairo
# matplotlib.use('gtk4cairo')  # requires pyGObject and pycairo
# matplotlib.use('tkagg')  # requires TkInter
# matplotlib.use('wxagg')  # requires wxPython


class satelliteAnimation:
    def __init__(self):
        # Used to indicate initialization
        self.flagInit = True        
        # Initializes a figure and axes object
        self.fig, self.ax = plt.subplots()
        # Initializes a list object that will be used to contain
        # handles to the patches and line objects.
        self.handle = []
        plt.axis([-2.0*P.length, 2.0*P.length, -2.0*P.length, 2.0*P.length])
        plt.plot([-2.0*P.length, 2.0*P.length], [0, 0], 'b--')
        self.length = P.length
        self.width = P.width

    def update(self, u):
        # Process inputs to function
        theta = u[0][0]   # Angle of base, rad
        phi = u[1][0]     # angle of panel, rad
        self.drawBase(theta)
        self.drawPanel(phi)
        # This will cause the image to not distort
        # self.ax.axis('equal') 
        # After each function has been called, initialization is
        # over.
        if self.flagInit == True:
            self.flagInit = False

    def drawBase(self, theta):
        # points that define the base
        pts =np.matrix([
            [self.width/2.0, -self.width/2.0],
            [self.width/2.0, -self.width/6.0],
            [self.width/2.0 + self.width/6.0, -self.width/6.0],
            [self.width/2.0 + self.width/6.0, self.width/6.0],
            [self.width/2.0, self.width/6.0],
            [self.width/2.0, self.width/2.0],
            [-self.width/2.0, self.width/2.0],
            [-self.width/2.0, self.width/6.0],
            [-self.width/2.0 - self.width/6.0, self.width/6.0],
            [- self.width/2.0 - self.width/6.0, -self.width/6.0],
            [- self.width/2.0, -self.width/6.0],
            [-self.width/2.0, -self.width/2.0]]).T
        R = np.array([[np.cos(theta), np.sin(theta)],
                      [-np.sin(theta), np.cos(theta)]])
        pts = R @ pts
        xy = np.array(pts.T)
        # When the class is initialized, a polygon patch object
        # will be created and added to the axes. After
        # initialization, the polygon patch object will only be
        # updated.
        if self.flagInit == True:
            # Create the Rectangle patch and append its handle
            # to the handle list
            self.handle.append(mpatches.Polygon(xy,
                                                facecolor='blue',
                                                edgecolor='black'))
            # Add the patch to the axes
            self.ax.add_patch(self.handle[0]) 
        else:
            # Update polygon
            self.handle[0].set_xy(xy)         

    def drawPanel(self, phi):
            # points that define the base
            pts = np.array([
                [-self.length, -self.width/6.0],
                [self.length, -self.width/6.0],
                [self.length, self.width/6.0],
                [-self.length, self.width/6.0]]).T
            R = np.array([[np.cos(phi), np.sin(phi)],
                           [-np.sin(phi), np.cos(phi)]])
            pts = R @ pts
            xy = np.array(pts.T)
            # When the class is initialized, a polygon patch
            # object will be created and added to the
            # axes. After initialization, the polygon patch
            # object will only be updated.
            if self.flagInit == True:
                # Create the Rectangle patch and append its
                # handle to the handle list
                self.handle.append(mpatches.Polygon(xy,
                                   facecolor='green',
                                   edgecolor='black'))
                # Add the patch to the axes
                self.ax.add_patch(self.handle[1])  
            else:
                # Update polygon
                self.handle[1].set_xy(xy)  
