import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np 
import satelliteParam as P


class satelliteAnimation:
    '''
        Create satellite animation
    '''
    def __init__(self):
        self.flagInit = True                  # Used to indicate initialization
        self.fig, self.ax = plt.subplots()    # Initializes a figure and axes object
        self.handle = []                      # Initializes a list object that will
                                              # be used to contain handles to the
                                              # patches and line objects.
        plt.axis([-2.0*P.length, 2.0*P.length, -2.0*P.length, 2.0*P.length])
        plt.plot([-2.0*P.length, 2.0*P.length], [0, 0], 'b--')
        self.length = P.length
        self.width = P.width

    def drawSatellite(self, u):
        # Process inputs to function
        theta = u[0]   # Angle of base, rad
        phi = u[1]     # angle of panel, rad

        self.drawBase(theta)
        self.drawPanel(phi)
#        self.ax.axis('equal') # This will cause the image to not distort

        # After each function has been called, initialization is over.
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
        R = np.matrix([[np.cos(theta), np.sin(theta)],
                       [-np.sin(theta), np.cos(theta)]])
        pts = R*pts
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

    def drawPanel(self, phi):
            # points that define the base
            pts = np.matrix([
                [-self.length, -self.width/6.0],
                [self.length, -self.width/6.0],
                [self.length, self.width/6.0],
                [-self.length, self.width/6.0]]).T
            R = np.matrix([[np.cos(phi), np.sin(phi)],
                           [-np.sin(phi), np.cos(phi)]])
            pts = R * pts
            xy = np.array(pts.T)

            # When the class is initialized, a polygon patch object will be
            # created and added to the axes. After initialization, the polygon
            # patch object will only be updated.
            if self.flagInit == True:
                # Create the Rectangle patch and append its handle
                # to the handle list
                self.handle.append(mpatches.Polygon(xy, facecolor='green', edgecolor='black'))
                self.ax.add_patch(self.handle[1])  # Add the patch to the axes
            else:
                self.handle[1].set_xy(xy)  # Update polygon


# Used see the animation from the command line
if __name__ == "__main__":

    simAnimation = satelliteAnimation()    # Create Animate object
    theta = 0.0*np.pi/180                 # Angle of base rad
    phi = 0.0*np.pi/180                   # angle of panel, rad
    simAnimation.drawSatellite([theta, phi, 0, 0])
    #plt.show()
    # Keeps the program from closing until the user presses a button.
    print('Press key to close')
    plt.waitforbuttonpress()
    plt.close()