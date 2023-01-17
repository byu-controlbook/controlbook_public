import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np 
import blockbeamParam as P
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


class blockbeamAnimation:
    '''
        Create blockbeam animation
    '''
    def __init__(self):
        self.flagInit = True                  # Used to indicate initialization
        self.fig, self.ax = plt.subplots()    # Initializes a figure and axes object
        self.handle = []                      # Initializes a list object that will
                                              # be used to contain handles to the
                                              # patches and line objects.
        plt.axis([-P.length/5, P.length+P.length/5, -P.length, P.length]) # Change the x,y axis limits
        plt.plot([0.0, P.length], [0.0, 0.0], 'k')    # Draw a base line
        #plt.xlabel('z')
    # Draw blockbeam is the main function that will call the functions:
    # drawBlock, drawBeam to create the animation.
    def update(self, x):
        # Process inputs to function
        z = x[0][0]        # Horizontal position of cart, m
        theta = x[1][0]   # Angle of block-beam, rads
        self.drawBlock(z, theta)
        self.drawBeam(theta)
        self.ax.axis('equal') # This will cause the image to not distort
        # After each function has been called, initialization is over.
        if self.flagInit == True:
            self.flagInit = False

    def drawBlock(self, z, theta):
        x = z*np.cos(theta) - P.width/2.0*np.sin(theta)
        y = z*np.sin(theta) + P.height/2.0*np.cos(theta)
        xy = (x, y)  # bottom left of block
        # When the class is initialized, a Rectangle patch object will
        # be created and added to the axes. After initialization, the
        # patch object will only be updated.
        if self.flagInit == True:
            # Create the Rectangle patch and append its handle
            # to the handle list
            self.handle.append(mpatches.Rectangle(xy, width=P.width, 
                                height=P.width*0.25, angle=theta, 
                                fc='limegreen', ec='black'))
            self.ax.add_patch(self.handle[0])  # Add the patch to the axes
        else:
            self.handle[0].xy = xy  # "set" function, or "set_xy" don't work
            self.handle[0].angle = theta*180.0/np.pi

    def drawBeam(self, theta):
        X = [0, P.length*np.cos(theta)]  # X data points
        Y = [0, P.length*np.sin(theta)]  # Y data points
        # When the class is initialized, a line object will be
        # created and added to the axes. After initialization, the
        # line object will only be updated.
        if self.flagInit == True:
            # Create the line object and append its handle
            # to the handle list.
            line, = self.ax.plot(X, Y, lw=3, c='black')
            self.handle.append(line)
        else:
            self.handle[1].set_xdata(X)               # Update the line
            self.handle[1].set_ydata(Y)
