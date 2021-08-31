import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np 
import ballbeamParam as P


class ballbeamAnimation:
    '''
        Create ballbeam animation
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

        # Draw ballbeam is the main function that will call the functions:
        # drawCart, drawCircle, and drawRod to create the animation.
    def update(self, u):
        # Process inputs to function
        z = u.item(0)        # Horizontal position of cart, m
        theta = u.item(1)   # Angle of ballbeam, rads

        self.drawBall(z, theta)
        self.drawBeam(theta)
        self.ax.axis('equal') # This will cause the image to not distort

        # After each function has been called, initialization is over.
        if self.flagInit == True:
            self.flagInit = False

    def drawBall(self, z, theta):
        x = z*np.cos(theta) - P.radius*np.sin(theta)
        y = z*np.sin(theta) + P.radius*np.cos(theta)
        xy = (x, y)  # Center of circle

        # When the class is initialized, a CirclePolygon patch object will
        # be created and added to the axes. After initialization, the
        # CirclePolygon patch object will only be updated.
        if self.flagInit == True:
            # Create the CirclePolygon patch and append its handle
            # to the handle list
            self.handle.append(mpatches.CirclePolygon(xy,
                radius=P.radius, resolution=25,
                fc='limegreen', ec='black'))
            self.ax.add_patch(self.handle[0])  # Add the patch to the axes
        else:
            self.handle[0]._xy = xy

    def drawBeam(self, theta):
        X = [0, P.length*np.cos(theta)]  # X data points
        Y = [0, P.length*np.sin(theta)]  # Y data points

        # When the class is initialized, a line object will be
        # created and added to the axes. After initialization, the
        # line object will only be updated.
        if self.flagInit == True:
            # Create the line object and append its handle
            # to the handle list.
            line, = self.ax.plot(X, Y, lw=2, c='black')
            self.handle.append(line)
        else:
            self.handle[1].set_xdata(X)               # Update the line
            self.handle[1].set_ydata(Y)
