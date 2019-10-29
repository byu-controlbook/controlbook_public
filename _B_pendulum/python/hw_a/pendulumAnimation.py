import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np 
import pendulumParam as P

class pendulumAnimation:
    def __init__(self):
        self.flagInit = True  # Used to indicate initialization
        # Initialize a figure and axes object
        self.fig, self.ax = plt.subplots()
        # Initializes a list of objects (patches and lines)
        self.handle = []
        # Specify the x,y axis limits
        plt.axis([-3*P.ell,3*P.ell, -0.1, 3*P.ell])
        # Draw line for the ground
        plt.plot([-2*P.ell,2*P.ell],[0,0],'b--')
        # label axes
        plt.xlabel('z')

    def update(self, u):
        # Process inputs to function
        z = u[0]        # Horizontal position of cart, m
        theta = u[1]   # Angle of pendulum, rads

        self.drawCart(z)
        self.drawCircle(z, theta)
        self.drawRod(z, theta)
        self.ax.axis('equal') # This will cause the image to not distort

        # After each function has been called, initialization is over.
        if self.flagInit == True:
            self.flagInit = False

    def drawCart(self, z):
        x = z-P.w/2.0  # x coordinate
        y = P.gap      # y coordinate
        xy = (x, y)     # Bottom left corner of rectangle

        # When the class is initialized, a Rectangle patch object will be
        # created and added to the axes. After initialization, the Rectangle
        # patch object will only be updated.
        if self.flagInit == True:
            # Create the Rectangle patch and append its handle
            # to the handle list
            self.handle.append(mpatches.Rectangle(xy,
                P.w,P.h, fc = 'blue', ec = 'black'))
            self.ax.add_patch(self.handle[0]) # Add the patch to the axes
        else:
            self.handle[0].set_xy(xy)         # Update patch

    def drawCircle(self, z, theta):
        x = z+(P.ell+P.radius)*np.sin(theta)         # x coordinate
        y = P.gap+P.h+(P.ell+P.radius)*np.cos(theta) # y coordinate
        xy = (x,y)                                   # Center of circle

        # When the class is initialized, a CirclePolygon patch object will
        # be created and added to the axes. After initialization, the
        # CirclePolygon patch object will only be updated.
        if self.flagInit == True:
            # Create the CirclePolygon patch and append its handle
            # to the handle list
            self.handle.append(mpatches.CirclePolygon(xy,
                radius = P.radius, resolution = 15,
                fc = 'limegreen', ec = 'black'))
            self.ax.add_patch(self.handle[1])  # Add the patch to the axes
        else:
            self.handle[1]._xy=xy

    def drawRod(self, z, theta):
        X = [z,z+P.ell*np.sin(theta)]                  # X data points
        Y = [P.gap+P.h, P.gap+P.h+P.ell*np.cos(theta)] # Y data points

        # When the class is initialized, a line object will be
        # created and added to the axes. After initialization, the
        # line object will only be updated.
        if self.flagInit == True:
            # Create the line object and append its handle
            # to the handle list.
            line, =self.ax.plot(X,Y,lw = 1, c = 'black')
            self.handle.append(line)
        else:
            self.handle[2].set_xdata(X)               # Update the line
            self.handle[2].set_ydata(Y)