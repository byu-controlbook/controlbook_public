import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np 
import massParam as P


class massAnimation:
    '''
        Create mass animation
    '''
    def __init__(self):
        self.flagInit = True                  # Used to indicate initialization
        self.fig, self.ax = plt.subplots()    # Initializes a figure and axes object
        self.handle = []                      # Initializes a list object that will
                                              # be used to contain handles to the
                                              # patches and line objects.
        self.length=P.length
        self.width=P.width
        plt.axis([-P.length-P.length/5, 2*P.length, -P.length, 2*P.length]) # Change the x,y axis limits
        plt.plot([-P.length-P.length/5,2*P.length],[0,0],'k--')    # Draw track
        plt.plot([-P.length, -P.length], [0, 2*P.width], 'k')  # Draw wall

        # Draw mass is the main function that will call the functions:
    def update(self, u):
        # Process inputs to function
        z = u.item(0)   # position of mass, m

        self.drawWeight(z)
        self.drawSpring(z)

        # After each function has been called, initialization is over.
        if self.flagInit == True:
            self.flagInit = False

    def drawWeight(self, z):
        x = z-P.width/2.0  # x coordinate
        y = 0.0      # y coordinate
        xy = (x, y)     # Bottom left corner of rectangle

        # When the class is initialized, a Rectangle patch object will be
        # created and added to the axes. After initialization, the Rectangle
        # patch object will only be updated.
        if self.flagInit == True:
            # Create the Rectangle patch and append its handle
            # to the handle list
            self.handle.append(mpatches.Rectangle(xy,
                P.width, P.width, fc = 'blue', ec = 'black'))
            self.ax.add_patch(self.handle[0]) # Add the patch to the axes
        else:
            self.handle[0].set_xy(xy)         # Update patch

    def drawSpring(self, z):
        X = [-self.length, z-self.width/2.0]  # X data points
        Y = [self.width/2.0, self.width/2.0]  # Y data points

        # When the class is initialized, a line object will be
        # created and added to the axes. After initialization, the
        # line object will only be updated.
        if self.flagInit == True:
            # Create the line object and append its handle
            # to the handle list.
            line, = self.ax.plot(X, Y, lw=1, c='black')
            self.handle.append(line)
        else:
            self.handle[1].set_xdata(X)  # Update the line
            self.handle[1].set_ydata(Y)
