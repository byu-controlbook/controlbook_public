import pyqtgraph as pg
from pyqtgraph import Point
from pyqtgraph import QtGui
#
#
# import matplotlib.pyplot as plt
# import matplotlib.patches as mpatches
import numpy as np 
import pendulumParam as P

painter = QtGui.QPainter()
app=QtGui.QApplication([])


class pendulumAnimation:
    '''
        Create pendulum animation
    '''
    def __init__(self):
        # set up animation window
        self.window = pg.GraphicsWindow(border=0.5)
        self.window.resize(500, 500)  # size of the window
        self.window.setWindowTitle('Pendulum Animation')
        self.window.show()

        self.flagInit = True                  # Used to indicate initialization
        # plt.axis([-3*P.ell,3*P.ell, -0.1, 3*P.ell]) # Change the x,y axis limits
        # plt.plot([-2*P.ell,2*P.ell],[0,0],'b--')    # Draw a base line
        # plt.xlabel('z')
        painter.drawLine(-2*P.ell, 0, 2*P.ell, 0)
        self.cart = painter.drawRect(-P.w/2.0, P.gap, P.w, P.h)





        # Draw pendulum is the main function that will call the functions:
        # drawCart, drawCircle, and drawRod to create the animation.
    def drawPendulum(self, u):
        # Process inputs to function
        z = u[0]        # Horizontal position of cart, m
        theta = u[1]   # Angle of pendulum, rads

#        self.drawCart(z)
#         self.drawCircle(z, theta)
#         self.drawRod(z, theta)
# #        self.ax.axis('equal') # This will cause the image to not distort

        # After each function has been called, initialization is over.
        if self.flagInit == True:
            self.flagInit = False

    def drawCart(self, z):
        # x = z-P.w/2.0  # x coordinate
        # y = P.gap      # y coordinate
        # xy = (x, y)     # Bottom left corner of rectangle

        # When the class is initialized, a Rectangle patch object will be
        # created and added to the axes. After initialization, the Rectangle
        # patch object will only be updated.
        # if self.flagInit == True:
        #     # Create the Rectangle patch and append its handle
        #     # to the handle list
        #     self.handle.append(mpatches.Rectangle(xy,
        #         P.w,P.h, fc = 'blue', ec = 'black'))
        #     self.ax.add_patch(self.handle[0]) # Add the patch to the axes
        # else:
        #     self.handle[0].set_xy(xy)         # Update patch
        self.cart

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



# Used see the animation from the command line
if __name__ == "__main__":

    simAnimation = PendulumAnimation()    # Create Animate object
    z = 0.0                               # Position of cart, m
    theta = 0.0*np.pi/180                 # Angle of pendulum, rads
    simAnimation.drawPendulum([z, theta, 0, 0])  # Draw the pendulum
    #plt.show()
    # Keeps the program from closing until the user presses a button.
    print('Press key to close')
    plt.waitforbuttonpress()
    plt.close()