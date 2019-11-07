import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np 
import armParam as P


class armAnimation:
    '''
        Create arm animation
    '''
    def __init__(self):
        self.flagInit = True                  # Used to indicate initialization
        self.fig, self.ax = plt.subplots()    # Initializes a figure and axes object
        self.handle = []                      # Initializes a list object that will
                                              # be used to contain handles to the
                                              # patches and line objects.
        self.length=P.length
        self.width=P.width
        plt.axis([-2.0*P.length, 2.0*P.length, -2.0*P.length, 2.0*P.length]) # Change the x,y axis limits
        plt.plot([0, P.length], [0, 0],'k--')    # Draw a base line

        # Draw pendulum is the main function that will call the functions:
        # drawCart, drawCircle, and drawRod to create the animation.
    def update(self, u):
        # Process inputs to function
        theta = u[0]   # angle of arm, rads

        X = [0, self.length*np.cos(theta)]  # X data points
        Y = [0, self.length*np.sin(theta)]  # Y data points

        # When the class is initialized, a line object will be
        # created and added to the axes. After initialization, the
        # line object will only be updated.
        if self.flagInit == True:
            # Create the line object and append its handle
            # to the handle list.
            line, =self.ax.plot(X, Y, lw=5, c='blue')
            self.handle.append(line)
            self.flagInit=False
        else:
            self.handle[0].set_xdata(X)   # Update the line
            self.handle[0].set_ydata(Y)


# Used see the animation from the command line
if __name__ == "__main__":

    simAnimation = armAnimation()    # Create Animate object
    theta = 0.0*np.pi/180                 # Angle of arm, rads
    simAnimation.drawArm([z, theta, 0, 0])  # Draw the arm
    #plt.show()
    # Keeps the program from closing until the user presses a button.
    print('Press key to close')
    plt.waitforbuttonpress()
    plt.close()