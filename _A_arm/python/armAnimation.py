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
import sys
sys.path.append('..')  # add parent directory
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import armParam as P
# if you are having difficulty with the graphics, 
# try using one of the following backends
# import matplotlib
# matplotlib.use('qtagg')  # requires pyqt or pyside
# matplotlib.use('ipympl')  # requires ipympl
# matplotlib.use('gtk3agg')  # requires pyGObject and pycairo
# matplotlib.use('gtk4agg')  # requires pyGObject and pycairo
# matplotlib.use('gtk3cairo')  # requires pyGObject and pycairo
# matplotlib.use('gtk4cairo')  # requires pyGObject and pycairo
# matplotlib.use('tkagg')  # requires TkInter
# matplotlib.use('wxagg')  # requires wxPython



class armAnimation:
    def __init__(self):
        # Used to indicate initialization
        self.flagInit = True
        # Initializes a figure and axes object
        self.fig, self.ax = plt.subplots()
        # Initializes a list object that will be used to
        # contain handles to the patches and line objects.
        self.handle = []
        self.length=P.length
        self.width=P.width
        # Change the x,y axis limits
        plt.axis([-2.0*P.length, 2.0*P.length, -2.0*P.length,
                  2.0*P.length])
        # Draw a base line
        plt.plot([0, P.length], [0, 0],'k--')
        # Draw pendulum is the main function that will call the
        # functions:
        # drawCart, drawCircle, and drawRod to create the animation.

    def update(self, u):
        # Process inputs to function
        theta = u[0][0]   # angle of arm, rads
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


        plt.draw()
