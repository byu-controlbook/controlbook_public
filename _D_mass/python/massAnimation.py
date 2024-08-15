import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.widgets import Button
import numpy as np
import massParam as P
# if you are having difficulty with the graphics,
# try using one of the following backends.
# See https://matplotlib.org/stable/users/explain/backends.html
import matplotlib
# matplotlib.use('qtagg')  # requires pyqt or pyside
# matplotlib.use('ipympl')  # requires ipympl
# matplotlib.use('gtk3agg')  # requires pyGObject and pycairo
# matplotlib.use('gtk4agg')  # requires pyGObject and pycairo
# matplotlib.use('gtk3cairo')  # requires pyGObject and pycairo
# matplotlib.use('gtk4cairo')  # requires pyGObject and pycairo
matplotlib.use('tkagg')  # requires TkInter
# matplotlib.use('wxagg')  # requires wxPython

def exit_program(event):
    exit()

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
        self.length = P.length
        self.width = P.width
        # Change the x,y axis limits
        plt.axis([-P.length-P.length/5, 2*P.length, -P.length, 2*P.length])
        plt.plot([-P.length-P.length/5, 2*P.length],
                 [0, 0], 'k--')    # Draw track
        plt.plot([-P.length, -P.length], [0, 2*P.width], 'k')  # Draw wall
        # Create exit button
        self.button_ax = plt.axes([0.8, 0.805, 0.1, 0.075])  # [left, bottom, width, height]
        self.exit_button = Button(self.button_ax, label='Exit', color='r',)
        self.exit_button.label.set_fontweight('bold')
        self.exit_button.label.set_fontsize(18)
        self.exit_button.on_clicked(exit_program)

        # Draw mass is the main function that will call the functions:
    def update(self, x: np.ndarray):
        # Process inputs to function
        z = x[0][0]   # position of mass, m
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
                                                  P.width, P.width, fc='blue', ec='black'))
            self.ax.add_patch(self.handle[0])  # Add the patch to the axes
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
