import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.widgets import Button
import numpy as np
import armParam as P
import signal

# if you are having difficulty with the graphics,
# try using one of the following backends
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
        plt.axis([-2.0*P.length, 
                  2.0*P.length, 
                  -2.0*P.length, 
                  2.0*P.length])
        # Draw a base line
        plt.plot([0, P.length], [0, 0],'k--')

        # Create exit button
        # [left, bottom, width, height]
        self.button_ax = plt.axes([0.8, 0.805, 0.1, 0.075])  
        self.exit_button = Button(self.button_ax, label='Exit', color='r',)
        self.exit_button.label.set_fontweight('bold')
        self.exit_button.label.set_fontsize(18)
        self.exit_button.on_clicked(lambda event: exit())

        # Register <ctrl+c> signal handler to stop the simulation
        signal.signal(signal.SIGINT, signal.SIG_DFL)

    def update(self, x):
        # Process inputs to function
        theta = x[0, 0]   # angle of arm, rads
        X = [0, self.length*np.cos(theta)]  # X data points
        Y = [0, self.length*np.sin(theta)]  # Y data points
        # When the class is initialized, a line object will be
        # created and added to the axes. After initialization, the
        # line object will only be updated.
        if self.flagInit == True:
            # Create the line object and append its handle
            # to the handle list.
            line, = self.ax.plot(X, Y, lw=5, c='blue')
            self.handle.append(line)
            self.flagInit=False
        else:
            self.handle[0].set_xdata(X)   # Update the line
            self.handle[0].set_ydata(Y)
        plt.draw()
