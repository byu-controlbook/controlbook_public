import matplotlib.pyplot as plt 
from matplotlib.lines import Line2D
import numpy as np
import VTOLParam as P

plt.ion()  # enable interactive drawing

class dataPlotter:
    def __init__(self):
        # Number of subplots = num_of_rows*num_of_cols
        self.num_rows = 5    # Number of subplot rows
        self.num_cols = 1    # Number of subplot columns
        # Crete figure and axes handles
        self.fig, self.ax = plt.subplots(self.num_rows, self.num_cols, sharex=True)
        # Instantiate lists to hold the time and data histories
        self.time_history = []  # time
        self.zref_history = []  # reference position z_r
        self.z_history = []  # position z
        self.href_history = []  # reference altitude h_r
        self.h_history = []  # altitude h
        self.thetaref_history = []  # reference angle theta_r
        self.theta_history = []  # angle theta
        self.Force_history = []  # control force
        self.Torque_history = []  # control torque
        # create a handle for every subplot.
        self.handle = []
        self.handle.append(myPlot(self.ax[0], ylabel='z(m)', title='VTOL System Data'))
        self.handle.append(myPlot(self.ax[1], ylabel='h(m)'))
        self.handle.append(myPlot(self.ax[2], ylabel='theta(deg)'))
        self.handle.append(myPlot(self.ax[3], ylabel='force(N)'))
        self.handle.append(myPlot(self.ax[4], xlabel='t(s)', ylabel='torque(Nm)'))

    def update(self, t, states, path, motor_thrusts):
        force = motor_thrusts[0][0]+motor_thrusts[1][0]
        torque = P.d * (motor_thrusts[0][0] - motor_thrusts[1][0])
        z_ref = path[0][0]
        h_ref = path[1][0]
        theta_ref = path[2][0]
        # update the time history of all plot variables
        self.time_history.append(t)  # time
        self.zref_history.append(z_ref)  # reference position
        self.z_history.append(states.item(0))  # position
        self.href_history.append(h_ref)  # reference position
        self.h_history.append(states.item(1))  # position
        self.thetaref_history.append(theta_ref)  # reference angle
        self.theta_history.append(180.0/np.pi*states.item(2))  # VTOL angle (converted to degrees)
        self.Force_history.append(force)  # force
        self.Torque_history.append(torque)  # torque

        # update the plots with associated histories
        self.handle[0].update(self.time_history, [self.z_history, self.zref_history])
        self.handle[1].update(self.time_history, [self.h_history, self.href_history])
        self.handle[2].update(self.time_history, [self.theta_history, self.thetaref_history])
        self.handle[3].update(self.time_history, [self.Force_history])
        self.handle[4].update(self.time_history, [self.Torque_history])


class myPlot:
    ''' 
        Create each individual subplot.
    '''
    def __init__(self, ax,
                 xlabel='',
                 ylabel='',
                 title='',
                 legend=None):
        ''' 
            ax - This is a handle to the  axes of the figure
            xlable - Label of the x-axis
            ylable - Label of the y-axis
            title - Plot title
            legend - A tuple of strings that identify the data. 
                     EX: ("data1","data2", ... , "dataN")
        '''
        self.legend = legend
        self.ax = ax                  # Axes handle
        self.colors = ['b', 'g', 'r', 'c', 'm', 'y', 'b']
        # A list of colors. The first color in the list corresponds
        # to the first line object, etc.
        # 'b' - blue, 'g' - green, 'r' - red, 'c' - cyan, 'm' - magenta
        # 'y' - yellow, 'k' - black
        self.line_styles = ['-', '-', '--', '-.', ':']
        # A list of line styles.  The first line style in the list
        # corresponds to the first line object.
        # '-' solid, '--' dashed, '-.' dash_dot, ':' dotted

        self.line = []

        # Configure the axes
        self.ax.set_ylabel(ylabel)
        self.ax.set_xlabel(xlabel)
        self.ax.set_title(title)
        self.ax.grid(True)

        # Keeps track of initialization
        self.init = True   

    def update(self, time, data):
        ''' 
            Adds data to the plot.  
            time is a list, 
            data is a list of lists, each list corresponding to a line on the plot
        '''
        if self.init == True:  # Initialize the plot the first time routine is called
            for i in range(len(data)):
                # Instantiate line object and add it to the axes
                self.line.append(Line2D(time,
                                        data[i],
                                        color=self.colors[np.mod(i, len(self.colors) - 1)],
                                        ls=self.line_styles[np.mod(i, len(self.line_styles) - 1)],
                                        label=self.legend if self.legend != None else None))
                self.ax.add_line(self.line[i])
            self.init = False
            # add legend if one is specified
            if self.legend != None:
                plt.legend(handles=self.line)
        else: # Add new data to the plot
            # Updates the x and y data of each line.
            for i in range(len(self.line)):
                self.line[i].set_xdata(time)
                self.line[i].set_ydata(data[i])

        # Adjusts the axis to fit all of the data
        self.ax.relim()
        self.ax.autoscale()
           

