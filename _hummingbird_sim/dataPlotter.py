import matplotlib
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np

plt.ion()  # enable interactive drawing


class DataPlotter:
    ''' 
        This class plots the time histories for the pendulum data.
    '''
    def __init__(self):
        # Number of subplots = num_of_rows*num_of_cols
        self.num_rows = 3    # Number of subplot rows
        self.num_cols = 3    # Number of subplot columns

        # Crete figure and axes handles
        self.fig, self.ax = plt.subplots(self.num_rows,
                                         self.num_cols,
                                         sharex=True)
        # Instantiate lists to hold the time and data histories
        self.time_history = []  # time
        self.phi_history = []  # roll angle phi
        self.theta_history = []  # pitch angle theta
        self.psi_history = []  # yaw angle psi
        self.phi_ref_history = []  # roll reference angle
        self.theta_ref_history = []  # pitch reference angle
        self.psi_ref_history = []  # yaw reference angle
        self.phidot_history = []  # roll rate
        self.thetadot_history = []  # pitch rate
        self.psidot_history = []  # yaw rate
        self.force_history = []  # force
        self.torque_history = []  # torque
        # create a handle for every subplot.
        self.handle = []
        self.handle.append(MyPlot(self.ax[0][0], ylabel='phi(deg)', title='Hummingbird Data'))
        self.handle.append(MyPlot(self.ax[1][0], ylabel='theta(deg)'))
        self.handle.append(MyPlot(self.ax[2][0], ylabel='psi(deg)', xlabel='t(s)'))
        self.handle.append(MyPlot(self.ax[0][1], ylabel='phidot(deg/s)'))
        self.handle.append(MyPlot(self.ax[1][1], ylabel='thetadot(deg/s)'))
        self.handle.append(MyPlot(self.ax[2][1],  ylabel='psidot(deg/s)', xlabel='t(s)'))
        self.handle.append(MyPlot(self.ax[0][2], ylabel='force(N)'))
        self.handle.append(MyPlot(self.ax[1][2], ylabel='torque(Nm)'))

    def update(self, t, state, ref, force, torque):
        '''
            Add to the time and data histories, and update the plots.
        '''
        # update the time history of all plot variables
        self.time_history.append(t)  # time
        self.phi_history.append(180.0/np.pi*state[0][0])  # roll
        self.theta_history.append(180.0/np.pi*state[1][0])  # pitch
        self.psi_history.append(180.0/np.pi*state[2][0])  # psi
        self.phidot_history.append(180.0/np.pi*state[3][0])  # roll rate
        self.thetadot_history.append(180.0/np.pi*state[4][0])  # pitch rate
        self.psidot_history.append(180.0/np.pi*state[5][0])  # psi rate
        self.phi_ref_history.append(180.0 / np.pi * ref[0][0])  # roll
        self.theta_ref_history.append(180.0 / np.pi * ref[1][0])  # pitch
        self.psi_ref_history.append(180.0 / np.pi * ref[2][0])  # psi
        self.force_history.append(force)  # Force
        self.torque_history.append(torque)  # torque
        # update the plots with associated histories
        self.handle[0].update(self.time_history, [self.phi_history,
                                                  self.phi_ref_history])
        self.handle[1].update(self.time_history, [self.theta_history,
                                                  self.theta_ref_history])
        self.handle[2].update(self.time_history, [self.psi_history,
                                                  self.psi_ref_history])
        self.handle[3].update(self.time_history, [self.phidot_history])
        self.handle[4].update(self.time_history, [self.thetadot_history])
        self.handle[5].update(self.time_history, [self.psidot_history])
        self.handle[6].update(self.time_history, [self.force_history])
        self.handle[7].update(self.time_history, [self.torque_history])


class MyPlot:
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
           

