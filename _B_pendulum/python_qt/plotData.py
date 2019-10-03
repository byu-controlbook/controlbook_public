import pyqtgraph as pg
#from pyqtgraph.Qt import QtCore, QtGui
import numpy as np

## Switch to using white background and black foreground
pg.setConfigOption('background', 'w')
pg.setConfigOption('foreground', 'k')
green = (12, 142, 23)


class plotData:
    ''' 
        This class plots the time histories for the pendulum data.
    '''

    def __init__(self):
        # Create figure
        self.win = pg.GraphicsWindow()

        # set up plots: rows and columns
        self.z_plot = self.win.addPlot()
        self.win.nextRow()
        self.theta_plot = self.win.addPlot()
        self.win.nextRow()
        self.force_plot = self.win.addPlot()
        # add title and labels
        self.win.setWindowTitle('Pendulum Data')
        self.z_plot.setLabel('left', "position", units='m')
        self.theta_plot.setLabel('left', "angle", units='deg')
        self.force_plot.setLabel('left', "force", units='N')
        self.force_plot.setLabel('bottom', "time", units='s')
        # Use automatic downsampling and clipping to reduce the drawing load
        self.z_plot.setDownsampling(mode='peak')
        self.theta_plot.setDownsampling(mode='peak')
        self.force_plot.setDownsampling(mode='peak')
        self.z_plot.setClipToView(True)
        self.theta_plot.setClipToView(True)
        self.force_plot.setClipToView(True)
#        p3.setRange(xRange=[-100, 0])
#        p3.setLimits(xMax=0)
        # initialize curves and define their colors
        self.z_curve = self.z_plot.plot(pen='b')
        self.zref_curve = self.z_plot.plot(pen=green)
        self.theta_curve = self.theta_plot.plot(pen='b')
        self.force_curve = self.force_plot.plot(pen='b')

        # Instantiate arrays to hold the time and data histories
        self.time_history = np.empty(100)  # time
        self.zref_history = np.empty(100)  # reference position z_r
        self.z_history = np.empty(100)  # position z
        self.theta_history = np.empty(100)  # angle theta
        self.force_history = np.empty(100)  # control force
        self.ptr = 0  # pointer to current location in histories


    def updatePlots(self, t, reference, states, ctrl):
        '''
            Add to the time and data histories, and update the plots.
        '''
        # update the time curve of all plot variables
        self.time_history[self.ptr] = t  # time
        self.zref_history[self.ptr] = reference[0]  # reference base position
        self.z_history[self.ptr] = states[0]  # base position
        self.theta_history[self.ptr] = 180.0/np.pi*states[1]  # rod angle (converted to degrees)
        self.force_history[self.ptr] = ctrl[0]  # force on the base
        self.ptr += 1
        # add data to the buffers if needed, doubling each time
        data_length = self.time_history.shape[0]
        if self.ptr >= data_length:
            tmp = self.time_history
            self.time_history = np.empty(data_length * 2)
            self.time_history[:data_length] = tmp
            tmp = self.zref_history
            self.zref_history = np.empty(data_length * 2)
            self.zref_history[:data_length] = tmp
            tmp = self.z_history
            self.z_history = np.empty(data_length * 2)
            self.z_history[:data_length] = tmp
            tmp = self.theta_history
            self.theta_history = np.empty(data_length * 2)
            self.theta_history[:data_length] = tmp
            tmp = self.force_history
            self.force_history = np.empty(data_length * 2)
            self.force_history[:data_length] = tmp

        # update the plots with associated histories
        self.z_curve.setData(self.time_history[:self.ptr], self.z_history[:self.ptr])
        self.zref_curve.setData(self.time_history[:self.ptr], self.zref_history[:self.ptr])
        self.theta_curve.setData(self.time_history[:self.ptr], self.theta_history[:self.ptr])
        self.force_curve.setData(self.time_history[:self.ptr], self.force_history[:self.ptr])

