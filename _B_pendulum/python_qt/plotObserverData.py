import pyqtgraph as pg
import numpy as np


class plotObserverData:
    ''' 
        This class plots the time histories for the pendulum states and state estimates.
    '''

    def __init__(self):
        # Create figure
        self.win = pg.GraphicsWindow()

        # set up plots: rows and columns
        self.z_plot = self.win.addPlot()
        self.win.nextRow()
        self.theta_plot = self.win.addPlot()
        self.win.nextRow()
        self.z_dot_plot = self.win.addPlot()
        self.win.nextRow()
        self.theta_dot_plot = self.win.addPlot()
        # add title and labels
        self.win.setWindowTitle('Pendulum Observer Data')
        self.z_plot.setLabel('left', "position", units='m')
        self.theta_plot.setLabel('left', "angle", units='deg')
        self.z_dot_plot.setLabel('left', "velocity", units='m/s')
        self.theta_dot_plot.setLabel('left', "angular velocity", units='deg/s')
        self.theta_dot_plot.setLabel('bottom', "time", units='s')
        # Use automatic downsampling and clipping to reduce the drawing load
        self.z_plot.setDownsampling(mode='peak')
        self.theta_plot.setDownsampling(mode='peak')
        self.z_dot_plot.setDownsampling(mode='peak')
        self.theta_dot_plot.setDownsampling(mode='peak')
        self.z_plot.setClipToView(True)
        self.theta_plot.setClipToView(True)
        self.z_dot_plot.setClipToView(True)
        self.theta_dot_plot.setClipToView(True)
        # initialize curves and define their colors
        self.z_curve = self.z_plot.plot(pen='b')
        self.zhat_curve = self.z_plot.plot(pen='r')
        self.theta_curve = self.theta_plot.plot(pen='b')
        self.thetahat_curve = self.theta_plot.plot(pen='r')
        self.z_dot_curve = self.z_dot_plot.plot(pen='b')
        self.zhat_dot_curve = self.z_dot_plot.plot(pen='r')
        self.theta_dot_curve = self.theta_dot_plot.plot(pen='b')
        self.thetahat_dot_curve = self.theta_dot_plot.plot(pen='r')

        # Instantiate arrays to hold the time and data histories
        self.time_history = np.empty(100)  # time
        self.z_history = np.empty(100)  # position z
        self.zhat_history = np.empty(100)  # estimated position
        self.theta_history = np.empty(100)  # angle
        self.thetahat_history = np.empty(100)  # estimated angle
        self.z_dot_history = np.empty(100)  # velocity
        self.zhat_dot_history = np.empty(100)  # estimated velocity
        self.theta_dot_history = np.empty(100)  # angular velocity
        self.thetahat_dot_history = np.empty(100)  # estimated angular velocity
        self.ptr = 0  # pointer to current location in histories


    def updatePlots(self, t, x, xhat):
        '''
            Add to the time and data histories, and update the plots.
        '''

        # update the time curve of all plot variables
        self.time_history[self.ptr] = t  # time
        self.z_history[self.ptr] = x[0]  # position
        self.zhat_history[self.ptr] = xhat[0]  # estimated position
        self.theta_history[self.ptr] = 180.0/np.pi*x[1]  # rod angle
        self.thetahat_history[self.ptr] = 180.0/np.pi*xhat[1]  # estimted rod angle
        self.z_dot_history[self.ptr] = x[2]  # velocity
        self.zhat_dot_history[self.ptr] = xhat[2]  # estimated velocity
        self.theta_dot_history[self.ptr] = 180.0/np.pi*x[3]  # angular velocity
        self.thetahat_dot_history[self.ptr] = 180.0/np.pi*xhat[3]  # estimted angular velocity
        self.ptr += 1
        # add data to the buffers if needed, doubling each time
        data_length = self.time_history.shape[0]
        if self.ptr >= data_length:
            tmp = self.time_history
            self.time_history = np.empty(data_length * 2)
            self.time_history[:data_length] = tmp
            tmp = self.z_history
            self.z_history = np.empty(data_length * 2)
            self.z_history[:data_length] = tmp
            tmp = self.zhat_history
            self.zhat_history = np.empty(data_length * 2)
            self.zhat_history[:data_length] = tmp
            tmp = self.theta_history
            self.theta_history = np.empty(data_length * 2)
            self.theta_history[:data_length] = tmp
            tmp = self.thetahat_history
            self.thetahat_history = np.empty(data_length * 2)
            self.thetahat_history[:data_length] = tmp
            tmp = self.z_dot_history
            self.z_dot_history = np.empty(data_length * 2)
            self.z_dot_history[:data_length] = tmp
            tmp = self.zhat_dot_history
            self.zhat_dot_history = np.empty(data_length * 2)
            self.zhat_dot_history[:data_length] = tmp
            tmp = self.theta_dot_history
            self.theta_dot_history = np.empty(data_length * 2)
            self.theta_dot_history[:data_length] = tmp
            tmp = self.thetahat_dot_history
            self.thetahat_dot_history = np.empty(data_length * 2)
            self.thetahat_dot_history[:data_length] = tmp

        # update the plots with associated histories
        self.z_curve.setData(self.time_history[:self.ptr], self.z_history[:self.ptr])
        self.zhat_curve.setData(self.time_history[:self.ptr], self.zhat_history[:self.ptr])
        self.theta_curve.setData(self.time_history[:self.ptr], self.theta_history[:self.ptr])
        self.thetahat_curve.setData(self.time_history[:self.ptr], self.thetahat_history[:self.ptr])
        self.z_dot_curve.setData(self.time_history[:self.ptr], self.z_dot_history[:self.ptr])
        self.zhat_dot_curve.setData(self.time_history[:self.ptr], self.zhat_dot_history[:self.ptr])
        self.theta_dot_curve.setData(self.time_history[:self.ptr], self.theta_dot_history[:self.ptr])
        self.thetahat_dot_curve.setData(self.time_history[:self.ptr], self.thetahat_dot_history[:self.ptr])


