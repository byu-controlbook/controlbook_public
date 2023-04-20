import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np 
import rodMassParam as P


class rodMassAnimation:
    def __init__(self):
        self.flag_init = True                  # Used to indicate initialization
        self.fig, self.ax = plt.subplots()    # Initializes a figure and axes object
        self.handle = []                      # Initializes a list object that will
                                              # be used to contain handles to the
                                              # patches and line objects.
        self.L = 1.0
        self.w = 0.01
        self.R = 0.1
        self.H = 1.0
        # add wall
        corner = (-self.H/3.0, -self.H)  # bottom left corner
        self.handle.append(
            mpatches.Rectangle(corner, width=self.H/3.0, height=2.0*self.H,
                               fc='blue', ec='black'))
        # Add the patch to the axes
        self.ax.add_patch(self.handle[0])
        # draw zero line
        plt.axis([-2.0*self.L, 2.0*self.L, -2.0*self.L, 2.0*self.L])
        plt.plot([0, self.L], [0, 0],'k--')

    def update(self, state):
        theta = state.item(0)
        self.draw_rod(theta)
        self.draw_bob(theta)
        self.ax.axis('equal')
        # Set initialization flag to False after first call
        if self.flag_init == True:
            self.flag_init = False

    def draw_bob(self, theta):
        # specify center of circle
        x = (self.L + self.R) * np.cos(theta)
        y = (self.L + self.R) * np.sin(theta)
        center = (x,y)
        # create circle on first call, update on subsequent calls
        if self.flag_init == True:
            # Create the CirclePolygon patch and append its handle
            # to the handle list
            self.handle.append(
                mpatches.CirclePolygon(center, radius=self.R,
                    resolution=15, fc='limegreen', ec='black'))
            # Add the patch to the axes
            self.ax.add_patch(self.handle[2])
        else:
            self.handle[2].xy = center

    def draw_rod(self, theta):
        # specify x-y points of the rod
        pts = np.array([[0, self.L, self.L, 0],
                        [-self.w/2, -self.w/2, self.w/2, self.w/2]])
        R = np.array([[np.cos(theta), -np.sin(theta)],
                      [np.sin(theta), np.cos(theta)]])
        pts = R @ pts
        X = pts[0]
        Y = pts[1]
        # create rod on first call, update on subsequent calls
        if self.flag_init == True:
            # Create the line object and append its handle
            # to the handle list.
            line, =self.ax.plot(X, Y, lw=1, c='black')
            self.handle.append(line)
        else:
            self.handle[1].set_xdata(X)
            self.handle[1].set_ydata(Y)


# Used see the animation from the command line
if __name__ == "__main__":

    simAnimation = rodMassAnimation()
    theta = 45.0*np.pi/180
    state = np.array([[theta],[0]])
    simAnimation.update(state)
    #plt.show()
    # Keeps the program from closing until the user presses a button.
    print('Press key to close')
    plt.waitforbuttonpress()
    plt.close()