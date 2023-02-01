"""
hummingbirdAnimation
        12/2022 - R.W. Beard
"""
import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import pyqtgraph.Vector as Vector


class HummingbirdAnimation:
    def __init__(self):
        # initialize Qt gui application and window
        self.app = pg.QtWidgets.QApplication([])  # initialize QT
        self.window = gl.GLViewWidget()  # initialize the view object
        self.window.setWindowTitle('Hummingbird Viewer')
        self.window.setGeometry(800, 200, 700, 700)  # args: upper_left_x, upper_right_y, width, height
        grid = gl.GLGridItem() # make a grid to represent the ground
        grid.scale(20, 20, 20) # set the size of the grid (distance between each line)
        self.window.addItem(grid) # add grid to viewer
        self.window.setCameraPosition(distance=15) # distance from center of plot to camera
        self.window.setBackgroundColor('k')  # set background color to black
        self.window.show()  # display configured window
        self.window.raise_() # bring window to the front
        self.plot_initialized = False # has the mav been plotted yet?
        self.vtol_plot = []

    def update(self, t, state):
        # initialize the drawing the first time update() is called
        if not self.plot_initialized:
            self.hummingbird_plot = DrawHummingbird(state, self.window)
            self.plot_initialized = True
        # else update drawing on all other calls to update()
        else:
            self.hummingbird_plot.update(t, state)
        # update the center of the camera view to the hummingbird location
        # defined in ENU coordinates
        view_location = Vector(0, 0, 0)
        self.window.opts['center'] = view_location
        # redraw
        self.app.processEvents()


class DrawHummingbird():
    def __init__(self, state, window):
        """
        Draw the hummingbird
            state = [ phi, theta, psi, phidot, thetadot, psidot]
        """
        unit_length = 4.0
        self.base_width = 0.1 * unit_length
        self.base_height = 0.1 * unit_length
        self.base_length = 0.6 * unit_length
        self.cw_width = 0.2 * unit_length  # cw==counterweight
        self.cw_height = 0.2 * unit_length
        self.cw_length = 0.4 * unit_length
        self.body_width = 0.1 * unit_length
        self.body_height = 0.1 * unit_length
        self.body_length = 0.4 * unit_length
        self.arm_width = 0.05 * unit_length
        self.arm_height = 0.05 * unit_length
        self.arm_length = 0.8 * unit_length
        self.motor_width = 0.6 * self.arm_width
        self.motor_height = 0.6 * self.arm_height
        self.motor_length = 2 * self.motor_width
        phi = state[0][0]
        theta = state[1][0]
        psi = state[2][0]
        # base
        self.base_points, self.base_index, self.base_meshColors \
            = self.get_box_points(self.base_width,
                                  self.base_height,
                                  self.base_length)
        base_position = np.array([[0], [0], [-self.base_length/2]])
        base_rotation = Euler2Rotation(0, np.pi/2, 0)
        self.base = self.add_object(
            self.base_points,
            self.base_index,
            self.base_meshColors,
            base_rotation,
            base_position)
        window.addItem(self.base)  # add base to plot
        # counterweight
        self.cw_points, self.cw_index, self.cw_meshColors \
            = self.get_box_points(self.cw_width,
                                  self.cw_height,
                                  self.cw_length)
        self.counterweight = self.add_object(
            self.cw_points,
            self.cw_index,
            self.cw_meshColors,
            np.eye(3),
            np.array([[0], [0], [0]]))
        window.addItem(self.counterweight)  # add counterweight to plot
        # body
        self.body_points, self.body_index, self.body_meshColors \
            = self.get_box_points(self.body_width,
                                  self.body_height,
                                  self.body_length)
        self.body = self.add_object(
            self.body_points,
            self.body_index,
            self.body_meshColors,
            np.eye(3),
            np.array([[0], [0], [0]]))
        window.addItem(self.body)  # add body to plot
        # arm
        self.arm_points, self.arm_index, self.arm_meshColors \
            = self.get_box_points(self.arm_width,
                                  self.arm_height,
                                  self.arm_length)
        self.arm = self.add_object(
            self.arm_points,
            self.arm_index,
            self.arm_meshColors,
            np.eye(3),
            np.array([[0], [0], [0]]))
        window.addItem(self.arm)  # add arm to plot
        # motors
        self.motor_points, self.motor_index, self.motor_meshColors \
            = self.get_box_points(self.motor_width,
                                  self.motor_height,
                                  self.motor_length)
        self.motor1 = self.add_object(
            self.motor_points,
            self.motor_index,
            self.motor_meshColors,
            np.eye(3),
            np.array([[0], [0], [0]]))
        window.addItem(self.motor1)
        self.motor2 = self.add_object(
            self.motor_points,
            self.motor_index,
            self.motor_meshColors,
            np.eye(3),
            np.array([[0], [0], [0]]))
        window.addItem(self.motor2)
        # rotors
        self.rotor_points, self.rotor_index, self.rotor_meshColors \
            = self.get_rotor_points()
        self.rotor1 = self.add_object(
            self.rotor_points,
            self.rotor_index,
            self.rotor_meshColors,
            np.eye(3),
            np.array([[0], [0], [0]]))
        window.addItem(self.rotor1)
        self.rotor2 = self.add_object(
            self.rotor_points,
            self.rotor_index,
            self.rotor_meshColors,
            np.eye(3),
            np.array([[0], [0], [0]]))
        window.addItem(self.rotor2)

    def update(self, t, state):
        phi = state[0][0]
        theta = state[1][0]
        psi = state[2][0]
        # counterweight
        cw_position = np.array([[0], [0], [-self.base_length]])
        cw_rotation = Euler2Rotation(0, theta, psi)
        self.counterweight = self.update_object(
            self.counterweight,
            self.cw_points,
            self.cw_index,
            self.cw_meshColors,
            cw_rotation,
            cw_position)
        # body
        body_position = cw_position \
                        + cw_rotation \
                          @ np.array([[self.cw_length/2+self.body_length/2],
                                      [0], [0]])
        body_rotation = Euler2Rotation(phi, theta, psi)
        self.body = self.update_object(
            self.body,
            self.body_points,
            self.body_index,
            self.body_meshColors,
            body_rotation,
            body_position)
        # arm
        arm_position = body_position \
                        + body_rotation \
                          @ np.array([[self.body_length/2], [0], [0]])
        arm_rotation = Euler2Rotation(phi, theta, psi) \
                       @ Euler2Rotation(0, 0, np.pi/2)
        self.arm = self.update_object(
            self.arm,
            self.arm_points,
            self.arm_index,
            self.arm_meshColors,
            arm_rotation,
            arm_position)
        # motors
        motor1_position = arm_position \
                               + arm_rotation \
                                 @ np.array([[self.arm_length/2-self.motor_width/2],
                                             [0],
                                             [-self.arm_height/2-self.motor_length/2]])
        motor1_rotation = Euler2Rotation(phi, theta, psi) \
                          @ Euler2Rotation(0, np.pi/2, 0)
        self.motor1 = self.update_object(
            self.motor1,
            self.motor_points,
            self.motor_index,
            self.motor_meshColors,
            motor1_rotation,
            motor1_position)
        motor2_position = arm_position \
                               + arm_rotation \
                                 @ np.array([[-self.arm_length/2+self.motor_width/2],
                                             [0],
                                             [-self.arm_height/2-self.motor_length/2]])
        motor2_rotation = Euler2Rotation(phi, theta, psi) \
                          @ Euler2Rotation(0, np.pi/2, 0)
        self.motor2 = self.update_object(
            self.motor2,
            self.motor_points,
            self.motor_index,
            self.motor_meshColors,
            motor2_rotation,
            motor2_position)
        # rotors
        rotor1_position = arm_position \
                               + arm_rotation \
                                 @ np.array([[self.arm_length/2-self.motor_width/2],
                                             [0],
                                             [-self.arm_height/2-self.motor_length]])
        rotor1_rotation = Euler2Rotation(phi, theta, psi) \
                          @ Euler2Rotation(0, 0, 10*t)
        self.rotor1 = self.update_object(
            self.rotor1,
            self.rotor_points,
            self.rotor_index,
            self.rotor_meshColors,
            rotor1_rotation,
            rotor1_position)
        rotor2_position = arm_position \
                               + arm_rotation \
                                 @ np.array([[-self.arm_length/2+self.motor_width/2],
                                             [0],
                                             [-self.arm_height/2-self.motor_length]])
        rotor2_rotation = Euler2Rotation(phi, theta, psi) \
                          @ Euler2Rotation(0, 0, -10*t)
        self.rotor2 = self.update_object(
            self.rotor2,
            self.rotor_points,
            self.rotor_index,
            self.rotor_meshColors,
            rotor2_rotation,
            rotor2_position)

    def add_object(self, points, index, colors, R, position):
        rotated_points = self.rotate_points(points, R)
        translated_points = self.translate_points(rotated_points, position)
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        translated_points = R @ translated_points
        # convert points to triangular mesh defined as array of three 3D points (Nx3x3)
        mesh = self.points_to_mesh(translated_points, index)
        object = gl.GLMeshItem(
            vertexes=mesh,  # defines the triangular mesh (Nx3x3)
            vertexColors=colors,  # defines mesh colors (Nx1)
            drawEdges=True,  # draw edges between mesh elements
            smooth=False,  # speeds up rendering
            computeNormals=False)  # speeds up rendering
        return object

    def update_object(self, object, points, index, colors, R, position):
        rotated_points = self.rotate_points(points, R)
        translated_points = self.translate_points(rotated_points, position)
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        translated_points = R @ translated_points
        # convert points to triangular mesh defined as array of three 3D points (Nx3x3)
        mesh = self.points_to_mesh(translated_points, index)
        object.setMeshData(vertexes=mesh, vertexColors=colors)
        return object

    def rotate_points(self, points, R):
        "Rotate points by the rotation matrix R"
        rotated_points = R @ points
        return rotated_points

    def translate_points(self, points, translation):
        "Translate points by the vector translation"
        translated_points = points + np.dot(translation, np.ones([1, points.shape[1]]))
        return translated_points

    def points_to_mesh(self, points, index):
        """"
        Converts points to triangular mesh
        Each mesh face is defined by three 3D points
          (a rectangle requires two triangular mesh faces)
        """
        points = points.T
        mesh = np.array([[points[index[0,0]],points[index[0,1]],points[index[0,2]]]])
        for i in range(1, index.shape[0]):
            tmp = np.array([[points[index[i,0]], points[index[i,1]], points[index[i,2]]]])
            mesh = np.concatenate((mesh, tmp), axis=0)
        return mesh

    def get_box_points(self, width, height, length):
        # define box with dimensions width, height, length
        # points are in ENU coordinates
        points = np.array([
            [length / 2, -width / 2, -height / 2],  # 0
            [length / 2, width / 2, -height / 2],  # 1
            [length / 2, width / 2, height / 2],  # 2
            [length / 2, -width / 2, height / 2],  # 3
            [-length / 2, -width / 2, -height / 2],  # 4
            [-length / 2, width / 2, -height / 2],  # 5
            [-length / 2, width / 2, height / 2],  # 6
            [-length / 2, -width / 2, height / 2],  # 7
            ]).T

        # point index that defines the mesh
        index = np.array([
            [6, 3, 2],  # top
            [6, 3, 7],  # top
            [7, 0, 3],  # right side
            [7, 0, 4],  # right side
            [6, 1, 2],  # left side
            [6, 1, 5],  # left side
            [5, 0, 1],  # bottom
            [5, 0, 4],  # bottom
            [3, 1, 2],  # front
            [3, 1, 0],  # front
            [7, 5, 6],  # back
            [7, 5, 4],  # back
        ])

        #   define the colors for each face of triangular mesh
        # #red = np.array([1., 0., 0., 1])
        # red = np.array([211, 68, 63, 256])/256
        # #green = np.array([0., 1., 0., 1])
        # green = np.array([63, 211, 105, 256])/256.
        # blue = np.array([0., 0., 1., 1])
        # yellow = np.array([1., 1., 0., 1])
        mygrey1 = np.array([0.8, 0.8, 0.8, 1])  # light
        mygrey2 = np.array([0.6, 0.6, 0.6, 1])
        mygrey3 = np.array([0.5, 0.5, 0.5, 1])
        mygrey4 = np.array([0.3, 0.3, 0.3, 1])  # dark
        meshColors = np.empty((12, 3, 4), dtype=np.float32)
        meshColors[0] = mygrey1  # top
        meshColors[1] = mygrey1  # top
        meshColors[2] = mygrey1  # right side
        meshColors[3] = mygrey1  # right side
        meshColors[4] = mygrey1  # left side
        meshColors[5] = mygrey1  # left side
        meshColors[6] = mygrey4  # bottom
        meshColors[7] = mygrey4  # bottom
        meshColors[8] = mygrey2  # front
        meshColors[9] = mygrey2  # front
        meshColors[10] = mygrey3  # back
        meshColors[11] = mygrey3  # back
        return points, index, meshColors

    def get_rotor_points(self):
        radius = 5 * self.arm_width
        N = 10
        points = np.array([[0, 0, 0]])
        theta = 0
        while theta <= 2*np.pi:
            theta += 2 * np.pi / N
            new_point = np.array([[radius*np.cos(theta), radius*np.sin(theta), 0]])
            points = np.concatenate((points, new_point), axis=0)
        mygrey4 = np.array([0.3, 0.3, 0.3, 1])
        index = np.array([[0, 1, 2]])
        meshColors = np.empty((points.shape[0]-1, 3, 4))
        for i in range(1, (points.shape[0]-1)):
            new_mesh = np.array([[0, i, i+1]])
            index = np.concatenate((index, new_mesh), axis=0)
            meshColors[i] = mygrey4
        return points.T, index, meshColors


def Euler2Rotation(phi, theta, psi):
    """
    Converts euler angles to rotation matrix (R_b^i)
    """
    c_phi = np.cos(phi)
    s_phi = np.sin(phi)
    c_theta = np.cos(theta)
    s_theta = np.sin(theta)
    c_psi = np.cos(psi)
    s_psi = np.sin(psi)
    R_roll = np.array([[1, 0, 0],
                       [0, c_phi, -s_phi],
                       [0, s_phi, c_phi]])
    R_pitch = np.array([[c_theta, 0, s_theta],
                        [0, 1, 0],
                        [-s_theta, 0, c_theta]])
    R_yaw = np.array([[c_psi, -s_psi, 0],
                      [s_psi, c_psi, 0],
                      [0, 0, 1]])
    R = R_yaw @ R_pitch @ R_roll
    return R