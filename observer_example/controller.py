import numpy as np
import control as cnt
import param as P

class Controller:
    def __init__(self):
        #self.control_mode = 1  # no integrator, no disturbance estimate
        #self.control_mode = 2  # integrator, no disturbance estimate
        #self.control_mode = 3  # no integrator, disturbance estimate
        self.control_mode = 4  # integrator, disturbance estimate
        # pick poles of controller
        wn_ctrl = 1.
        zeta_ctrl = 0.707
        des_ctrl_charpoly = [1, 2. * zeta_ctrl * wn_ctrl, wn_ctrl**2]
        des_ctrl_poles = np.roots(des_ctrl_charpoly)
        integrator_pole = -1.  # pole for integrator
        # pick poles of observer
        wn_obsv = 10.
        zeta_obsv = 0.707
        des_obsv_charpoly = [1, 2. * zeta_obsv * wn_obsv, wn_obsv**2]
        des_obsv_poles = np.roots(des_obsv_charpoly)
        dis_obsv_pole = -1.  # pole for disturbance observer

        if self.control_mode==1: # no integrator, no disturbance estimate
            # is the system controllable?
            if np.linalg.matrix_rank(cnt.ctrb(P.A, P.B)) != 2: 
                print("System Not Controllable") 
            else:
                self.K = cnt.place(P.A, P.B, des_ctrl_poles)
                tmp = P.C @ np.linalg.inv(P.A - P.B @ self.K) @ P.B 
                self.kr = -1.0 / tmp[0][0]
            # is the system observable?
            if np.linalg.matrix_rank(cnt.obsv(P.A, P.C)) != 2: 
                print("System Not Observable") 
            else:   
                self.L = cnt.place(P.A.T, P.C.T, des_obsv_poles).T 
            print('K: ', self.K)
            print('kr: ', self.kr)
            print('L^T: ', self.L.T)
            self.observer_state = np.array([[0.0], [0.0]])
            self.A = P.A
            self.B = P.B
            self.C = P.C
        elif self.control_mode==2: # integrator, no disturbance estimate
            # augment system to add integrator
            A1 = np.concatenate((np.concatenate((P.A, np.zeros((2,1))), axis=1),
                                np.concatenate((-P.C, np.zeros((1,1))), axis=1)), axis=0)
            B1 = np.concatenate((P.B, np.zeros((1,1))), axis=0)
            # is the system controllable?
            if np.linalg.matrix_rank(cnt.ctrb(A1, B1)) != 3: 
                print("System Not Controllable") 
            else:
                des_char_poly = np.convolve(des_ctrl_charpoly, np.poly([integrator_pole]))
                K1 = cnt.place(A1, B1, np.roots(des_char_poly))
                self.K  = K1[0][0:2]
                self.ki = K1[0][2]
                # is the system observable?
            if np.linalg.matrix_rank(cnt.obsv(P.A, P.C)) != 2: 
                print("System Not Observable") 
            else: 
                self.L = cnt.place(P.A.T, P.C.T, des_obsv_poles).T
            print('K: ', self.K)
            print('ki: ', self.ki)
            print('L^T: ', self.L.T)
            self.observer_state = np.array([[0.0], [0.0]])
            self.A = P.A
            self.B = P.B
            self.C = P.C
        elif self.control_mode==3: # no integrator, disturbance estimate
            # is the system controllable?
            if np.linalg.matrix_rank(cnt.ctrb(P.A, P.B)) != 2: 
                print("System Not Controllable") 
            else:
                self.K = cnt.place(P.A, P.B, des_ctrl_poles)
                tmp = P.C @ np.linalg.inv(P.A - P.B @ self.K) @ P.B 
                self.kr = -1.0 / tmp[0][0]
            #augment system to disturbance observer
            A2 = np.concatenate((np.concatenate((P.A, P.B), axis=1), 
                                np.zeros((1, 3))), axis=0)
            B1 = np.concatenate((P.B, np.zeros((1,1))), axis=0)
            C2 = np.concatenate((P.C, np.zeros((1,1))), axis=1)
            # is the system observable?
            if np.linalg.matrix_rank(cnt.obsv(A2, C2)) != 3: 
                print("System Not Observable")
            else: 
                des_char_poly = np.convolve(des_obsv_charpoly, np.poly([dis_obsv_pole]))
                L2  = cnt.place(A2.T, C2.T, np.roots(des_char_poly)).T 
                L = L2[0:2]
                Ld = L2[2][0]
            print('K: ', self.K)
            print('kr: ', self.kr)
            print('L^T: ', L.T)
            print('Ld: ', Ld)
            self.observer_state = np.array([[0.0], [0.0], [0.0]])
            self.A = A2
            self.B = B1
            self.C = C2
            self.L = L2
        elif self.control_mode==4: # integrator, disturbance estimate
            # augment system to add integrator
            A1 = np.concatenate((np.concatenate((P.A, np.zeros((2,1))), axis=1),
                                np.concatenate((-P.C, np.zeros((1,1))), axis=1)), axis=0)
            B1 = np.concatenate((P.B, np.zeros((1,1))), axis=0)
            # is the system controllable?
            if np.linalg.matrix_rank(cnt.ctrb(A1, B1)) != 3: 
                print("System Not Controllable")
            else:
                des_char_poly = np.convolve(des_ctrl_charpoly, np.poly([integrator_pole]))
                K1 = cnt.place(A1, B1, np.roots(des_char_poly))
                self.K  = K1[0][0:2]
                self.ki = K1[0][2]
            # augment system to disturbance observer
            A2 = np.concatenate((np.concatenate((P.A, P.B), axis=1),
                                np.zeros((1, 3))), axis=0)
            C2 = np.concatenate((P.C, np.zeros((1,1))), axis=1)
            # is the system observable?
            if np.linalg.matrix_rank(cnt.obsv(A2, C2)) !=3: 
                print("System Not Observable")
            else:
                des_char_poly = np.convolve(des_obsv_charpoly, np.poly([dis_obsv_pole]))
                L2  = cnt.place(A2.T, C2.T, np.roots(des_char_poly)).T 
                L = L2[0:2]
                Ld = L2[2][0]
            print('K: ', self.K)
            print('ki: ', self.ki)
            print('L^T: ', L.T)
            print('Ld: ', Ld)
            self.observer_state = np.array([[0.0], [0.0], [0.0]])
            self.A = A2
            self.B = B1
            self.C = C2
            self.L = L2
        self.Ts = P.Ts               
        self.u_d1 = 0.0              # input, delayed by one sample
        self.integrator = 0.0        # integrator
        self.error_d1 = 0.0          # error signal delayed by 1 sample

    def update(self, y_ref, y_m):
        # update the observer and extract theta_hat
        x_hat, d_hat = self.update_observer(y_m)
        y_hat = P.C @ x_hat

        # integrate error
        error = y_ref - y_hat
        self.integrator = self.integrator + (self.Ts/2.0)*(error + self.error_d1)
        self.error_d1 = error

        if self.control_mode==1: # no integrator, no disturbance estimate
            u = -self.K @ x_hat + self.kr * y_ref
        elif self.control_mode==2: # integrator, no disturbance estimate
            u = -self.K @ x_hat - self.ki * self.integrator
        elif self.control_mode==3: # no integrator, disturbance estimate
            u = -self.K @ x_hat + self.kr * y_ref - d_hat
        elif self.control_mode==4: # integrator, disturbance estimate
            u = -self.K @ x_hat - self.ki * self.integrator - d_hat
        # compute total torque
        self.u_d1 = u.item(0)
        return u.item(0), x_hat, d_hat

    def update_observer(self, y_m):
        # update the observer using RK4 integration
        F1 = self.observer_f(self.observer_state, y_m)
        F2 = self.observer_f(self.observer_state + self.Ts / 2 * F1, y_m)
        F3 = self.observer_f(self.observer_state + self.Ts / 2 * F2, y_m)
        F4 = self.observer_f(self.observer_state + self.Ts * F3, y_m)
        self.observer_state += self.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)
        x_hat = self.observer_state[0:2]
        if ((self.control_mode==3) or  (self.control_mode==4)):
            d_hat = self.observer_state[2][0]
        else:
            d_hat = 0.0
        return x_hat, d_hat
        
    def observer_f(self, x_hat, y_m):
        xhat_dot = self.A @ x_hat + self.B * self.u_d1 + self.L * (y_m - self.C @ x_hat)
        return xhat_dot


