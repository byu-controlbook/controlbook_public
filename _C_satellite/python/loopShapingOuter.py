import matplotlib.pyplot as plt
from control import tf, margin, bode, tf2ss, step_response, mag2db
import numpy as np
import satelliteParam as P
import hw16 as P16
import loopShapingInner as L_in
import loopshape_tools as ls
from ctrlPID import ctrlPID
P10 = ctrlPID()

# flag to define if using dB or absolute scale for M(omega)
dB_flag = P16.dB_flag

# Compute outer loop transfer function as described in Chapter 18
# num_out = [P10.sigma, 1.0]
# den_out = [P10.sigma*P.Jp, P10.sigma*P.b+P.Jp,
#            P10.sigma*P.k+P.b+P10.kd_phi, P.k]
# P_out = tf(num_out, den_out)
P_out = tf([P.b, P.k], [P.Jp+P.b*P10.kd_phi, P.b+P.k*P10.kd_phi, P.k])

# construct plant as cascade of P_out and closed inner loop
Plant = P_out * (L_in.Plant * L_in.C /
                 (1 + L_in.Plant * L_in.C))

#########################################
#   Control Design
#########################################
C = tf([1], [1])\
    * ls.integral(ki=0.2)\
    * ls.proportional(kp=1.0)\
    * ls.lead(w=0.2, M=5.0)\
    * ls.lpf(p=6.0)
    

############################################
#  Prefilter Design
############################################
# low pass filter
<<<<<<< HEAD
F = tf([1],[1])
# p = 0.1
# F = lt.get_control_lpf(p)
=======
F = tf([1],[1]) \
    * ls.lpf(p=0.4)
    #* ls.notch(p1=0.1, p2=0.3, M=1.5)
>>>>>>> aed2968cd37c9808e5d33ed9c6b5ad771c62cf73

###########################################################
# Extracting coefficients for controller
###########################################################
C_num = np.asarray(C.num[0])
C_den = np.asarray(C.den[0])
F_num = np.asarray(F.num[0])
F_den = np.asarray(F.den[0])


if __name__ == '__main__':

    # calculate bode plot and gain and phase margin
    # for original PID * plant dynamics
    mag, phase, omega = bode(Plant, dB=dB_flag,
                             omega=np.logspace(-4, 4),
                             plot=True, label=r'$P_{\theta,in}(s)$')

    gm, pm, Wcg, Wcp = margin(Plant)
    print("for original system:")
    print(" pm: ", pm, " Wcp: ", Wcp, "gm: ", gm, " Wcg: ", Wcg)

     #########################################
    #   Define Design Specifications
    #########################################
    ls.spec_disturbance(gamma_d=0.1, omega_d=10**(-2.0), plant=Plant, dB_flag=dB_flag)
    ls.spec_noise(gamma_n=10.0**(-4.0), omega_n=10.0, dB_flag=dB_flag)

    #########################################
    #  Create the plots
    #########################################
    mag, phase, omega = bode(Plant * C, dB=dB_flag,
                             omega=np.logspace(-4, 4),
                             plot=True, margins=True,
                             label=r"$C_{\phi,out}(s)"+
                                   "P_{\phi,out}(s)$")
    gm, pm, Wcg, Wcp = margin(Plant * C)
    print("for final C*P:")
    print(" pm: ", pm, " Wcp: ", Wcp, "gm: ", gm, " Wcg: ", Wcg)
    fig = plt.gcf()
    fig.axes[0].legend()
    fig.axes[0].set_title('Bode Diagram')
    fig.axes[0].grid(True)
    fig.axes[1].grid(True)

    ############################################
    # closed-loop response with prefilter
    ############################################
    # Closed loop transfer function from R to Y - no prefilter
    CLOSED_R_to_Y = (Plant * C / (1.0 + Plant * C))
    # Closed loop transfer function from R to Y - with prefilter
    CLOSED_R_to_Y_with_F = (F * Plant * C / (1.0 + Plant * C))
    # Closed loop transfer function from R to U - no prefilter
    CLOSED_R_to_U = (C / (1.0 + Plant * C))
    # Closed loop transfer function from R to U - with prefilter
    CLOSED_R_to_U_with_F = (F*C / (1.0 + Plant * C))

    plt.figure(2)
    plt.grid(True)
    plt.subplot(311)
    mag, phase, omega = bode(CLOSED_R_to_Y, dB=dB_flag, plot=False)
    if dB_flag:
        plt.semilogx(omega, mag2db(mag), color=[0,0,1],
            label='closed-loop $\\frac{Y}{R}$ - no pre-filter')
    else:
        plt.loglog(omega, mag, color=[0,0,1],
            label='closed-loop $\\frac{Y}{R}$ - no pre-filter')
    mag, phase, omega = bode(CLOSED_R_to_Y_with_F, dB=dB_flag, plot=False)
    if dB_flag:
        plt.semilogx(omega, mag2db(mag), color=[0,1,0],
            label='closed-loop $\\frac{Y}{R}$ - with pre-filter')
    else:
        plt.loglog(omega, mag, color=[0,1,0],
            label='closed-loop $\\frac{Y}{R}$ - with pre-filter')
    plt.ylabel('Closed-Loop Bode Plot')
    plt.grid(True)
    plt.legend()

    plt.subplot(312), plt.grid(True)
    T = np.linspace(0, 50, 50*100)
    _, yout_no_F = step_response(CLOSED_R_to_Y, T)
    _, yout_F = step_response(CLOSED_R_to_Y_with_F, T)
    plt.plot(T, yout_no_F, color=[0,0,1],
            label='response without prefilter')
    plt.plot(T, yout_F, color=[0,1,0],
            label='response with prefilter')
    plt.legend()
    plt.ylabel('Step Response')

    plt.subplot(313), plt.grid(True)
    _, Uout = step_response(CLOSED_R_to_U, T)
    _, Uout_F = step_response(CLOSED_R_to_U_with_F, T)
    plt.plot(T, Uout, color=[0,0,1],
            label='control effort without prefilter')
    plt.plot(T, Uout_F, color=[0,1,0],
            label='control effort with prefilter')
    plt.ylabel('Control Effort')
    plt.legend()
    plt.show()
