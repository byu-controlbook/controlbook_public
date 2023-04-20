import matplotlib.pyplot as plt
from control import tf, bode, margin, step_response, mag2db, tf2ss, c2d
import numpy as np
import armParam as P
import hw16 as P16
import loopshape_tools as ls

# flag to define if using dB or absolute scale for M(omega)
dB_flag = P16.dB_flag

# assigning plant and controller from past HW
# (to make sure we don't introduce additional errors)
Plant = P16.Plant
C_pid = P16.C_pid

###################################################################
#   Control Design
###################################################################
C = C_pid \
    * ls.lag(z=0.9, M=20.0)\
    * ls.lpf(p=90.0) \
    * ls.lead(w=6.1, M=1.5)


###########################################################
# add a prefilter to eliminate the overshoot
###########################################################
#F = tf(1, 1) * ls.notch(p1=5.0, p2=1.0, M=10.0)
#F = tf(1, 1) * ls.notch2(ws=.0, M=10.0)
F = tf(1, 1) * ls.lpf(p=1.0)


###########################################################
# Extracting coefficients for controller and prefilter
###########################################################
C_num = np.asarray(C.num[0])
C_den = np.asarray(C.den[0])
F_num = np.asarray(F.num[0])
F_den = np.asarray(F.den[0])


if __name__=="__main__":
    # calculate bode plot and gain and phase margin
    # for original PID * plant dynamics
    mag, phase, omega = bode(Plant * C_pid, dB=dB_flag,
                             omega=np.logspace(-3, 5),
                             plot=True, label="$C_{pid}P$")


    gm, pm, Wcg, Wcp = margin(Plant * C_pid)
    print("for original C_pid system:")
    if dB_flag is True:
        print(" pm: ", pm, " Wcp: ", Wcp,
              "gm: ", mag2db(gm), " Wcg: ", Wcg)
    elif dB_flag is False:
        print(" pm: ", pm, " Wcp: ", Wcp,
              "gm: ", gm, " Wcg: ", Wcg)


    #########################################
    #   Define Design Specifications
    #########################################
    #----------- noise specification --------
    omega_n=1000
    mag, phase, omega = bode(Plant * C_pid, dB=dB_flag, omega=[omega_n])
    ls.spec_noise(gamma_n=0.1*mag[0], omega_n=omega_n, dB_flag=dB_flag)
    #----------- general tracking specification --------
    ls.spec_disturbance(gamma_d=0.1, omega_d=0.07, plant=Plant*C_pid, dB_flag=dB_flag)

    #########################################
    #   Plotting routines
    #########################################

    ## plot the effect of adding the new compensator terms
    mag, phase, omega = bode(Plant * C, dB=dB_flag,
                             omega=np.logspace(-3, 5),
                             plot=True, label="$C_{final}(s)P(s)$",
                             color='orange')

    gm, pm, Wcg, Wcp = margin(Plant * C)
    print("for final C*P:")
    if dB_flag is True:
        print(" pm: ", pm, " Wcp: ", Wcp,
              "gm: ", mag2db(gm), " Wcg: ", Wcg)
    elif dB_flag is False:
        print(" pm: ", pm, " Wcp: ", Wcp,
              "gm: ", gm, " Wcg: ", Wcg)

    fig = plt.gcf()
    fig.axes[0].legend()
    #plt.show()


    ############################################
    # now check the closed-loop response with prefilter
    ############################################
    # Closed loop transfer function from R to Y - no prefilter
    CLOSED_R_to_Y = (Plant * C / (1.0 + Plant * C))
    # Closed loop transfer function from R to Y - with prefilter
    CLOSED_R_to_Y_with_F = (F * Plant * C / (1.0 + Plant * C))
    # Closed loop transfer function from R to U - no prefilter
    CLOSED_R_to_U = (C / (1.0 + Plant * C))
    # Closed loop transfer function from R to U - with prefilter
    CLOSED_R_to_U_with_F = (F*C / (1.0 + Plant * C))

    plt.figure(4)
    plt.clf()
    plt.grid(True)
    plt.subplot(311)
    mag, phase, omega = bode(CLOSED_R_to_Y, dB=dB_flag, plot=False)
    if dB_flag:
        plt.semilogx(omega, mag2db(mag), color=[0, 0, 1],
            label='closed-loop $\\frac{Y}{R}$ - no pre-filter')
    else:
        plt.loglog(omega, mag, color=[0, 0, 1],
            label='closed-loop $\\frac{Y}{R}$ - no pre-filter')
    mag, phase, omega = bode(CLOSED_R_to_Y_with_F,
                             dB=dB_flag, plot=False)
    if dB_flag:
        plt.semilogx(omega, mag2db(mag), color=[0, 1, 0],
            label='closed-loop $\\frac{Y}{R}$ - with pre-filter')
    else:
        plt.loglog(omega, mag, color=[0, 1, 0],
            label='closed-loop $\\frac{Y}{R}$ - with pre-filter')
    plt.ylabel('Closed-Loop Bode Plot')
    plt.grid(True)
    plt.legend()

    plt.subplot(312), plt.grid(True)
    T = np.linspace(0, 2, 100)
    _, yout_no_F = step_response(CLOSED_R_to_Y, T)
    _, yout_F = step_response(CLOSED_R_to_Y_with_F, T)
    plt.plot(T, yout_no_F, color=[0,0,1],
             label='response without prefilter')
    plt.plot(T, yout_F, color=[0,1,0],
             label='response with prefilter')
    plt.legend()
    plt.ylabel('Step Response')


    plt.subplot(313)
    plt.grid(True)
    _, Uout = step_response(CLOSED_R_to_U, T)
    _, Uout_F = step_response(CLOSED_R_to_U_with_F, T)
    plt.plot(T, Uout, color=[0, 0, 1],
             label='control effort without prefilter')
    plt.plot(T, Uout_F, color=[0, 1, 0],
             label='control effort with prefilter')
    plt.ylabel('Control Effort')
    plt.legend()

    plt.show()
