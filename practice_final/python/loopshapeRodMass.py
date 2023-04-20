import rodMassParam as P
import matplotlib.pyplot as plt
from control import TransferFunction as tf
from control import tf, bode, margin, step_response, mag2db
import numpy as np
import loopshape_tools as ls
from ctrlPID import ctrlPID
PID = ctrlPID()

# Compute plant transfer functions
Plant = 
C_pid = 
PLOT = True
#PLOT = False
dB_flag = True

#######################################################################
#   Control Design
#######################################################################
C = 

###########################################################
# add a prefilter to eliminate the overshoot
###########################################################
F = 
##############################################
#  Convert Controller to State Space Equations if following method in 18.1.7
##############################################
C_num = np.asarray(C.num[0])
C_den = np.asarray(C.den[0])
F_num = np.asarray(F.num[0])
F_den = np.asarray(F.den[0])

if __name__ == "__main__":
    # calculate bode plot and gain and phase margin for original PID * plant dynamics
    mag, phase, omega = bode(Plant * C_pid, dB=True,
                             omega=np.logspace(-3, 5),
                             Plot=True, label="$P(s)C_{pid}(s)$")

    gm, pm, Wcg, Wcp = margin(Plant * C_pid)
    print("for original C_pid system:")
    if dB_flag is True:
        print(" pm: ", pm, " Wcp: ", Wcp, "gm: ", mag2db(gm), " Wcg: ", Wcg)
    elif dB_flag is False:
        print(" pm: ", pm, " Wcp: ", Wcp, "gm: ", gm, " Wcg: ", Wcg)

    #########################################
    #   Define Design Specifications
    #########################################

    # plot the effect of adding the new compensator terms
    mag, phase, omega = bode(Plant * C, dB=dB_flag,
                             omega=np.logspace(-4, 5),
                             plot=True, label="$P(s)C_{final}(s)$")

    gm, pm, Wcg, Wcp = margin(Plant * C)
    print("for final P*C:")
    if dB_flag is True:
        print(" pm: ", pm, " Wcp: ", Wcp, "gm: ", mag2db(gm), " Wcg: ", Wcg)
    elif dB_flag is False:
        print(" pm: ", pm, " Wcp: ", Wcp, "gm: ", gm, " Wcg: ", Wcg)

    plt.figure(1)
    fig = plt.gcf()
    fig.axes[0].legend()

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
    CLOSED_R_to_U_with_F = (F * C / (1.0 + Plant * C))

    fig = plt.figure(2)
    plt.clf()
    plt.grid(True)
    mag, phase, omega = bode(CLOSED_R_to_Y, dB=dB_flag, plot=True,
                             color=[0, 0, 1], label='closed-loop $\\frac{Y}{R}$ - no pre-filter')
    mag, phase, omega = bode(CLOSED_R_to_Y_with_F, dB=dB_flag, plot=True,
                             color=[0, 1, 0], label='closed-loop $\\frac{Y}{R}$ - with pre-filter')
    fig.axes[0].set_title('Closed-Loop Bode Plot')
    fig.axes[0].legend()

    plt.figure(4)
    plt.clf()
    plt.subplot(211), plt.grid(True)
    T = np.linspace(0, 2, 100)
    _, yout_no_F = step_response(CLOSED_R_to_Y, T)
    _, yout_F = step_response(CLOSED_R_to_Y_with_F, T)
    plt.plot(T, yout_no_F, 'b', label='response without prefilter')
    plt.plot(T, yout_F, 'g', label='response with prefilter')
    plt.legend()
    plt.ylabel('Step Response')

    plt.subplot(212), plt.grid(True)
    _, Uout_no_F = step_response(CLOSED_R_to_U, T)
    _, Uout_F = step_response(CLOSED_R_to_U_with_F, T)
    plt.plot(T, Uout_no_F, color='b', label='control effort without prefilter')
    plt.plot(T, Uout_F, color='g', label='control effort with prefilter')
    plt.ylabel('Control Effort')
    plt.legend()

    plt.show()

