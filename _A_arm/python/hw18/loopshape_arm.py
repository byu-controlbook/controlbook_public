import sys
sys.path.append('..')  # add parent directory
sys.path.append('../hw16')  # add parent directory
import armParamHW16 as P16
import matplotlib.pyplot as plt
from control import tf, bode, margin, step_response, tf2ss, mag2db
import numpy as np
import helper_functions as hf


# flag to define if using dB or absolute scale for M(omega)
dB_flag = P16.dB_flag

# assigning plant and controller from past HW
# (to make sure we don't introduce additional errors)
Plant = P16.Plant
C_pid = P16.C_pid

#######################################################################
#   Control Design
#######################################################################
C = C_pid

#########################################
#  Add a lead compensator to improve phase margin
w_lead = 10.8  # location of maximum frequency bump
M = 10.0  # separation between zero and pole
CLead = hf.get_control_lead(w_lead, M)

#########################################
#  Add a lag compensator for low frequency performance
z = 0.7
M = 20.0
CLag = hf.get_control_lag(z, M)

#########################################
#  Add a low pass filter: decrease gain at high frequency (noise)
p = 50.0
Clpf1 = hf.get_control_lpf(p)

# could add a second low-pass filter, but it's not needed
# # low pass filter: decrease gain at high frequency (noise)
# p = 150.0
# Clpf2 = hf.get_control_lpf(p)

#########################################
# combine all terms for C
C = C*CLead*CLag #*Clpf1

###########################################################
# add a prefilter to eliminate the overshoot
###########################################################
F = 1.0
# low pass filter
p = 3.0
F = F * hf.get_control_lpf(p)


##############################################
#  Convert Controller to State Space Equations if following method in 18.1.7
##############################################
C_ss = tf2ss(C)  # convert to state space
F_ss = tf2ss(F)  # convert to state space


if __name__=="__main__":
    # calculate bode plot and gain and phase margin for original PID * plant dynamics
    mag, phase, omega = bode(Plant * C_pid, dB=dB_flag,
                             omega=np.logspace(-3, 5),
                             plot=True, label="$C_{pid}(s)P(s)$")


    gm, pm, Wcg, Wcp = margin(Plant * C_pid)
    print("for original C_pid system:")
    if dB_flag == True:
        print(" pm: ", pm, " Wcp: ", Wcp, "gm: ", mag2db(gm), " Wcg: ", Wcg)
    elif dB_flag == False:
        print(" pm: ", pm, " Wcp: ", Wcp, "gm: ", gm, " Wcg: ", Wcg)

    #----------- noise specification --------
    omega_n = 1000    # attenuate noise above this frequency
    gamma_n = 0.1    # attenuate noise by this amount
    hf.add_spec_noise(gamma_n, omega_n, dB_flag)

    #----------- general tracking specification --------
    omega_d = 0.07    # track signals below this frequency
    gamma_d = 0.1    # tracking error improvement over the original system
    hf.add_spec_input_disturbance(gamma_d, omega_d, Plant*C_pid, dB_flag)

    ## plot the effect of adding the new compensator terms
    mag, phase, omega = bode(Plant * C, dB=dB_flag,
                             omega=np.logspace(-3, 5),
                             plot=True, label="$C_{final}(s)P(s)$")

    gm, pm, Wcg, Wcp = margin(Plant * C)
    print("for final C*P:")
    if dB_flag == True:
        print(" pm: ", pm, " Wcp: ", Wcp, "gm: ", mag2db(gm), " Wcg: ", Wcg)
    elif dB_flag == False:
        print(" pm: ", pm, " Wcp: ", Wcp, "gm: ", gm, " Wcg: ", Wcg)

    fig = plt.gcf()
    fig.axes[0].legend()
    plt.show()


    ############################################
    # now check the closed-loop response with prefilter
    ############################################
    # Closed loop transfer function from R to Y - no prefilter
    CLOSED_R_to_Y = (Plant * C / (1.0 + Plant * C))
    # Closed loop transfer function from R to Y - with prefilter
    CLOSED_R_to_Y_with_F = (F * Plant * C / (1.0 + Plant * C))
    # Closed loop transfer function from R to U
    CLOSED_R_to_U = (C / (1.0 + Plant * C))

    fig = plt.figure()
    plt.grid(True)
    mag, phase, omega = bode(CLOSED_R_to_Y, dB=dB_flag, plot=True,
                             color=[0, 0, 1], label='closed-loop $\\frac{Y}{R}$ - no pre-filter')
    mag, phase, omega = bode(CLOSED_R_to_Y_with_F, dB=dB_flag, plot=True,
                             color=[0, 1, 0], label='closed-loop $\\frac{Y}{R}$ - with pre-filter')
    fig.axes[0].set_title('Closed-Loop Bode Plot')
    fig.axes[0].legend()


    plt.figure()
    plt.subplot(211), plt.grid(True)
    T = np.linspace(0, 2, 100)
    _, yout_no_F = step_response(CLOSED_R_to_Y, T)
    _, yout_F = step_response(CLOSED_R_to_Y_with_F, T)
    plt.plot(T, yout_no_F, 'b', label='response without prefilter')
    plt.plot(T, yout_F, 'g', label='response with prefilter')
    plt.legend()
    plt.ylabel('Step Response')


    plt.subplot(212), plt.grid(True)
    T, Uout = step_response(CLOSED_R_to_U, T)
    plt.plot(T, Uout, color='b', label='control effort without prefilter')
    plt.ylabel('Control Effort')
    plt.legend()

    plt.show()
