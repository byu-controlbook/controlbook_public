import pendulumParam as P
import matplotlib.pyplot as plt
from control import tf, step_response, bode, tf2ss, margin, mag2db
import numpy as np
import hw16 as P16
import loopshape_tools as ls

# flag to define if using dB or absolute scale for M(omega)
dB_flag = P16.dB_flag

# assigning plant and controller from past HW
# (to make sure we don't introduce additional errors)
P_in = P16.P_in
Plant = P_in

#########################################
#   Control Design
#########################################

C = tf([1], [1]) * ls.proportional(kp=-1.) # correct for negative sign in plant
C = C * ls.proportional(kp=800.0) # set cross-over to 40
C = C * ls.lead(w=40., M=15.) # increase phase margin

###########################################################
# Extracting coefficients for controller
###########################################################
C_num = np.asarray(C.num[0])
C_den = np.asarray(C.den[0])

if __name__=="__main__":
    # calculate bode plot and gain and phase margin
    # for original PID * plant dynamics
    mag, phase, omega = bode(Plant, dB=dB_flag,
                                omega=np.logspace(-3, 5),
                                plot=True, label="$P(s)$")

    gm, pm, Wcg, Wcp = margin(Plant)
    print("for original system:")
    if dB_flag == True:
        print(" pm: ", pm, " Wcp: ", Wcp,
                "gm: ", mag2db(gm), " Wcg: ", Wcg)
    elif dB_flag == False:
        print(" pm: ", pm, " Wcp: ", Wcp,
                "gm: ", gm, " Wcg: ", Wcg)


    #########################################
    #   Define Design Specifications
    #########################################
    # ----------- noise specification --------
    ls.spec_noise(gamma_n=0.1, omega_n=200., dB_flag=dB_flag)


    #########################################
    #  Create the plots
    #########################################

    ## plot the effect of adding the new compensator terms
    mag, phase, omega = bode(Plant * C, dB=dB_flag,
                                omega=np.logspace(-3, 5),
                                plot=True, label="$C_{final}(s)P(s)$",
                                color='orange')

    gm, pm, Wcg, Wcp = margin(Plant * C)
    print("for final C*P:")
    if dB_flag == True:
        print(" pm: ", pm, " Wcp: ", Wcp,
                "gm: ", mag2db(gm), " Wcg: ", Wcg)
    elif dB_flag == False:
        print(" pm: ", pm, " Wcp: ", Wcp,
                "gm: ", gm, " Wcg: ", Wcg)

    fig = plt.gcf()
    fig.axes[0].legend()
    fig.axes[0].set_title('Inner Open-loop Bode Plot')
    plt.show()

    ############################################
    # now check the closed-loop response
    ############################################
    # Open-loop transfer function
    OPEN = Plant*C
    # Closed loop transfer function from R to Y
    CLOSED_R_to_Y = (Plant*C/(1.0+Plant*C))
    # Closed loop transfer function from R to U
    CLOSED_R_to_U = (C/(1.0+Plant*C))

    plt.figure()
    plt.subplot(311)
    mag, phase, omega = bode(CLOSED_R_to_Y,
                                dB=dB_flag, plot=False)
    if dB_flag:
        plt.semilogx(omega, mag2db(mag), color=[0,0,1])
    else:
        plt.loglog(omega, mag, color=[0,0,1])
    plt.grid(True)
    plt.ylabel('Magnitude (dB)')
    plt.title('closed-loop magnitude ratio $\\frac{Y}{R}$')

    plt.subplot(312), plt.grid(True)
    T = np.linspace(0, 2, 100)
    _, yout = step_response(CLOSED_R_to_Y, T)
    plt.plot(T, yout, color=[0,0,1])
    plt.ylabel('Amplitude')
    plt.title('Step Response')

    plt.subplot(313), plt.grid(True)
    _, Uout = step_response(CLOSED_R_to_U, T)
    plt.plot(T, Uout, color=[0,0,1])
    plt.ylabel('Amplitude')
    plt.title('Control Effort')

    plt.tight_layout()
    plt.show()



