
import matplotlib.pyplot as plt
from control import tf, step_response, bode, tf2ss, \
    margin, mag2db, minreal
import numpy as np
import hw16 as P16
import loopshape_tools as ls
import loopShapingInner as L_in

# flag to define if using dB or absolute scale for M(omega)
dB_flag = P16.dB_flag
P_out = P16.P_out

# construct plant as cascade of P_out and closed inner loop
Plant = minreal(P_out*(L_in.P_in*L_in.C/(1+L_in.P_in*L_in.C)))

#########################################
#   Control Design
#########################################
C = tf([1], [1])
C = C * ls.proportional(0.1) \
      * ls.lead(w=1.0,M=20.0) \
      * ls.lag(z=0.04, M=10) \
      * ls.lpf(p=50)

############################################
#  Prefilter Design
############################################
F = tf([1], [1])\
    * ls.lpf(p=2.0)

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
    ls.spec_track_ref(gamma_r=0.00001, omega_r=0.0032, dB_flag=dB_flag)
    ls.spec_noise(gamma_n=0.0001, omega_n=1000, dB_flag=dB_flag)

   #########################################
    #  Create the plots
    #########################################
    mag, phase, omega = bode(Plant * C, dB=dB_flag,
                             omega=np.logspace(-3, 5),
                             plot=True, label="$C_{final}(s)P(s)$",
                             margins=True, color='orange')

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
    fig.axes[0].set_title('Outer Open-loop Bode Plot')
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
    mag, phase, omega = bode(CLOSED_R_to_Y,
                             dB=dB_flag, plot=False)
    if dB_flag:
        plt.semilogx(omega, mag2db(mag), color=[0,0,1],
            label='closed-loop $\\frac{Y}{R}$ - no pre-filter')
    else:
        plt.loglog(omega, mag, color=[0,0,1],
            label='closed-loop $\\frac{Y}{R}$ - no pre-filter')
    mag, phase, omega = bode(CLOSED_R_to_Y_with_F,
                             dB=dB_flag, plot=False)
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
    T = np.linspace(0, 5, 5*200)
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
