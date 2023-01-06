
import matplotlib.pyplot as plt
from control import tf, step_response, bode, tf2ss, \
    margin, mag2db, minreal
import numpy as np
import hw16 as P16
import loopshape_tools as lt
import loopShapingInner as L_in

# flag to define if using dB or absolute scale for M(omega)
dB_flag = P16.dB_flag

P_out = P16.P_out

# construct plant as cascade of P_out and closed inner loop
Plant = minreal(P_out* \
            (L_in.P_in*L_in.C/(1+L_in.P_in*L_in.C)))

#########################################
#   Control Design
#########################################
C = tf([1], [1])

# define specifications:
omega_r = 0.0032  # track signals below this frequency
gamma_r = 0.00001  # tracking error below this value

omega_n = 1000  # attenuate noise above this frequency
gamma_n = 0.0001  # attenuate noise by this amount


#  phase lead: increase PM (stability)
# At desired crossover frequency, PM = -3
# Add 70 deg of PM with lead

#location of maximum frequency bump (desired crossover)
w_max = 1.1
M = 32   # lead ratio
Lead = lt.get_control_lead(w_max, M)
C = C*Lead

# find gain to set crossover at w_max = 1.1 rad/s
mag, phase, omega = bode(Plant*C, dB=False,
                         omega=[w_max], Plot=False)
K = lt.get_control_proportional(1.0/mag[0])
C = K*C

## boost low-frequency gain
# Find gain increase needed at omega_r
mag, phase, omega = bode(Plant*C, omega=[omega_r],
                         Plot=False)
gain_increase_needed = 1/gamma_r/mag[0]

# minimum gain increase at low frequencies is 4.8 let lag ratio
# be 8.
M = 8
p = omega_r # set pole at omega_r
z = M*p # set zero at M*omega_r
Lag = lt.get_control_lag(z, M)
C = C*Lag


# Noise attenuation constraint not quite satisfied can be
# satisfied by reducing gain at 400 rad/s by a factor of 2 Use
# a low-pass filter
lpf = lt.get_control_lpf(100.0)
C = lpf*C

############################################
#  Prefilter Design
############################################
F = tf([1], [1])

# low pass filter
p = 2
LPF = tf([p], [1, p])
F = F*LPF

##############################################
#  Convert Controller to State Space Equations
##############################################
C_ss = tf2ss(C)
F_ss = tf2ss(F)

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
    # ----------- general tracking specification --------
    lt.add_spec_ref_tracking(gamma_r, omega_r, dB_flag)

    # ----------- noise specification --------
    lt.add_spec_noise(gamma_n, omega_n, dB_flag)

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
    plt.show()


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

    plt.figure()
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
