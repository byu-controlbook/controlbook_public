import matplotlib.pyplot as plt
from control import tf, margin, bode, tf2ss, step_response, mag2db
import numpy as np
import satelliteParam as P
from ctrlPID import ctrlPID
import hw16 as P16
import loopShapingInner as L_in
import loopshape_tools as lt

P10 = ctrlPID()

# flag to define if using dB or absolute scale for M(omega)
dB_flag = P16.dB_flag

# Compute outer loop transfer function as described in Chapter 18
num_out = [P10.sigma, 1.0]
den_out = [P10.sigma*P.Jp, P10.sigma*P.b+P.Jp,
           P10.sigma*P.k+P.b+P10.kd_phi, P.k]
P_out = tf(num_out, den_out)

# construct plant as cascade of P_out and closed inner loop
Plant = P_out * (L_in.Plant * L_in.C /
                 (1 + L_in.Plant * L_in.C))

#########################################
#   Control Design
#########################################
C = tf([1], [1])

#  -----integral control: change low freq. slope----
ki = 0.1
C_int = lt.get_control_integral(ki)


# -----lead control: change the phase margin ----
omega_max = 0.15
M = 120.0
C_lead = lt.get_control_lead(omega_max, M)

# -----lag control: increase magnitude near required freq. ----
z = 2.0
M = 60.0
C_lag = lt.get_control_lag(0.5, 60)

#  -----proportional control: change cross over frequency----
mag, phase, omega = bode(Plant*C_int*C_lead*C_lag,
                         omega=[omega_max],
                         Plot=False)
C_k = lt.get_control_proportional(1.0/mag[0])

#  -----low pass filter: decrease gain at high frequency (noise)----
# we will have to use two to get the desired noise attenuation
p = 1.5
C_lpf1 = lt.get_control_lpf(p)
p = 1.8
C_lpf2 = lt.get_control_lpf(p)

# calculating final controller
C = C*C_int*C_lead*C_lag*C_k*C_lpf1*C_lpf2


############################################
#  Prefilter Design
############################################
# low pass filter
p = 0.1
F = lt.get_control_lpf(p)


##############################################
#  Convert Controller to State Space Equations
##############################################
C_ss = tf2ss(C)
F_ss = tf2ss(F)


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
    #----------- input disturbance spec ---------
    omega_din = 10 ** (-2.0)  # reject input disturbances below freq.
    gamma_din = 0.1  # amount of input disturbance in output
    lt.add_spec_input_disturbance(gamma_din, omega_din,
                                  Plant, dB_flag)

    # ----------- noise specification --------
    omega_n = 10  # attenuate noise above this frequency
    gamma_n = 10.0 ** (-4.0)  # attenuate noise by this amount
    lt.add_spec_noise(gamma_n, omega_n, dB_flag)

    ## plot the effect of adding the new compensator terms
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
