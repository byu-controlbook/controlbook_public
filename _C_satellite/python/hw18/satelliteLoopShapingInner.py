from control import tf, margin, bode, tf2ss, step_response, mag2db
import numpy as np
import matplotlib.pyplot as plt

import satelliteParam as P
import hw10.satelliteParamHW10 as P10
import hw16.satelliteParamHW16 as P16
import hw18.loopshape_tools as lt

# flag to define if using dB or absolute scale for M(omega)
dB_flag = P16.dB_flag

# Compute open-loop transfer functions as described in Chapter 18
Plant = tf([P10.sigma, 1],
           [P10.sigma*P.Js, P10.sigma*P.b+P.Js,
            P10.sigma*P.k+P.b+P10.kd_th, P.k])

#########################################
#   Control Design
#########################################
C = tf([1], [1])

#  -----proportional control: change cross over frequency----
kp = 45.0
C_k = lt.get_control_proportional(kp)

#  -----low pass filter: decrease gain at high frequency (noise)----
p = 8.0
C_lpf = lt.get_control_lpf(p)

# calculating final controller
C = C * C_lpf * C_k

##############################################
#  Convert Controller to State Space Equations
##############################################
C_ss = tf2ss(C)  # convert to state space


def main():
    
    # calculate bode plot and gain and phase margin
    # for original PID * plant dynamics
    mag, phase, omega = bode(Plant, dB=dB_flag,
                             omega=np.logspace(-4, 5),
                             plot=True, label=r'$P_{\theta,in}(s)$')

    gm, pm, Wcg, Wcp = margin(Plant)
    print("for original system:")
    print(" pm: ", pm, " Wcp: ", Wcp, "gm: ", gm, " Wcg: ", Wcg)

    #########################################
    #   Define Design Specifications
    #########################################
    #----------- general tracking specification --------
    omega_r = 0.01  # track signals below this frequency
    gamma_r = 0.01  # tracking error below this value
    lt.add_spec_ref_tracking(gamma_r, omega_r, dB_flag)

    #----------- noise specification --------
    omega_n = 20    # attenuate noise above this frequency
    gamma_n = 0.01  # attenuate noise by this amount
    lt.add_spec_noise(gamma_n, omega_n, dB_flag)

    ## plot the effect of adding the new compensator terms
    mag, phase, omega = bode(Plant * C, dB=dB_flag,
                             omega=np.logspace(-4, 5),
                             plot=True,
                             label=r"$C_{\theta,in}(s)$"+
                                   r"$P_{\theta,in}(s)$",
                             margins=True)

    gm, pm, Wcg, Wcp = margin(Plant * C)
    print("for final C*P:")
    print(" pm: ", pm, " Wcp: ", Wcp, "gm: ", gm, " Wcg: ", Wcg)

    fig = plt.gcf()
    fig.axes[0].legend()
    fig.axes[0].grid(True)
    fig.axes[1].grid(True)
    fig.axes[0].set_title('Bode Diagram')
    plt.show()

    ############################################
    # now check the closed-loop response
    ############################################
    # Closed loop transfer function from R to Y
    CLOSED_R_to_Y = (Plant * C / (1.0 + Plant * C))
    # Closed loop transfer function from R to U
    CLOSED_R_to_U = (C / (1.0 + Plant * C))

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
