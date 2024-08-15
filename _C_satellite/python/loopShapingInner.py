from control import tf, margin, bode, tf2ss, step_response, mag2db
import numpy as np
import matplotlib.pyplot as plt
import satelliteParam as P
from ctrlPID import ctrlPID
import hw16 as P16
import loopshape_tools as lt
P10 = ctrlPID()

# flag to define if using dB or absolute scale for M(omega)
dB_flag = P16.dB_flag

# Compute open-loop transfer functions as described in Chapter 18
# Plant = tf([P10.sigma, 1],
#            [P10.sigma*P.Js, P10.sigma*P.b+P.Js,
#             P10.sigma*P.k+P.b+P10.kd_th, P.k])
Plant = tf([1.0], [(P.Js+P.Jp), 0.0, 0.0])

#########################################
#   Control Design
#########################################
C = tf([1], [1]) \
    * lt.get_control_lead(omega_lead=0.41, M=15.0)\

###########################################################
# Extracting coefficients for controller
###########################################################
C_num = np.asarray(C.num[0])
C_den = np.asarray(C.den[0])


if __name__ == '__main__':
    
    # calculate bode plot and gain and phase margin
    # for original PID * plant dynamics
    mag, phase, omega = bode(Plant, dB=dB_flag,
                            omega=np.logspace(-4, 5),
                            plot=True, label="$P_{\theta,in}(s)$")

    gm, pm, Wcg, Wcp = margin(Plant)
    print("for original system:")
    print(" pm: ", pm, " Wcp: ", Wcp, "gm: ", gm, " Wcg: ", Wcg)

    #########################################
    #   Define Design Specifications
    #########################################
    lt.add_spec_ref_tracking(gamma_r=0.01, omega_r=0.001, dB_flag=dB_flag)
    lt.add_spec_noise(gamma_n=0.01, omega_n=20, dB_flag=dB_flag)

    ## plot the effect of adding the new compensator terms
    mag, phase, omega = bode(Plant * C, dB=dB_flag,
                             omega=np.logspace(-4, 5),
                             plot=True, label="$C_{\theta,in}(s) P_{\theta,in}(s)$",
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
