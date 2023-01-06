# satellite parameter file
import satelliteParam as P
from ctrlPID import ctrlPID
import hw15 as P15
from control import tf, bode
import matplotlib.pyplot as plt
import numpy as np

P10 = ctrlPID()

# flag to define if using dB or absolute scale for M(omega)
dB_flag = P15.dB_flag

# Compute inner and outer open-loop transfer functions
P_in = tf([1],[P.Js+P.Jp,0,0])
P_out = tf([P.b/P.Jp, P.k/P.Jp], [1, P.b/P.Jp, P.k/P.Jp])

# Compute controller transfer functions

# PD control xfer function for inner loop
C_in = tf([(P10.kd_th+P10.sigma*P10.kp_th), P10.kp_th], [P10.sigma, 1])

# PID xfer function for outer loop
C_out = tf([(P10.kd_phi+P10.kp_phi*P10.sigma),
            (P10.kp_phi+P10.ki_phi*P10.sigma),
            P10.ki_phi],
           [P10.sigma, 1, 0])

if __name__ == '__main__':

    omegas = np.logspace(-2, 3, 1000)

    # display bode plots of transfer functions
    fig1 = plt.figure()
    bode([P_in, P_in*C_in, tf([1.0], [1.0, 0.0, 0.0])],
         omega=omegas, dB=dB_flag)
    plt.legend(['$P_{in}(s)$', '$C_{in}(s)P_{in}(s)$', '$\\frac{1}{s^2}$'])
    fig1.axes[0].set_title('Satellite, Inner Loop')

    fig2 = plt.figure()
    bode([P_out, P_out*C_out], omega=omegas, dB=dB_flag)
    plt.legend(['$P_{out}(s)$', '$C_{out}(s)P_{out}(s)$'])
    fig2.axes[0].set_title('Satellite, Outer Loop')

    print('Close window(s) to end program')
    plt.show()
