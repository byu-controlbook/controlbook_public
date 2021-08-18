# Satellite Parameter File
import sys
sys.path.append('..')  # add parent directory
import satelliteParam as P
sys.path.append('../hw16')  # add parent directory
import satelliteParamHW16 as P16
from control import *
import matplotlib.pyplot as plt

# flag to define if using dB or absolute scale for M(omega)
dB_flag = P16.dB_flag

# Compute inner and outer open-loop transfer functions
P_in = tf([1/P.Js], [1, P.b/P.Js, P.k/P.Js])
P_out = tf([P.b/P.Jp, P.k/P.Jp], [1, P.b/P.Jp, P.k/P.Jp])

# Compute the controller transfer functions from HW10 (to make sure we don't introduce additional errors)
C_in = P16.C_in
C_out = P16.C_out

if __name__=="__main__":

    # Plot the closed loop and open loop bode plots for the inner loop
    fig1 = plt.figure()
    bode([P_in*C_in, P_in*C_in/(1+P_in*C_in)], dB=dB_flag)
    plt.legend(['$C_{in}(s)P_{in}(s)$', r'Closed-loop ${\theta}$ Controller'])
    fig1.axes[0].set_title(r'Inner Loop Controller for ${\theta}$')

    fig2 = plt.figure()
    bode([P_out*C_out, P_out*C_out/(1+P_out*C_out)], dB=dB_flag)
    plt.legend(['$C_{out}(s)P_{out}(s)$', r'Closed-loop ${\phi}$ Controller'])
    fig2.axes[0].set_title(r'Outer Loop Controller for ${\phi}$')

    # Calculate the phase and gain margin
    ## inner loop
    gm, pm, Wcg, Wcp = margin(P_in*C_in)
    if dB_flag:
        print("Inner Loop: gm: ", mag2db(gm)," pm: ", pm," Wcg: ", Wcg, " Wcp: ", Wcp)
    elif dB_flag == False:
        print("Inner Loop: gm: ",gm," pm: ", pm," Wcg: ", Wcg, " Wcp: ", Wcp)

    ## outer loop
    gm, pm, Wcg, Wcp = margin(P_out*C_out)
    if dB_flag:
        print("Outer Loop: gm: ", mag2db(gm)," pm: ", pm," Wcg: ", Wcg, " Wcp: ", Wcp)
    elif dB_flag == False:
        print("Outer Loop: gm: ",gm," pm: ", pm," Wcg: ", Wcg, " Wcp: ", Wcp)

    print('Close window to end program')
    plt.show()
