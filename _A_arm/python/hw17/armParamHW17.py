# Single link arm Parameter File
import sys
sys.path.append('..')  # add parent directory
import armParam as P
sys.path.append('../hw16')  # add parent directory
import armParamHW16 as P16
from control import bode, margin
import matplotlib.pyplot as plt

# flag to define if using dB or absolute scale for M(omega)
dB_flag = P16.dB_flag

# assigning plant and controller from past HW (to make sure we don't introduce additional errors)
Plant = P16.Plant
C_pid = P16.C_pid

if __name__=="__main__":

    # display bode plots of transfer functions
    fig1 = plt.figure()
    bode([Plant, Plant*C_pid, Plant*C_pid/(1+Plant*C_pid)], dB=dB_flag)
    plt.legend(('No control - P(s)', 'C(s)P(s)', 'Closed-loop PID'))
    fig1.axes[0].set_title('Single Link Arm')

    fig2 = plt.figure()
    bode([Plant, Plant*C_pid], dB=dB_flag, margins=True)
    fig2.axes[0].set_title('Single Link Arm - Stability Margins')

    # Calculate the phase and gain margin and frequencies where they are calculated.
    # Wcp is the crossover frequency used to find phase margin.
    gm, pm, Wcg, Wcp = margin(Plant*C_pid)

    if dB_flag:
        print("gm: ", mag2db(gm)," pm: ", pm," Wcg: ", Wcg, " Wcp: ", Wcp)
    else:
        print("gm: ",gm," pm: ", pm," Wcg: ", Wcg, " Wcp: ", Wcp)

    plt.show()
