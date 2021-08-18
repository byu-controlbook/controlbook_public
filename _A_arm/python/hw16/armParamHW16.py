# Single link arm Parameter File
import sys
sys.path.append('..')  # add parent directory
import armParam as P
sys.path.append('../hw10')  # add parent directory
sys.path.append('../hw15')  # add parent directory
import armParamHW10 as P10
import armParamHW15 as P15
from control import tf, bode
import matplotlib.pyplot as plt


# flag to define if using dB or absolute scale for M(omega)
dB_flag = P15.dB_flag

# Assign plan from previous homework solution
Plant = P15.Plant

# Compute transfer function of controller
C_pid = tf([(P10.kd+P10.kp*P.sigma), (P10.kp+P10.ki*P.sigma), P10.ki],
           [P.sigma, 1, 0])

if __name__=="__main__":
    # display bode plots of transfer functions
    fig = plt.figure()
    bode([Plant, Plant*C_pid], dB=dB_flag)
    fig.suptitle('Single Link Arm')
    plt.legend(['P(s)', 'C(s)P(s)'])
    print('Close plot window to end program')
    plt.show()
