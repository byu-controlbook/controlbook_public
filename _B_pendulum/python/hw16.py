from ctrlPID import ctrlPID
import hw15 as P15
from control import tf, bode
import matplotlib.pyplot as plt

P10 = ctrlPID()

# flag to define if using dB or absolute scale for M(omega)
dB_flag = P15.dB_flag

# Assign inner and outer open-loop transfer functions from previous HW solution
P_in = P15.P_in
P_out = P15.P_out

# Compute controller transfer functions
# PD control xfer function for inner loop
C_in = tf([P10.kd_th + P10.sigma * P10.kp_th, P10.kp_th], [P10.sigma, 1])

# PID xfer function for outer loop
C_out = tf([P10.kd_z + P10.kp_z * P10.sigma,
            P10.kp_z + P10.ki_z * P10.sigma,
            P10.ki_z],
           [P10.sigma, 1, 0])

if __name__=="__main__":
     # display bode plots of transfer functions
     fig1 = plt.figure()
     bode([P_in, P_in * C_in], dB=dB_flag)
     plt.legend(['$P_{in}(s)$', '$C_{in}(s)P_{in}(s)$'])
     fig1.axes[0].set_title('Inverted Pendulum, Inner Loop')


     fig2 = plt.figure()
     bode([P_out, P_out * C_out],
          omega_limits=[10**(-5), 10**(3)], dB=dB_flag)
     plt.legend(['$P_{out}(s)$', '$C_{out}(s)P_{out}(s)$'])
     fig2.axes[0].set_title('Inverted Pendulum, Outer Loop')


     print('Close window(s) to end program')
     plt.show()

