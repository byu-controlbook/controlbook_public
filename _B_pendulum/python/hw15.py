# Inverted Pendulum Parameter File
import pendulumParam as P
from control import tf, bode
import matplotlib.pyplot as plt

# flag to define if using dB or absolute scale for M(omega)
dB_flag = True

# Compute inner and outer open-loop transfer functions
temp = (P.m1*P.ell/6.0+P.m2*2*P.ell/3.0)
P_in = tf([-1/temp],
          [1, 0, -(P.m1+P.m2)*P.g/temp])
P_out = tf([-2*P.ell/3.0, 0, P.g], [1, 0, 0])

if __name__=="__main__":
    # Plot the open loop bode plots for the inner loop
    fig1 = plt.figure()
    bode(P_in, dB=dB_flag)
    fig1.axes[0].set_title('$P_{in}(s)$')

    fig2 = plt.figure()
    bode(P_out, dB=dB_flag)
    fig2.axes[0].set_title('$P_{out}(s)$')

    print('Close window to end program')
    plt.show()
