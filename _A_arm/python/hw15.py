# Single link arm Parameter File
import armParam as P
from control import tf, bode
import matplotlib.pyplot as plt

# flag to define if using dB or absolute scale for M(omega)
dB_flag = False

# Compute plant transfer functions
th_e = 0
Plant = tf([3.0/P.m/P.ell**2],
           [1, 3.0*P.b/P.m/P.ell**2, 0.0])

if __name__ == '__main__':
    # Bode plot for the plant
    fig = plt.figure()
    bode(Plant, dB=dB_flag, margins=False)
    fig.axes[0].set_title('P(s) for arm')

    # if you want specific values at specific frequencies, you can
    # do the following (but the magnitudes are absolute, not dB)
    mag, phase, omega = bode(Plant, plot=False,
                             omega = [0.3, 10.0, 1000.0])

    print('Close window to end program')
    plt.show()

