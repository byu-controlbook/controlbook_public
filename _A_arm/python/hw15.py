# Single link arm Parameter File
import armParam as P
import numpy as np
import control as cnt
import matplotlib.pyplot as plt

# flag to define if using dB or absolute scale for M(omega)
dB_flag = False

# Compute plant transfer functions
th_e = 0
Plant = cnt.tf(
    [3.0 / P.m / P.ell**2],  # numerator
    [1, 3.0 * P.b / P.m / P.ell**2, 0.0],  # denominator
)

if __name__ == "__main__":
    # Bode plot for the plant
    fig = plt.figure()
    axes = fig.subplots(2, sharex=True)
    cnt.bode(Plant, dB=dB_flag, display_margins=False, ax=axes, label="P(s)")
    fig.suptitle("Bode Plot: Single Link Arm")
    axes[0].legend()

    # if you want specific values at specific frequencies, you can
    # do the following (but the magnitudes are absolute, not dB)
    mag, phase, omega = cnt.frequency_response(Plant, omega=[0.3, 10.0, 1000.0])

    np.set_printoptions(precision=4, suppress=True)
    print("magnitude:\n", mag)
    print("omega:\n", omega)

    print("Close window to end program")

    plt.show()
