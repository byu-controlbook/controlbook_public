# Single link arm Parameter File
import armParam as P
from ctrlPID import ctrlPID
import hw15 as P15
import control as cnt
import matplotlib.pyplot as plt

P10 = ctrlPID()
kp, ki, kd, sigma = P10.kp, P10.ki, P10.kd, P10.sigma

# flag to define if using dB or absolute scale for M(omega)
dB_flag = False

# Assign plan from previous homework solution
Plant = P15.Plant

# Compute transfer function of controller
C_pid = cnt.tf(
    [(kd + kp * sigma), (kp + ki * sigma), ki],
    [sigma, 1, 0],
)

if __name__ == "__main__":
    # display bode plots of transfer functions
    fig = plt.figure()
    axes = fig.subplots(2, sharex=True)
    cnt.bode(
        [Plant, Plant * C_pid],
        omega_limits=[10 ** (-5), 10 ** (3)],
        dB=dB_flag,
        ax=axes,
        label=["P(s)", "C(s)P(s)"],
    )
    fig.suptitle("Bode Plot: Single Link Arm")
    axes[0].legend()

    fig.tight_layout()
    plt.show()
