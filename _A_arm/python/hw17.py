# Single link arm Parameter File
import hw16 as P16
import loopshape_tools as ls
import control as cnt
import matplotlib.pyplot as plt

# flag to define if using dB or absolute scale for M(omega)
dB_flag = False

# assigning plant and controller from past HW (to make sure
# we don't introduce additional errors)
Plant = P16.Plant
C_pid = P16.C_pid

sys_open_loop = C_pid * Plant
sys_closed_loop = sys_open_loop / (1 + sys_open_loop)

if __name__ == "__main__":
    # display the phase and gain margins
    ls.print_margins(sys_open_loop, "Open-loop", dB_flag)

    # display bode plots of transfer functions
    fig = plt.figure()
    axes = fig.subplots(2, sharex=True)
    fig.suptitle("Bode Plots: Single Link Arm")

    # this makes two bode plots for open and closed loop
    cnt.bode(
        [sys_open_loop, sys_closed_loop],
        dB=dB_flag,
        ax=axes,
        display_margins=True,
        label=["Open-loop", "Closed-loop"],
    )

    fig.tight_layout()
    plt.show()
