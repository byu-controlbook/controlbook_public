# Single link arm Parameter File
import hw16 as P16
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
    # Calculate the phase and gain margins
    gm, pm, Wcg, Wcp = cnt.margin(sys_open_loop)
    if dB_flag:
        gm = cnt.mag2db(gm)
    print("Inner Loop:")
    print(f"\tgain margin: {gm:.2f}, at Wc = {Wcg:.2f}")
    print(f"\tphase margin: {pm:.2f}, at Wc = {Wcp:.2f}")

    # display bode plots of transfer functions
    fig = plt.figure()
    axes = fig.subplots(2, sharex=True)

    # this makes two bode plots for open and closed loop
    cnt.bode(
        [sys_open_loop, sys_closed_loop],
        dB=dB_flag,
        ax=axes,
        label=["Open-loop", "Closed-loop"],
    )

    # now we can add lines to show where we calculated the GM and PM
    axes[0].plot([Wcg, Wcg], axes[0].get_ylim(), "k--", label="Gain Margin")
    axes[1].plot([Wcp, Wcp], axes[1].get_ylim(), "b--", label="Phase Margin")

    axes[0].legend()
    axes[1].legend()

    fig.suptitle("Bode Plots: Single Link Arm")
    axes[0].set_title(f"GM: {gm:.2f}, PM: {pm:.2f}")

    plt.show()
