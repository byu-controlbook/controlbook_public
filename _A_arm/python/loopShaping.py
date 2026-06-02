import matplotlib.pyplot as plt
import control as cnt
import numpy as np
import hw16 as P16
import loopshape_tools as ls

# flag to define if using dB or absolute scale for M(omega)
dB_flag = False

# assigning plant and controller from past HW
# (to make sure we don't introduce additional errors)
Plant = P16.Plant
C_pid = P16.C_pid
sys_orig_OL = C_pid * Plant

###################################################################
#   Control Design
###################################################################
C = C_pid

# Because our PM starts out good enough, we will skip to adding a
# low-pass filter and lag compensator to meet the low-frequency and
# high-frequency requirements.
C_lpf = ls.get_control_lpf(90.0)
C_lag = ls.get_control_lag(z=5, M=90.0)
C = C * C_lpf * C_lag

# after checking the requirements, we need to add a lead compensator now,
# along with a proportional gain and 2nd low-pass filter to meet the noise
# specification.
C_lead = ls.get_control_lead(omega_lead=10, M=10.0)
C = C * C_lead

mag, _, _ = cnt.frequency_response(Plant * C, omega=[6.35])
C_k = ls.get_control_proportional(1 / mag[0])
C_lpf2 = ls.get_control_lpf(100.0)

# this is our final controller
C = C * C_k * C_lpf2

sys_final_OL = C * Plant

###########################################################
# add a prefilter to eliminate the overshoot
###########################################################
F = ls.get_control_lpf(p=2.0)


###########################################################
# Extracting coefficients for controller and prefilter
# (used in other files, not used in this file)
###########################################################
C_num = np.asarray(C.num[0])
C_den = np.asarray(C.den[0])
F_num = np.asarray(F.num[0])
F_den = np.asarray(F.den[0])


if __name__ == "__main__":
    fig1 = plt.figure()
    axes = fig1.subplots(2, sharex=True)
    fig1.suptitle("Open-Loop Bode Plots: Single Link Arm")

    # plot bode responses
    cnt.bode(
        [sys_orig_OL, sys_final_OL],
        dB=dB_flag,
        ax=axes,
        omega_limits=[10**-3, 10**5],
        display_margins=False,
        label=["$C_{pid}(s)P(s)$", "$C_{final}(s)P(s)$"],
    )

    # display gain and phase margins
    ls.print_margins(sys_orig_OL, "Original (C_pid) Open-Loop", dB_flag)
    ls.print_margins(sys_final_OL, "Final (C_final) Open-Loop", dB_flag)

    #########################################
    #   Define Design Specifications
    #########################################
    # ----------- noise specification --------
    omega_n = 1000
    improvement_factor = 10.0

    mag, _, _ = cnt.frequency_response(sys_orig_OL, omega=[omega_n])
    gamma_n = mag[0] / improvement_factor

    # plot noise specification area
    omega_max = axes[0].get_xlim()[1]
    x_pts = [omega_n, omega_max]
    y_upper = [1.0] * 2
    y_lower = [gamma_n] * 2
    if dB_flag:
        y_upper = cnt.mag2db(y_upper)
        y_lower = cnt.mag2db(y_lower)
    axes[0].fill_between(
        x_pts, y_upper, y_lower, color="red", alpha=0.2, label="noise spec"
    )

    # ----------- general tracking specification --------
    omega_d = 0.07
    improvement_factor = 10.0

    # plot disturbance specification line
    mag, phase, omega = cnt.frequency_response(sys_orig_OL, omega=[omega_d])

    x_pts = [omega_d, omega_d]
    y_pts = [mag[0], mag[0] * improvement_factor]
    if dB_flag:
        y_pts = cnt.mag2db(y_pts)
    axes[0].plot(x_pts, y_pts, "g", label="$d_{in}$ spec")

    axes[0].legend()

    ############################################
    # now check the closed-loop response with prefilter
    ############################################
    # Closed loop transfer function from R to Y - no prefilter
    R_to_Y_CL = Plant * C / (1.0 + Plant * C)
    # Closed loop transfer function from R to Y - with prefilter
    R_to_Y_with_F_CL = F * Plant * C / (1.0 + Plant * C)
    # Closed loop transfer function from R to U - no prefilter
    R_to_U_CL = C / (1.0 + Plant * C)
    # Closed loop transfer function from R to U - with prefilter
    R_to_U_with_F_CL = F * C / (1.0 + Plant * C)

    fig2 = plt.figure()
    axes = fig2.subplots(2, sharex=True)
    fig2.suptitle("Closed-Loop Bode Plots: Single Link Arm")

    cnt.bode(
        [R_to_Y_CL, R_to_Y_with_F_CL],
        dB=dB_flag,
        ax=axes,
        # plot_phase=False,
        label=[
            r"Closed-Loop $\frac{Y}{R}$ - no pre-filter",
            r"Closed-Loop $\frac{Y}{R}$ - with pre-filter",
        ],
    )
    axes[0].legend()

    # Step response plots
    fig3 = plt.figure()
    axes = fig3.subplots(2, sharex=True)
    fig3.suptitle("Closed-Loop Step Responses: Single Link Arm")

    ax = axes[0]
    T = np.linspace(0, 2, 100)
    _, yout_no_F = cnt.step_response(R_to_Y_CL, T)
    _, yout_F = cnt.step_response(R_to_Y_with_F_CL, T)
    ax.plot(T, yout_no_F, label="no prefilter")
    ax.plot(T, yout_F, label="with prefilter")
    ax.legend()
    ax.grid(True)
    ax.set_ylabel("Step Response")

    ax = axes[1]
    _, Uout = cnt.step_response(R_to_U_CL, T)
    _, Uout_F = cnt.step_response(R_to_U_with_F_CL, T)
    ax.plot(T, Uout, label="no prefilter")
    ax.plot(T, Uout_F, label="with prefilter")
    ax.set_ylabel("Control Effort")
    ax.set_xlabel("Time (s)")
    ax.grid(True)

    fig1.tight_layout()
    fig2.tight_layout()
    fig3.tight_layout()
    plt.show()
