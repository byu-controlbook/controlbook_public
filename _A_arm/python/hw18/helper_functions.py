import matplotlib.pyplot as plt
import numpy as np
from control import bode, tf


def add_spec_noise(gamma_n, omega_n, dB_flag = False):
    w = np.logspace(np.log10(omega_n), 2 + np.log10(omega_n))
    fig = plt.gcf()
    if dB_flag == False:
        fig.axes[0].loglog(w, gamma_n * np.ones(len(w)),
                 '--', color=[0, 1, 0], label = 'noise spec')
    else:
        fig.axes[0].semilogx(w, 20.0* np.log10(gamma_n) * np.ones(len(w)),
                 '--', color=[0, 1, 0], label = 'noise spec')


def add_spec_input_disturbance(gamma_d, omega_d, system, dB_flag=False):
    fig = plt.gcf()
    w = np.logspace(np.log10(omega_d)-2, np.log10(omega_d))
    mag, phase, omega = bode(system, dB=dB_flag, omega=w, Plot=False)
    if dB_flag == False:
        fig.axes[0].loglog(w, 1. / gamma_d * np.ones(len(w))*mag,
                 '--', color=[1, 0, 0], label='$d_{in}$ spec')
    else:
        fig.axes[0].semilogx(w, 20. * np.log10(1./gamma_d * np.ones(len(w))*mag),
                 '--', color=[1, 0, 0], label='$d_{in}$ spec')

def add_spec_ref_tracking(gamma_r, omega_r, dB_flag = False):
    w = np.logspace(np.log10(omega_r)-2, np.log10(omega_r))
    fig = plt.gcf()
    if dB_flag == False:
        fig.axes[0].loglog(w, 1./gamma_r*np.ones(len(w)),
                 '.', color=[1, 0, 0],label='ref tracking spec')
    else:
        fig.axes[0].semilogx(w, 20.*np.log10(1./gamma_r)*np.ones(len(w)),
                '.', color=[1, 0, 0],label='ref tracking spec')

def add_spec_tracking_step(gamma_r, dB_flag = False):
    w = np.logspace(-5, 0);
    fig = plt.gcf()
    if dB_flag == False:
        fig.axes[0].loglog(w, (1./ gamma_r - 1) * ones(size(w)),
                 '.', color=[0, 1, 0], label = 'step tracking spec');
    else:
        fig.axes[0].semilogx(w, 20.0 * np.log10(1 / gamma_r - 1) * np.ones(len(w)),
                 '.', color=[0, 1, 0], label = 'step tracking spec')

# add_spec_tracking_ramp(gamma_r, flag_abs)
# w = logspace(-4, 0);
# if flag_abs
#     plot(w, (1 / gamma_r). / (w), 'g')
# else
#     plot(w, 20 * log10(1 / gamma_r) - 20 * log10(w), 'g')
# end
#
# end
#
# % --- steady
# state
# tracking
# of
# parabola - --
# % track
# ramp
# to
# within
# gamma_r
# function
# add_spec_tracking_parabola(gamma_r, flag_abs)
# w = logspace(-5, 0);
# if flag_abs
#     plot(w, (1 / gamma_r). / (w. ^ 2), 'g')
# else
#     plot(w, 20 * log10(1 / gamma_r) - 40 * log10(w), 'g')
# end
# end
#

#

def get_control_proportional(kp):

    return tf([kp], [1])

def get_control_integral(ki):
    Integrator = tf([1, ki], [1, 0])

    return Integrator


# phase lag (|p|<|z|): add gain at low frequency
#                      (tracking, dist rejection)
# low frequency gain = K*z/p
# high frequency gain = K
def get_control_lag(z, M):
    Lag = tf([1, z], [1, z / M])

    return Lag

def get_control_lpf(p):
    LPF = tf(p, [1, p])

    return LPF

# phase lead (|p|>|z|): increase PM (stability)
# low frequency gain = K*z/p
# high frequency gain = K
def get_control_lead(omega_lead, M):
    Lead = tf([M, M*omega_lead / np.sqrt(M)], [1, omega_lead * np.sqrt(M)])

    return Lead

def get_control_notch(ws, M):
    Notch = tf([1, 2 * np.sqrt(M) * ws, M * ws**2], [1, (M + 1) * ws, M * ws**2]);

    return Notch