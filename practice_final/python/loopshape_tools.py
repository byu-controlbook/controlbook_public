import matplotlib.pyplot as plt
from control import TransferFunction as tf
from control import bode
import numpy as np


def spec_noise(gamma, omega, flag):
    # attenuate noise above omega_n by gamma_n
    w = np.logspace(np.log10(omega), np.log10(omega) + 4)
    if flag is True:
        plt.subplot(211)
        plt.plot(w,
                 (20 * np.log10(gamma)) * np.ones(len(w)),
                 color='g',
                 label='noise spec')


def spec_disturbance(gamma, omega, flag):
    # reject disturbance above omega by gamma
    w = np.logspace(np.log10(omega)-4, np.log10(omega))
    if flag is True:
        plt.subplot(211)
        plt.plot(w,
                 20*np.log10(1.0/gamma)*np.ones(len(w)),
                 color='g',
                 label='disturbance spec')

def spec_input_disturbance(gamma_d, omega_d, system, dB_flag=False):
    fig = plt.gcf()
    w = np.logspace(np.log10(omega_d)-2, np.log10(omega_d))
    mag, phase, omega = bode(system, dB=dB_flag, omega=w, plot=False)
    if dB_flag == False:
        fig.axes[0].loglog(w, 1. / gamma_d * np.ones(len(w))*mag,
                 '--', color=[1, 0, 0], label='$d_{in}$ spec')
    else:
        fig.axes[0].semilogx(w, 20. * np.log10(1./gamma_d * np.ones(len(w))*mag),
                 '--', color=[1, 0, 0], label='$d_{in}$ spec')


def spec_tracking(gamma, omega, flag):
    # track references below omega by gamma
    w = np.logspace(np.log10(omega) - 2, np.log10(omega))
    if flag is True:
        plt.subplot(211)
        plt.plot(w,
                 20*np.log10(1/gamma)*np.ones(len(w)),
                 color='g',
                 label='tracking spec')


def spec_tracking_step(gamma, flag):
    # track step by gamma
    omega = 0.01
    w = np.logspace(np.log10(omega)-4, np.log10(omega))
    if flag is True:
        plt.subplot(211)
        plt.plot(w,
                 20*np.log10(1.0/gamma),
                 color='g',
                 label='tracking spec')


def spec_tracking_ramp(gamma, flag):
    # track ramp by gamma
    omega = 0.01
    w = np.logspace(np.log10(omega)-4, np.log10(omega))
    if flag is True:
        plt.subplot(211)
        plt.plot(w,
                 20*np.log10(1.0/gamma)-20*np.log10(w),
                 color='g',
                 label='tracking spec')


def proportional(kp):
    # proportional control: change cross over frequency
    return tf([kp], [1])


def integral(ki):
    # integral control: increase steady state tracking and dist rejection
    # ki: frequency at which integral action ends
    return tf([1, ki], [1, 0])


def lag(z, M):
    # phase lag: add gain at low frequency
    # z: frequency at which gain ends
    # M: separation between pole and zero
    return tf([1, z], [1, z/M])


def lpf(p):
    # low pass filter: decrease gain at high frequency (noise)
    # p: lpf cutoff frequency
    return tf(p, [1, p])


def lead(w, M):
    # phase lead: increase PM (stability)
    # w: location of maximum frequency bump
    # M: separation between zero and pole
    return tf([np.sqrt(M), w], [1.0, w*np.sqrt(M)])


def notch(p1, p2, M):
    # notch filter: used for prefilter
    # p1 - frequency where notch starts
    # p2 - frequency where notch ends
    # M: depth of notch
    return tf([1.0, (p1*M+p2/M), p1*p2], [1.0, (p1+p2), p1*p2])
