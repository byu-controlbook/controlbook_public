# loopshape_tool.py
# Tools for loopshaping
# Modification history:
#   4/17/2023 - RWB
import matplotlib.pyplot as plt
from numpy import log10, ones, size, divide, logspace, square, sqrt
from control import bode, tf


def spec_noise(gamma_n, omega_n, dB_flag=True):
    # Plot noise specification
    # attenuate noise by gamma_n for all noise signals
    # with frequency content above omega_n
    w = logspace(log10(omega_n), 2 + log10(omega_n))
    fig = plt.gcf()
    if dB_flag == False:
        fig.axes[0].plot(w,gamma_n*ones(size(w)),'g')
        # fig.axes[0].scatter(omega_n, gamma_n, facecolors='none', 
        #                     edgecolors='red', label='noise spec')
    else:
        fig.axes[0].plot(w,20*log10(gamma_n)*ones(size(w)),'g')
        # fig.axes[0].scatter(omega_n, 20.0* log10(gamma_n), facecolors='none', 
        #                     edgecolors='red', label='noise spec')


def spec_disturbance(gamma_d, omega_d, plant, dB_flag=True):
    # Plot input disturbance specification
    # reject input disturbances by gamma_d for all disturbance signals
    # with frequency content below omega_d
    # plant is the transfer function
    fig = plt.gcf()
    w = logspace(log10(omega_d)-2, log10(omega_d));
    mag, phase, omega = bode(plant, dB=False, omega=w, plot=False)
    if dB_flag == False:
        fig.axes[0].plot(w, mag/gamma_d,'g')
        # fig.axes[0].scatter(omega_d, 1. / gamma_d * mag, facecolors='none', 
        #                     edgecolors='green', label='$d_{in}$ spec')
    else:
        fig.axes[0].plot(w,20*log10(1/gamma_d)*ones(size(mag))+20*log10(mag),'g')
        # fig.axes[0].scatter(omega_d, 20. * log10(1. / gamma_d * mag), facecolors='none', 
        #                     edgecolors='green', label='$d_{in}$ spec')


def spec_track_ref(gamma_r, omega_r, dB_flag=True):
    # Plot reference tracking specification
    # track references with error gamma_r for all references signals
    # with frequency content below omega_r
    w = logspace(log10(omega_r)-2, log10(omega_r))
    fig = plt.gcf()
    if dB_flag == False:
        fig.axes[0].plot(w, (1./gamma_r)*ones(size(w)), 'g')
        # fig.axes[0].scatter(omega_r, 1. / gamma_r, facecolors='none', 
        #                     edgecolors=[1, 0, 1], label='ref tracking spec')
    else:
        fig.axes[0].plot(w, 20*log10(1./gamma_r)*ones(size(w)), 'g')
        # fig.axes[0].scatter(omega_r, 20. * log10(1. / gamma_r), facecolors='none', 
        #                     edgecolors=[1, 0, 1], label='ref tracking spec')


def spec_track_step(gamma_r, dB_flag=True):
    # Plot step tracking specification
    # track step input with error gamma_r 
    # PC must be above spec as omega -> 0
    w = logspace(-5, 0)
    fig = plt.gcf()
    if dB_flag == False:
        fig.axes[0].plot(w, (1./gamma_r-1)*ones(size(w)), 'g')
        # fig.axes[0].loglog(w, (1./ gamma_r - 1) * ones(size(w)),
        #          '.', color=[0, 0, 1], label = 'step tracking spec')
    else:
        fig.axes[0].plot(w, 20.*log10(1./gamma_r-1)*ones(size(w)), 'g')
        # fig.axes[0].semilogx(w, 20.0 * log10(1 / gamma_r - 1) * ones(len(w)),
        #          '.', color=[0, 0, 1], label = 'step tracking spec')


def spec_track_ramp(gamma_r, dB_flag=True):
    # Plot ramp tracking specification
    # track ramp input with error gamma_r 
    # PC must be above spec as omega -> 0
    w = logspace(-5, 0)
    fig = plt.gcf()
    if dB_flag == False:
        fig.axes[0].plot(w, divide(1./ gamma_r, w), 'g')
        # fig.axes[0].loglog(w, divide(1./ gamma_r, w),
        #          '.', color=[0, 0, 1], label = 'ramp tracking spec');
    else:
        fig.axes[0].plot(w, 20.*log10(divide(1./ gamma_r, w)), 'g')
        # fig.axes[0].semilogx(w, 20.0 * log10(divide(1./ gamma_r, w)),
        #          '.', color=[0, 0, 1], label = 'ramp tracking spec')


def spec_track_parabola(gamma_r, dB_flag=True):
    # Plot parabola tracking specification
    # track parabola input with error gamma_r 
    # PC must be above spec as omega -> 0
    w = logspace(-5, 0)
    fig = plt.gcf()
    if dB_flag == False:
        fig.axes[0].plot(w, divide(1./ gamma_r, square(w)), 'g')
        # fig.axes[0].loglog(w, divide(1./ gamma_r, square(w)),
        #          '.', color=[0, 0, 1], label = 'parabola tracking spec');
    else:
        fig.axes[0].plot(w, 20.*log10(divide(1./ gamma_r, square(w))), 'g')
        # fig.axes[0].semilogx(w, 20.0 * log10(divide(1./ gamma_r, square(w))),
        #          '.', color=[0, 0, 1], label = 'parabola tracking spec')


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
    return tf([sqrt(M), w], [1.0, w*sqrt(M)])


def notch(p1, p2, M):
    # notch filter: used for prefilter
    # p1 - frequency where notch starts
    # p2 - frequency where notch ends
    # M: depth of notch
    return tf([1.0, (p1*M+p2/M), p1*p2], [1.0, (p1+p2), p1*p2])


def notch2(ws, M):
    '''
        Generates a notch filter defined by frequency of interest (ws) and gain M. 
    '''
    return tf([1, 2*sqrt(M)*ws, M*ws**2], [1, (M+1)*ws, M*ws**2])
