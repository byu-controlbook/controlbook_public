import matplotlib.pyplot as plt
from numpy import log10, ones, size, divide, logspace, square, sqrt
from control import bode, tf


def spec_noise(gamma_n, omega_n, dB_flag=True):
    '''
        Add noise specification, defined by gamma_n for remaining magnitude 
        of error, and omega_n for the frequency at which it needs to happen. 
    '''
    w = logspace(log10(omega_n), log10(omega_n)+2)
    fig = plt.gcf()
    if dB_flag == False:
        fig.axes[0].plot(w,gamma_n*ones(size(w)),'g')
        # fig.axes[0].scatter(omega_n, gamma_n, facecolors='none', 
        #                     edgecolors='red', label='noise spec')
    else:
        fig.axes[0].plot(w, 20*log10(gamma_n)*ones(size(w)),'g')
        # fig.axes[0].scatter(omega_n, 20.0* log10(gamma_n), facecolors='none', 
        #                     edgecolors='red', label='noise spec')


def spec_disturbance(gamma_d, omega_d, system, dB_flag=True):
    '''
        Add disturbance input specification, defined by gamma_d, 
        for remaining magnitude of error, and omega_d for the frequency 
        at which it needs to happen. "system" is a transfer function
        object that represents the plant before any control is added. 
    '''    
    fig = plt.gcf()
    w = logspace(log10(omega_d)-2, log10(omega_d));
    mag, phase, omega = bode(system, dB=False, omega=w, Plot=False)
    if dB_flag == False:
        fig.axes[0].plot(w, mag/gamma_d,'g')
        # fig.axes[0].scatter(omega_d, 1. / gamma_d * mag, facecolors='none', 
        #                     edgecolors='green', label='$d_{in}$ spec')
    else:
        fig.axes[0].plot(w,20*log10(1/gamma_d)*ones(size(mag))+20*log10(mag),'g')
        # fig.axes[0].scatter(omega_d, 20. * log10(1. / gamma_d * mag), facecolors='none', 
        #                     edgecolors='green', label='$d_{in}$ spec')


def spec_track_ref(gamma_r, omega_r, dB_flag=True):
    '''
        Add reference tracking specification, defined by gamma_r for amount 
        of remaining magnitude of error, and omega_r for the frequency at 
        which it needs to happen. 
    '''
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
    '''
        Add step tracking constant error requirement, specified by gamma_r, (the
        amount of residual error), and a mag. ratio slope at low frequencies of 0.  
    '''
    w = logspace(-5, 0)
    fig = plt.gcf()
    if dB_flag == False:
        fig.axes[0].plot(w, (1./gamma_r-1)*ones(size(w)), 'g')
        # fig.axes[0].loglog(w, (1./ gamma_r - 1) * ones(size(w)),
        #          '.', color=[0, 0, 1], label = 'step tracking spec')
    else:
        fig.axes[0].plot(w, 20.*log10(1./gamma_r-1)*ones(size(w)), 'g')
        # fig.axes[0].semilogx(w, 20.0 * log10(1 / gamma_r - 1) * np.ones(len(w)),
        #          '.', color=[0, 0, 1], label = 'step tracking spec')


def spec_track_ramp(gamma_r, dB_flag=True):
    '''
        Add ramp tracking constant error requirement, specified by gamma_r, (the
        amount of residual error), and a mag. ratio slope at low frequencies of 1.  
    '''
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
    '''
        Add parabola tracking constant error requirement, specified by gamma_r, (the
        amount of residual error), and a mag. ratio slope at low frequencies of 2.  
    '''
    w = logspace(-5, 0)
    fig = plt.gcf()
    if dB_flag == False:
        fig.axes[0].plot(w, divide(1./ gamma_r, square(w)), 'g')
        # fig.axes[0].loglog(w, divide(1./ gamma_r, np.square(w)),
        #          '.', color=[0, 0, 1], label = 'parabola tracking spec');
    else:
        fig.axes[0].plot(w, 20.*log10(divide(1./ gamma_r, square(w))), 'g')
        # fig.axes[0].semilogx(w, 20.0 * log10(divide(1./ gamma_r, np.square(w))),
        #          '.', color=[0, 0, 1], label = 'parabola tracking spec')


def proportional(kp):
    '''
        Generate transfer function for single gain or proportional control. This 
        term can be used to change the crossover frequency. 
    '''
    return tf([kp], [1])


def integral(ki):
    '''
        Generate transfer function for PI controller defined by gain "ki". 
        Can be used to change the slope of the mag. ratio at low frequencies. 
    '''
    return tf([1, ki], [1, 0])


def lag(z, M):
    '''
        Makes a lag controller to add gain at low frequency such that: 
            phase lag (|p|<|z|): 
            M - low frequency gain
            z - frequency where effect roughly ends
    '''
    return tf([1, z], [1, z / M])


def lpf(p):
    '''
        generates a transfer function for a low-pass filter with a
        cutoff frequency of "p" 
    '''
    return tf(p, [1, p])


def lead(w, M):
    '''
        Generates a lead controller transfer function defined by:
            M - the amount of gain at high frequencies (corresponds to PM boost)
            w - frequency where max PM will be added
    '''
    return tf([M, M*w / sqrt(M)], [1, w * sqrt(M)])


def notch(ws, M):
    '''
        Generates a notch filter defined by frequency of interest (ws) and gain M. 
    '''
    return tf([1, 2 * sqrt(M) * ws, M * ws**2], [1, (M + 1) * ws, M * ws**2])
