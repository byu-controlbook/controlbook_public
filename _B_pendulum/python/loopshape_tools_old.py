import matplotlib.pyplot as plt
import numpy as np
from control import bode, tf


def add_spec_noise(gamma_n, omega_n, dB_flag = False):
    '''
        Add noise specification, defined by gamma_n for remaining magnitude 
        of error, and omega_n for the frequency at which it needs to happen. 
    '''
    w = np.logspace(np.log10(omega_n), 2 + np.log10(omega_n))
    fig = plt.gcf()
    if dB_flag == False:
        fig.axes[0].scatter(omega_n, gamma_n, facecolors='none', 
                            edgecolors='red', label='noise spec')
    else:
        fig.axes[0].scatter(omega_n, 20.0* np.log10(gamma_n), facecolors='none', 
                            edgecolors='red', label='noise spec')


def add_spec_input_disturbance(gamma_d, omega_d, system, dB_flag=False):
    '''
        Add disturbance input specification, defined by gamma_d, 
        for remaining magnitude of error, and omega_d for the frequency 
        at which it needs to happen. "system" is a transfer function
        object that represents the plant before any control is added. 
    '''    
    fig = plt.gcf()
    mag, phase, omega = bode(system, dB=dB_flag, omega=omega_d, Plot=False)
    if dB_flag == False:
        fig.axes[0].scatter(omega_d, 1. / gamma_d * mag, facecolors='none', 
                            edgecolors='green', label='$d_{in}$ spec')
    else:
        fig.axes[0].scatter(omega_d, 20. * np.log10(1. / gamma_d * mag), facecolors='none', 
                            edgecolors='green', label='$d_{in}$ spec')

def add_spec_ref_tracking(gamma_r, omega_r, dB_flag = False):
    '''
        Add reference tracking specification, defined by gamma_r for amount 
        of remaining magnitude of error, and omega_r for the frequency at 
        which it needs to happen. 
    '''
    w = np.logspace(np.log10(omega_r)-2, np.log10(omega_r))
    fig = plt.gcf()
    if dB_flag == False:
        fig.axes[0].scatter(omega_r, 1. / gamma_r, facecolors='none', 
                            edgecolors=[1, 0, 1], label='ref tracking spec')
    else:
        fig.axes[0].scatter(omega_r, 20. * np.log10(1. / gamma_r), facecolors='none', 
                            edgecolors=[1, 0, 1], label='ref tracking spec')

def add_spec_tracking_step(gamma_r, dB_flag = False):
    '''
        Add step tracking constant error requirement, specified by gamma_r, (the
        amount of residual error), and a mag. ratio slope at low frequencies of 0.  
    '''
    w = np.logspace(-5, 0)
    fig = plt.gcf()
    if dB_flag == False:
        fig.axes[0].loglog(w, (1./ gamma_r - 1) * ones(size(w)),
                 '.', color=[0, 0, 1], label = 'step tracking spec')
    else:
        fig.axes[0].semilogx(w, 20.0 * np.log10(1 / gamma_r - 1) * np.ones(len(w)),
                 '.', color=[0, 0, 1], label = 'step tracking spec')

def add_spec_tracking_ramp(gamma_r, dB_flag = False):
    '''
        Add ramp tracking constant error requirement, specified by gamma_r, (the
        amount of residual error), and a mag. ratio slope at low frequencies of 1.  
    '''
    w = np.logspace(-5, 0)
    fig = plt.gcf()
    if dB_flag == False:
        fig.axes[0].loglog(w, np.divide(1./ gamma_r, w),
                 '.', color=[0, 0, 1], label = 'ramp tracking spec');
    else:
        fig.axes[0].semilogx(w, 20.0 * np.log10(np.divide(1./ gamma_r, w)),
                 '.', color=[0, 0, 1], label = 'ramp tracking spec')

def add_spec_tracking_parabola(gamma_r, dB_flag = False):
    '''
        Add parabola tracking constant error requirement, specified by gamma_r, (the
        amount of residual error), and a mag. ratio slope at low frequencies of 2.  
    '''
    w = np.logspace(-5, 0)
    fig = plt.gcf()
    if dB_flag == False:
        fig.axes[0].loglog(w, np.divide(1./ gamma_r, np.square(w)),
                 '.', color=[0, 0, 1], label = 'parabola tracking spec');
    else:
        fig.axes[0].semilogx(w, 20.0 * np.log10(np.divide(1./ gamma_r, np.square(w))),
                 '.', color=[0, 0, 1], label = 'parabola tracking spec')

def get_control_proportional(kp):
    '''
        Generate transfer function for single gain or proportional control. This 
        term can be used to change the crossover frequency. 
    '''

    return tf([kp], [1])

def get_control_integral(ki):
    '''
        Generate transfer function for PI controller defined by gain "ki". 
        Can be used to change the slope of the mag. ratio at low frequencies. 
    '''
    Integrator = tf([1, ki], [1, 0])

    return Integrator



def get_control_lag(z, M):
    '''
        Makes a lag controller to add gain at low frequency such that: 
            phase lag (|p|<|z|): 
            M - low frequency gain
            z - frequency where effect roughly ends
    '''
    Lag = tf([1, z], [1, z / M])

    return Lag

def get_control_lpf(p):
    '''
        generates a transfer function for a low-pass filter with a
        cutoff frequency of "p" 
    '''
    LPF = tf(p, [1, p])

    return LPF

# phase lead (|p|>|z|): increase PM (stability)
# low frequency gain = K*z/p
# high frequency gain = K
def get_control_lead(omega_lead, M):
    '''
        Generates a lead controller transfer function defined by:
            M - the amount of gain at high frequencies (corresponds to PM boost)
            omega_lead - frequency where max PM will be added
    '''
    Lead = tf([M, M*omega_lead / np.sqrt(M)], [1, omega_lead * np.sqrt(M)])

    return Lead


def get_control_notch(ws, M):
    '''
        Generates a notch filter defined by frequency of interest (ws) and gain M. 
    '''
    Notch = tf([1, 2 * np.sqrt(M) * ws, M * ws**2], [1, (M + 1) * ws, M * ws**2]);

    return Notch
