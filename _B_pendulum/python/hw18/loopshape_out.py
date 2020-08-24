import sys
sys.path.append('..')  # add parent directory
import pendulumParam as P
import matplotlib.pyplot as plt
from control import TransferFunction as tf
import control as cnt
import numpy as np
import loopshape_in as L_in

P_out = tf([P.ell/2,0,-P.g], [1, 0, 0])

# construct plant as cascade of P_out and closed inner loop
Plant = tf.minreal(P_out* \
                   (L_in.P_in*L_in.C/(1+L_in.P_in*L_in.C)))

PLOT = False
#PLOT = True

if PLOT: plt.figure(3), plt.clf()
mag, phase, omega = cnt.bode(Plant, dB=True,
                             omega=np.logspace(-3, 5),
                             Plot=False)
if PLOT: 
    plt.subplot(2, 1, 1), plt.grid(True)
    plantMagPlot, = plt.semilogx(omega, mag,
                                 label='Plant')
    plt.subplot(2,1,2), plt.grid(True)
    plantPhasePlot, = plt.semilogx(omega, phase,
                                   label='Plant')

#########################################
#   Define Design Specifications
#########################################

#----------- general tracking specification --------
omega_r = 0.0032    # track signals below this frequency
gamma_r = 0.01    # tracking error below this value
w = np.logspace(np.log10(omega_r) - 2, np.log10(omega_r))
if PLOT:
    plt.subplot(2, 1, 1)
    trackPlot, = plt.plot(w,
                          20*np.log10(1/gamma_r)*np.ones(len(w)),
                          color='g', label='tracking spec')

#----------- noise specification --------
omega_n = 1000    # attenuate noise above this frequency
gamma_n = 0.0001    # attenuate noise by this amount
w = np.logspace(np.log10(omega_n), np.log10(omega_n)+2)
if PLOT:
    plt.subplot(2, 1, 1)
    noisePlot, = plt.plot(w,
                          20*np.log10(gamma_n)*np.ones(len(w)),
                          color='g', label='noise spec')


#########################################
#   Control Design
#########################################

C = tf([1], [1])

#  phase lead: increase PM (stability)
# At desired crossover frequency, PM = -3
# Add 70 deg of PM with lead

#location of maximum frequency bump (desired crossover)
w_max = 1.1  
phi_max = 70*np.pi/180

# lead ratio
M = (1 + np.sin(phi_max))/(1 - np.sin(phi_max))  
z = w_max/np.sqrt(M)
p = w_max*np.sqrt(M)
Lead = tf([1/z, 1], [1/p, 1])
C = C*Lead

# find gain to set crossover at w_max = 1.1 rad/s
mag, phase, omega = cnt.bode(Plant*C, dB=False,
                             omega=[w_max], Plot=False)
K = tf([1/mag.item(0)], [1])
C = K*C

# Tracking constraint not satisfied -- add lag compensation to
# boost low-frequency gain

# Find gain increase needed at omega_r
mag, phase, omega = cnt.bode(Plant*C, dB=True, omega=[omega_r],
                             Plot=False)
gain_increase_needed = 1/gamma_r/mag.item(0)

# minimum gain increase at low frequencies is 4.8 let lag ratio
# be 8.
M = 8
p = omega_r # set pole at omega_r
z = M*p # set zero at M*omega_r
Lag = tf([M/z, M], [1/p, 1])
C = C*Lag

# Noise attenuation constraint not quite satisfied can be
# satisfied by reducing gain at 400 rad/s by a factor of 2 Use
# a low-pass filter
m = 0.5 #attenuation factor
a = m*omega_n*np.sqrt(1/(1-m**2))
lpf = tf([a],[1,a])
C = lpf*C

############################################
#  Prefilter Design
############################################
F = tf([1], [1])

# low pass filter
p = 2
LPF = tf([p], [1, p])
F = F*LPF



############################################
#  Create Plots
############################################
# Open-loop transfer function
OPEN = Plant*C
# Closed loop transfer function from R to Y
CLOSED_R_to_Y = (Plant*C/(1.0+Plant*C))
# Closed loop transfer function from R to U
CLOSED_R_to_U = (C/(1.0+Plant*C))

mag, phase, omega = cnt.bode(OPEN, dB=True, Plot=False)
if PLOT: 
    openMagPlot, = plt.semilogx(omega, mag, color='r',
                                label='OPEN')
    plt.subplot(212)
    openPhasePlot, = plt.semilogx(omega, phase, color='r',
                                  label='OPEN')
if PLOT:
    plt.subplot(211)
    plt.legend(handles=[plantMagPlot, trackPlot, noisePlot,
                        openMagPlot])
    plt.pause(0.0001)  # the pause causes the figure to be
                       # displayed during the simulation


if PLOT:
    plt.figure(4), plt.clf()

    plt.subplot(311), plt.grid(True)
    mag, phase, omega = cnt.bode(CLOSED_R_to_Y, dB=True,
                                 Plot=False)
    plt.semilogx(omega, mag, color='b')
    mag, phase, omega = cnt.bode(CLOSED_R_to_Y*F, dB=True,
                                 Plot=False)
    plt.semilogx(omega, mag, color='r')
    plt.title('Close Loop Bode Plot')

    plt.subplot(312), plt.grid(True)
    T = np.linspace(0,7,100)
    T, yout = cnt.step_response(CLOSED_R_to_Y, T)
    plt.plot(T, yout, color='b')
    T, yout = cnt.step_response(CLOSED_R_to_Y*F, T)
    plt.plot(T, yout, color='r')
    plt.title('Close Loop Step Response')

    plt.subplot(313), plt.grid(True)
    T = np.linspace(0, 2, 100)
    T, yout = cnt.step_response(CLOSED_R_to_U, T)
    plt.plot(T, yout, color='b')
    T, yout = cnt.step_response(CLOSED_R_to_U*F, T)
    plt.plot(T, yout, color='r')
    plt.title('Control Effort for Step Response')

if PLOT:
    # Keeps the program from closing until the user presses a
    # button.
    print('Press key to close')
    plt.waitforbuttonpress()
    plt.close()


##############################################
#  Convert Controller to State Space Equations
##############################################
C_ss = cnt.tf2ss(C)
F_ss = cnt.tf2ss(F)


##########################################################
#  Convert Controller to discrete transfer functions for
#  implementation
##########################################################
#bilinear: Tustin's approximation ("generalized bilinear
#transformation" with alpha=0.5)
C_out_d = tf.sample(C, P.Ts, method='bilinear') 

#bilinear: Tustin's approximation ("generalized bilinear
#transformation" with alpha=0.5)
F_d = tf.sample(F, P.Ts, method='bilinear') 
