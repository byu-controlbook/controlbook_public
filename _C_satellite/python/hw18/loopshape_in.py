import sys
sys.path.append('..')  # add parent directory
import satelliteParam as P
sys.path.append('../hw10')  # add parent directory
import satelliteParamHW10 as P10
import matplotlib.pyplot as plt
from control import TransferFunction as tf
import control as cnt
import numpy as np 

P_in = tf([P.sigma, 1], [P.sigma*P.Js, (P.sigma*P.b+P.Js),
                         (P.b+P10.kd_th+P.sigma*P.k), P.k])
Plant = P_in

#PLOT = True
PLOT = False

if PLOT: plt.figure(3), plt.clf() #, plt.hold(True)
mag, phase, omega = cnt.bode(Plant, dB=True,
                             omega=np.logspace(-3, 5),
                             Plot=False)
if PLOT: 
    plt.subplot(2, 1, 1), plt.grid(True)
    plantMagPlot, = plt.semilogx(omega, mag, label='Plant')
    plt.subplot(2, 1, 2), plt.grid(True)
    plantPhasePlot, = plt.semilogx(omega, phase, label='Plant')

#########################################
#   Define Design Specifications
#########################################

#----------- general tracking specification --------
omega_r = 0.001  # track signals below this frequency
gamma_r = 10.0**(-40.0/20.0)  # tracking error below this value
w = np.logspace(np.log10(omega_r)-2, np.log10(omega_r))
if PLOT:
    plt.subplot(211)
    noisePlot, = plt.plot(w,
                          (20*np.log10(1.0/gamma_r))*np.ones(len(w)),
                          color='g',
                          label='tracking spec')

#----------- noise specification --------
omega_n = 20    # attenuate noise above this frequency
gamma_n = 10.0**(-40.0/20.0)    # attenuate noise by this amount
w = np.logspace(np.log10(omega_n), np.log10(omega_n)+2)
if PLOT:
    plt.subplot(211)
    noisePlot, = plt.plot(w, (20*np.log10(gamma_n))*np.ones(len(w)),
                          color='g', label='noise spec')


#########################################
#   Control Design
#########################################
Control = tf([1], [1])

#  -----proportional control: change cross over frequency----
kp = 30
Control = kp*Control
if PLOT:
    mag, phase, omega = cnt.bode(Plant*Control, dB=True, Plot=False)
    plt.subplot(211)
    openMagPlot, = plt.semilogx(omega, mag, color='r', label='OPEN')
    plt.subplot(212)
    openPhasePlot, = plt.semilogx(omega, phase, color='r',
                                  label='OPEN')
    # Calculate the phase and gain margin
    gm, pm, Wcg, Wcp = cnt.margin(Plant*Control)
    print("pm: ",pm," gm: ", gm," Wcp: ", Wcp, " Wcg: ", Wcg)

#  -----low pass filter: decrease gain at high frequency (noise)----
p = 10.0
LPF = tf([p], [1, p])
Control = Control*LPF
if PLOT:
    mag, phase, omega = cnt.bode(Plant*Control, dB=True, Plot=False)
    plt.subplot(211)
    openMagPlot, = plt.semilogx(omega, mag, color='r', label='OPEN')
    plt.subplot(212)
    openPhasePlot, = plt.semilogx(omega, phase, color='r',
                                  label='OPEN')
    # Calculate the phase and gain margin
    gm, pm, Wcg, Wcp = cnt.margin(Plant * Control)
    print("pm: ", pm, " gm: ", gm, " Wcp: ", Wcp, " Wcg: ", Wcg)

# #  ----  phase lead: increase PM (stability) ----
# w_max = 25 #location of maximum frequency bump (desired crossover)
# phi_max = 60*np.pi/180
# M = (1 + np.sin(phi_max))/(1 - np.sin(phi_max))  # lead ratio
# z = w_max/np.sqrt(M)
# p = w_max*np.sqrt(M)
# Lead = tf([1/z, 1], [1/p, 1])
# C = C*Lead
#
# # find gain to set crossover at w_max = 25 rad/s
# mag, phase, omega = cnt.bode(Plant*C, dB=False, omega=[w_max],
#                     Plot=False)
# K = tf([1/mag.item(0)], [1])
# C = K*C

############################################
#  Create Plots
############################################
# Open-loop transfer function
OPEN = Plant*Control
# Closed loop transfer function from R to Y
CLOSED_R_to_Y = (Plant*Control/(1.0+Plant*Control)).minreal()
# Closed loop transfer function from R to U
CLOSED_R_to_U = (Control/(1.0+Plant*Control)).minreal()

if PLOT:
    plt.figure(4), plt.clf()

    plt.subplot(311),  plt.grid(True)
    mag, phase, omega = cnt.bode(CLOSED_R_to_Y, dB=True, Plot=False)
    plt.semilogx(omega, mag, color='b')
    plt.title('Close Loop Bode Plot')

    plt.subplot(312), plt.grid(True)
    T = np.linspace(0, 2, 100)
    T, yout = cnt.step_response(CLOSED_R_to_Y, T)
    plt.plot(T, yout, color='b')
    plt.title('Close Loop Step Response')

    plt.subplot(313), plt.grid(True)
    T = np.linspace(0, 2, 100)
    T, yout = cnt.step_response(CLOSED_R_to_U, T)
    plt.plot(T, yout, color='b')
    plt.title('Control Effort for Step Response')

    # the pause causes the figure to be displayed during the
    # simulation Keeps the program closing until the user presses a
    # button.
    plt.pause(0.0001)  
    print('Press key to close')
    plt.waitforbuttonpress()
    plt.close()

##############################################
#  Convert Controller to State Space Equations
##############################################
C_ss = cnt.tf2ss(Control)  # convert to state space

#########################################################
#  Convert Controller to discrete transfer functions for
#  implementation
#########################################################
#bilinear: Tustin's approximation ("generalized bilinear
# transformation" with alpha=0.5) C_in_d = tf.sample(C, P.Ts,
# method='bilinear')

