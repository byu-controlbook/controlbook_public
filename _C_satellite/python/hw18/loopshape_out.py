import sys
sys.path.append('..')  # add parent directory
import satelliteParam as P
sys.path.append('../hw10')  # add parent directory
import satelliteParamHW10 as P10
import matplotlib.pyplot as plt
from control import TransferFunction as tf
import control as cnt
import numpy as np
import loopshape_in as L_in

P_out = tf([P.sigma*P.b, P.b+P.sigma*P.k, P.k],
           [P.sigma*P.Jp,
            P.Jp+P.b*P10.kd_phi+P.sigma*P.b,
            P.b+P.k*P10.kd_phi+P.sigma*P.k, P.k])

# construct plant as cascade of P_out and closed inner loop
Plant = tf.minreal(P_out* \
                   (L_in.P_in*L_in.Control/ \
                    (1+L_in.P_in*L_in.Control)))

PLOT = False
#PLOT = True

if PLOT: plt.figure(3), plt.clf()
mag, phase, omega = cnt.bode(Plant,
                             dB=True,
                             omega=np.logspace(-3, 5),
                             Plot=False)
if PLOT: 
    plt.subplot(2, 1, 1), plt.grid(True)
    plantMagPlot, = plt.semilogx(omega, mag, label='Plant')
    plt.subplot(2,1,2), plt.grid(True)
    plantPhasePlot, = plt.semilogx(omega, phase, label='Plant')

#########################################
#   Define Design Specifications
#########################################

#----------- input disturbance specification --------
omega_din = 10**(-2.0)  # reject input disturbances below this
                        # frequency
gamma_din = 0.1  # amount of input disturbance in output
w = np.logspace(np.log10(omega_din)-2, np.log10(omega_din)+2)
if PLOT:
    plt.subplot(211)
    noisePlot, = plt.plot(w,
                          20*np.log10(1.0/gamma_din)*np.ones(len(w)),
                          color='g', label='noise spec')

#----------- noise specification --------
omega_n = 10    # attenuate noise above this frequency
gamma_n = 10.0**(-80.0/20.0)    # attenuate noise by this amount
w = np.logspace(np.log10(omega_n), np.log10(omega_n)+2)
if PLOT:
    plt.subplot(211)
    noisePlot, = plt.plot(w,
                          20*np.log10(gamma_n)*np.ones(len(w)),
                          color='g', label='noise spec')

#########################################
#   Control Design
#########################################
Control = tf([1], [1])

#----------- integral control --------
k_I = 0.4  # frequency at which integral action ends
Integrator = tf([1, k_I], [1, 0])
Control = Control*Integrator
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

#----------- proportional control --------
kp = 0.5
Control = Control*kp
if PLOT:
    mag, phase, omega = cnt.bode(Plant*Control, dB=True, Plot=False)
    plt.subplot(211)
    openMagPlot, = plt.semilogx(omega, mag, color='r', label='OPEN')
    plt.subplot(212)
    openPhasePlot, = plt.semilogx(omega, phase,
                                  color='r', label='OPEN')
    # Calculate the phase and gain margin
    gm, pm, Wcg, Wcp = cnt.margin(Plant * Control)
    print("pm: ", pm, " gm: ", gm, " Wcp: ", Wcp, " Wcg: ", Wcg)

#----------- low pass filter (decrease gain at high freq - noise) --
p = 3.0
LPF = tf([p], [1, p])
Control = Control*LPF
if PLOT:
    mag, phase, omega = cnt.bode(Plant*Control, dB=True, Plot=False)
    plt.subplot(211)
    openMagPlot, = plt.semilogx(omega, mag, color='r', label='OPEN')
    plt.subplot(212)
    openPhasePlot, = plt.semilogx(omega, phase,
                                  color='r', label='OPEN')
    # Calculate the phase and gain margin
    gm, pm, Wcg, Wcp = cnt.margin(Plant * Control)
    print("pm: ", pm, " gm: ", gm, " Wcp: ", Wcp, " Wcg: ", Wcg)


############################################
#  Prefilter Design
############################################
F = tf([1], [1])

# low pass filter
p = 2.0
LPF = tf([p], [1, p])
F = F*LPF

############################################
#  Create Plots
############################################
# Open-loop transfer function
OPEN = Plant*Control
# Closed loop transfer function from R to Y
CLOSED_R_to_Y = (F*Plant*Control/(1.0+Plant*Control)).minreal()
# Closed loop transfer function from R to U
CLOSED_R_to_U = (F*Control/(1.0+Plant*Control)).minreal()

if PLOT:
    plt.figure(4), plt.clf()

    plt.subplot(311), plt.grid(True)
    mag, phase, omega = cnt.bode(CLOSED_R_to_Y,
                                 dB=True, Plot=False)
    plt.semilogx(omega, mag, color='b')
    mag, phase, omega = cnt.bode(CLOSED_R_to_Y*F,
                                 dB=True, Plot=False)
    plt.semilogx(omega, mag, color='r')
    plt.title('Close Loop Bode Plot')

    plt.subplot(312), plt.grid(True)
    T, yout = cnt.step_response(CLOSED_R_to_Y)
    plt.plot(T, yout, color='b')
    T, yout = cnt.step_response(CLOSED_R_to_Y*F)
    plt.plot(T, yout, color='r')
    plt.title('Close Loop Step Response')

    plt.subplot(313), plt.grid(True)
    T, yout = cnt.step_response(CLOSED_R_to_U)
    plt.plot(T, yout, color='b')
    T, yout = cnt.step_response(CLOSED_R_to_U*F)
    plt.plot(T, yout, color='r')
    plt.title('Control Effort for Step Response')

    # the pause causes the figure to be displayed during the
    # simulation Keeps the program from closing until the user 
    # presses a button.
    plt.pause(0.0001)  
    print('Press key to close')
    plt.waitforbuttonpress()
    plt.close()


##############################################
#  Convert Controller to State Space Equations
##############################################
C_ss = cnt.tf2ss(Control)
F_ss = cnt.tf2ss(F)


