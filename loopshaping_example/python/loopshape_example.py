import param as P
import matplotlib.pyplot as plt
from control import TransferFunction as tf
import control as cnt
import numpy as np
import loopshape_tools as ls

#########################################
# Specify plant transfer functions
Plant = tf([1.0], [1.0, 1.0])

#########################################
#   Control Design
C = tf(1, 1) \
         * ls.integral(ki=0.3) \
         * ls.proportional(kp=3.0) \
         * ls.lag(z=0.01, M=40.0) \
         * ls.lpf(p=3.0) \
        #* ls.lead(w_L=3.0, M=10.0)

###########################################################
# add a prefilter to eliminate the overshoot
###########################################################
F = tf(1, 1) \
    #* lt.get_control_lpf(3.0)

###########################################################
# Extracting coefficients for controller and prefilter
###########################################################
C_num = np.asarray(C.num[0])
C_den = np.asarray(C.den[0])
F_num = np.asarray(F.num[0])
F_den = np.asarray(F.den[0])

if __name__=="__main__":

    # calculate bode plot and gain and phase margin
    mag, phase, omega = cnt.bode(Plant, dB=True, omega=np.logspace(-3, 5), Plot=False)
    gm, pm, Wcg, Wcp = cnt.margin(Plant)
    print(" pm: ", pm, " Wcp: ", Wcp, "gm: ", gm, " Wcg: ", Wcg)

    plt.figure(3), plt.clf()
    plt.subplot(211), plt.grid(True)
    plantMagPlot, = plt.semilogx(omega, 20*np.log10(mag), label='Plant')
    plt.subplot(212), plt.grid(True)
    plantPhasePlot, = plt.semilogx(omega, phase, label='Plant')

    #########################################
    #   Define Design Specifications
    ls.spec_track_ramp(gamma_r=0.03)
    ls.spec_noise(gamma_n=0.1, omega_n=10)
    ls.spec_disturbance(gamma_d=0.1, omega_d=0.1, system=Plant)
    #ls.spec_track_ref(gamma_r=0.03, omega_r=0.1)

    mag, phase, omega = cnt.bode(Plant*C, dB=True, omega=np.logspace(-5, 5), Plot=False)
    gm, pm, Wcg, Wcp = cnt.margin(Plant*C)
    print(" pm: ", pm, " Wcp: ", Wcp, "gm: ", gm, " Wcg: ", Wcg)
    plt.subplot(211),
    plantMagPlot, = plt.semilogx(omega, 20*np.log10(mag), label='PC')
    plt.subplot(212),
    plantPhasePlot, = plt.semilogx(omega, 180/3.14*phase, label='PC')

    # Closed loop transfer function from R to Y - no prefilter
    CLOSED_R_to_Y = (Plant*C/(1.0+Plant*C))
    # Closed loop transfer function from R to Y - with prefilter
    CLOSED_R_to_Y_with_F = (F*Plant*C/(1.0+Plant*C))
    # Closed loop transfer function from R to U
    CLOSED_R_to_U = (C/(1.0+Plant*C))

    plt.figure(4), plt.clf()
    plt.subplot(311),  plt.grid(True)
    mag, phase, omega = cnt.bode(CLOSED_R_to_Y, dB=True, Plot=False)
    plt.semilogx(omega, mag, color='b')
    mag, phase, omega = cnt.bode(CLOSED_R_to_Y_with_F, dB=True, Plot=False)
    plt.semilogx(omega, mag, color='g')
    plt.title('Close Loop Bode Plot')

    plt.subplot(312), plt.grid(True)
    T = np.linspace(0, 2, 100)
    T, yout = cnt.step_response(CLOSED_R_to_Y, T)
    plt.plot(T, yout, color='b')
    plt.ylabel('Step Response')

    plt.subplot(313), plt.grid(True)
    T = np.linspace(0, 2, 100)
    T, yout = cnt.step_response(CLOSED_R_to_U, T)
    plt.plot(T, yout, color='b')
    plt.ylabel('Control Effort')

    # Keeps the program from closing until the user presses a button.
    plt.pause(0.0001)  # not sure why this is needed for both figures to display
    print('Press key to close')
    plt.waitforbuttonpress()
    plt.close()


