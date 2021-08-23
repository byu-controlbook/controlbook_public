from control import *
import matplotlib.pyplot as plt
import numpy as np

Plant = tf([1],[1,1]) #transfer function for plant

# initialize frequency response for plant only
plt.figure()
plt.grid(which='both')

###################################
# input disturbance specification #
###################################
# reject dist below this frequency
omega_din = 10**-1
omega = np.logspace(-4, np.log10(omega_din))
Pmag, phase, _ = bode(Plant, omega, Plot=False)

# amount of input disturbance in output
gamma_din = 0.1
mag_din = 20*np.log10(1/gamma_din)*np.ones(omega.size)+\
		  20*np.log10(Pmag)
plt.semilogx(omega, mag_din, color=[0, 1, 0], ls=':',
             label='$d_{in}$ constraint')

#######################
# noise specification #
#######################
# attenuate noise above this frequency
omega_n = 10^1

#attenuate noise by this amount
gamma_n = .1
omega = np.logspace(np.log10(omega_n),2+np.log10(omega_n))
plt.semilogx(omega, 20*np.log10(gamma_n)*np.ones(omega.size),
			 color=[0, 1, 0], ls='--', label='noise constraint')

#######################
# steady state errror #
#######################
#steady state tracking of ramp
gamma_1 = .03
omega = np.logspace(-4, 0)
plt.semilogx(omega, 20*np.log10(1/gamma_1)-20*np.log10(omega),
			 color=[0, 1, 0], ls='-.',
             label='tracking error constraint')

# open-loop freq response for plant only
omega = np.logspace(-4, 3)
Pmag, phase, omega = bode(Plant,omega, Plot=False)
Pmag_dB = 20*np.log10(Pmag)
plt.semilogx(omega, Pmag_dB, label='$P(s)$')

##################
# control design #
##################
C = 1.0

#proportional control
C_kp = 2.0

#integral control
k_I = 0.4
C_PI = tf([1, k_I], [1, 0])

#phase lag
z = 0.8
M = 50.0
C_lag = tf([1, z], [1, z/M])

#low-pass filter
p = 5
C_lpf = tf([p], [1, p])

#total filter:
# for figure 18-7
C1 = C*C_kp*C_PI

# for figure 18-8
C2 = C*C_kp*C_PI*C_lag

# for figure 18-9
C3 = C*C_kp*C_PI*C_lag*C_lpf

# open-loop freq response of the controller + plant
omega = np.logspace(-4, 3)
PC_mag, phase, _ = bode(Plant*C3, omega, Plot=False)
plt.semilogx(omega, 20*np.log10(PC_mag),
			 label='$P(s)C_{k_p}C_{PI}C_{lag}C_{lpf}= P(s)C(s)$')

plt.title('Bode Diagram')
plt.xlabel('Frequency (rad/s)')
plt.ylabel('Magnitude (dB)')
plt.legend()

plt.show()