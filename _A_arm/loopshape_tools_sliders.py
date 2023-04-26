import matplotlib.pyplot as plt
import warnings
warnings.filterwarnings("ignore")
from matplotlib.widgets import Slider
from numpy import log10, ones, size, divide, logspace, square
from control import bode, tf, margin
import numpy as np




def spec_noise(gamma_n, omega_n, dB_flag=True):
    '''
        Add noise specification, defined by gamma_n for remaining magnitude 
        of error, and omega_n for the frequency at which it needs to happen. 
    '''
    # w = logspace(log10(omega_n), 2 + log10(omega_n))
    fig = plt.gcf()
    w = np.linspace(omega_n, omega_n + 1e5)
    fig.axes[0].plot(w, gamma_n*np.ones(np.size(w)), 'g')
    # if dB_flag == False:
    #     fig.axes[0].plot(w,gamma_n*ones(size(w)),'g')
    #     # fig.axes[0].scatter(omega_n, gamma_n, facecolors='none', 
    #     #                     edgecolors='red', label='noise spec')
    # else:
    #     fig.axes[0].plot(w,20*log10(gamma_n)*ones(size(w)),'g')
    #     # fig.axes[0].scatter(omega_n, 20.0* log10(gamma_n), facecolors='none', 
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


def lead(omega_lead, M):
    '''
        Generates a lead controller transfer function defined by:
            M - the amount of gain at high frequencies (corresponds to PM boost)
            omega_lead - frequency where max PM will be added
    '''
    return tf([np.sqrt(M)/omega_lead,1], [1/(omega_lead*np.sqrt(M)), 1])


def notch(ws, M):
    '''
        Generates a notch filter defined by frequency of interest (ws) and gain M. 
    '''
    return tf([1, 2 * np.sqrt(M) * ws, M * ws**2], [1, (M + 1) * ws, M * ws**2])

class LS_design:
    def __init__(self,
           plant,                #initial controler
           ctr1,               #physical system
           #Specifications      #inputs
           noise = 0,           #[gamma_n, omega_n]
           disturbance = 0,     #[gamma_d, omega_d]
           track_ref = 0,       #[gamma_r, omega_r]
           track_step = 0,      #gamma_r
           track_ramp = 0,      #gamma_r
           track_parabola = 0,  #gamma_r
           
           #Controlers          
           proportional = 0,    #raise/lower whole thing
           integral = 0,        #Raise only top part
           lead = 0,            #add phase @crossover
           lag = 0,             #increase gain @ low frequencies
           lpf = 0,             #low pass filter
           notch = 0            #Remove resonant frequencies
           ):
        
        #
        '''
            This class is intended to aid with using loopshaping to modify a controller.
            An initial controller and plant are required.
            Any filters or specifications that are not specified are not used.
            After completing the design phase, the transfer functions for the controller
            and prefilter are available to the code by calling the object.ctr2 or object.F
            respectively. Gains are available using the same method.
        '''
        
        #Save input transfer functions
        self.ctr1 = ctr1
        self.plant = plant

        #Store specifications
        self.noise = noise
        self.disturbance = disturbance
        self.track_ref = track_ref
        self.track_step = track_step
        self.track_ramp = track_ramp
        self.track_parabola = track_parabola
    
        #Controler parameters:
        self.kp = 0.
        self.ki = 0.
        self.z_lag = 0.
        self.M_lag = 0.
        self.p_lpf = 0.
        self.w_lead = 0.
        self.M_lead = 0.
        self.ws_notch = 0.
        self.M_notch = 0.

        #################
        # Set up sliders:
        #################

        fig2 = plt.figure(2) #Want these on the second figure
        pos = .9 #location of the slider This will change dynamically
        #Margins on either side. Wide enough to show relevant information
        right_margin = 1-.3
        left_margin = .2
        height = 0.03
        
        ########################
        # Build specific sliders
        ########################
        # Some values are log scaled to make it easier to match to bode plot
        # Comments are only on proportional and lead, but apply to all
        if proportional != 0:
            self.kp = proportional #Store the intial value for the gain
            ax_prop = fig2.add_axes([left_margin, pos, right_margin, height]) #create the axis for the slider
            pos -= .1 # Increment the position for the next slider
            self.slide_prop = Slider(ax_prop, "Kp = 10**", -3, 3, np.log10(self.kp)) #Create the next slider with initial values
            self.slide_prop.on_changed(self.slider_update) # Link the slider to the update function
        
        if integral != 0:
            self.ki = integral
            ax_int = fig2.add_axes([left_margin, pos, right_margin, height])
            pos -=.1
            self.slide_int = Slider(ax_int, "Ki = 10**", -2, 2, np.log10(self.ki))
            self.slide_int.on_changed(self.slider_update)
        
        if lead != 0:
            #There are 2 parameters for the lead filter, so 2 sliders are generated. The input is a list of 2 floats
            self.w_lead = lead[0] 
            self.M_lead = lead[1] 
            ax_w_lead = fig2.add_axes([left_margin, pos, right_margin, height])
            pos -=.1
            ax_M_lead = fig2.add_axes([left_margin, pos, right_margin, height])
            pos -=.1
            self.slide_w_lead = Slider(ax_w_lead, "w_lead = 10**", -1,2, np.log10(self.w_lead))
            self.slide_M_lead = Slider(ax_M_lead, "m_lead = 10**", -2, 10, np.log10(self.M_lead))
            self.slide_w_lead.on_changed(self.slider_update)
            self.slide_M_lead.on_changed(self.slider_update)
        
        if lag != 0:
            self.z_lag = lag[0]
            self.M_lag = lag[1]
            ax_z_lag = fig2.add_axes([left_margin, pos, right_margin, height])
            pos -=.1
            ax_M_lag = fig2.add_axes([left_margin, pos, right_margin, height])
            pos-=.1
            self.slide_z_lag = Slider(ax_z_lag, "z_lag = ", 0, 180, self.z_lag)
            self.slide_M_lag = Slider(ax_M_lag, "M_lag = 10**", -2, 10, np.log10(self.M_lag))
            self.slide_z_lag.on_changed(self.slider_update)
            self.slide_M_lag.on_changed(self.slider_update)
        
        if lpf != 0:
            self.p_lpf = lpf
            ax_p_lpf = fig2.add_axes([left_margin, pos, right_margin, height])
            pos-=.1
            self.slide_p_lpf = Slider(ax_p_lpf, "LPF = 10**", -5, 2, np.log10(self.p_lpf))
            self.slide_p_lpf.on_changed(self.slider_update)
                
        if notch != 0:
            self.ws_notch = notch[0]
            self.M_notch = notch[1]
            ax_ws_notch = fig2.add_axes([left_margin, pos, right_margin, height])
            pos-=.1
            ax_M_notch = fig2.add_axes([left_margin, pos, right_margin, height])
            self.slide_ws_notch = Slider(ax_ws_notch, "ws_notch = 10**", -1, 2, np.log10(self.ws_notch))
            self.slide_M_notch = Slider(ax_M_notch, "M notch = 10**", -1, 2, np.log10(self.M_notch))
            self.slide_ws_notch.on_changed(self.slider_update)
            self.slide_M_notch.on_changed(self.slider_update)
        
        plt.figure(1)
        #There needs to be an initial bode plot so that the axes exist (this is just a bug fix)
        bode(plant)
        #Run the update program to get all the plots shown before using them
        self.slider_update(0)
        #plt.show will pause the program until the figures are closed.
        plt.show()
        
    def slider_update(self,val):
        #Plot everything on figure 1. Sliders live on figure 2
        fig = plt.figure(1)
        #Clear off old charts
        fig.axes[0].cla()
        fig.axes[1].cla()
        #Make a bode plot of the plant for reference
        # bode(self.plant)

        #Plot specs
        if self.noise !=0:
            spec_noise(self.noise[0], self.noise[1])
        if self.disturbance !=0:
            spec_disturbance(self.disturbance[0], self.disturbance[1], self.plant)
        if self.track_ref !=0:
            spec_track_ref(self.track_ref[0],self.track_ref[1])
        if self.track_step != 0:
            spec_track_step(self.track_step)
        if self.track_ramp !=0:
            spec_track_ramp(self.track_ramp)
        if self.track_parabola !=0:
            spec_track_parabola(self.track_parabola)



        #Create augmented controller:
        #Pull the slider data and update the gains
        ctr = self.ctr1
        self.F = tf([1],[1])
        if self.kp !=0:
            self.kp = 10**self.slide_prop.val
            ctr*= proportional(self.kp)
        if self.ki != 0:
            self.ki = 10**self.slide_int.val
            ctr*= integral(self.ki)
        if self.M_lead !=0:
            self.M_lead = 10**self.slide_M_lead.val
            self.w_lead = 10**self.slide_w_lead.val
            ctr*=lead(self.w_lead, self.M_lead)
        if self.M_lag !=0:
            self.z_lag = self.slide_z_lag.val
            self.M_lag = 10**self.slide_M_lag.val
            ctr*=lag(self.z_lag, self.M_lag)
        if self.p_lpf !=0:
            self.p_lpf = 10**self.slide_p_lpf.val
            self.F =lpf(self.p_lpf)
        if self.M_notch !=0:
            self.ws_notch = 10**self.slide_ws_notch.val
            self.M_notch = 10**self.slide_M_notch.val
            self.F*=notch(self.ws_notch, self.M_notch)
        
        self.ctr2 = ctr
        


        #Draw the new Bode Plot
        bode(ctr*self.plant*self.F,
              label='C(s)P(s) - Open-loop')
        bode(ctr*self.plant*self.F/(1+ctr*self.plant*self.F),
             label=r'$\frac{P(s)C(s)}{1+P(s)C(s)}$ - Closed-loop')
        fig.axes[1].legend()
        fig.canvas.draw()
        #Find the gain margin
        gm, pm, Wcg, Wcp = margin(self.plant*self.ctr2)
        print("Phase Margin \t", pm,
              "\nCrossover Frequency: \t", Wcg,
              "\nlog 10 of Crossover Frequency:\t", np.log10(Wcg))
        





        

if __name__ == '__main__':

    plant = tf([1],[1,.5,2])
    ctr = tf([3],[1,2,3])
    LS_design(plant, ctr, 
        
        disturbance=[.01, 1], 
        track_ref=[.01, 3],
        track_step = .01,
        track_ramp = .01,
        track_parabola= .01,

        proportional=1,
        integral=1,
        lead = [1, 1e2],
        lag = [1,1e2],
        lpf = 2,
        notch = [1, 10]

        )
