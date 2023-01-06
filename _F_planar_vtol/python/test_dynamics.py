#%%[markdown]

# The cell below can be used to double check the output of your dynamics file for hw 03. 

# The output for the VTOL dynamics with the given input should be the following:
#
# $\left[ \begin{array}{c} -6.09755153e-06 \\ 5.50949070e-03 \\  6.09756098e-03 \\ -2.43860844e-03 \\ 1.10189442e+00 \\ 1.21951220e+00 \end{array} \right]$
#%%

from VTOLDynamics import VTOLDynamics
import numpy as np
VTOL = VTOLDynamics()

# this numbers below for input are just for testing purposes
y = VTOL.update(np.array([[100.0], [80.0]]))
print("Your state is:\n")
print(VTOL.state)