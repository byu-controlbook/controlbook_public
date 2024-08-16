#%%
from hw03_pendulum_solving_for_state_variable_form import *

#%%
############################################################
### Defining vectors for x_dot, x, and u, then taking partial derivatives
############################################################


# defining derivative of states, states, and inputs symbolically
### for this first one, keep in mind that zdd_eom is actually a full
### row of equations, while zdd is just the symbolic variable itself. 
state_variable_form = Matrix([[zdd_eom], [zd], [thetadd_eom], [thetad]])
states = Matrix([[zd], [z], [thetad], [theta]])
inputs = Matrix([[F]])


# finding the jacobian with respect to states (A) and inputs (B)
A = state_variable_form.jacobian(states)
B = state_variable_form.jacobian(inputs)

# sub in values for equilibrium points (x_e, u_e) or (x_0, u_0)
A_lin = simplify(A.subs([(thetad,0.), (theta,0.), (zd,0.), (z,0.), (F, 0.)]))
B_lin = simplify(B.subs([(thetad,0.), (theta,0.), (zd,0.), (z,0.), (F, 0.)]))

display(Math(vlatex(A_lin)))
display(Math(vlatex(B_lin)))

# %%