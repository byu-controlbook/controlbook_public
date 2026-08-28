# %%
################################################################################
# This file is meant to be run interactively with VSCode's Jupyter extension.
# It works as a regular Python script as well, but the printed results will not
# display as nicely.
################################################################################
# %%
import sympy as sp
from sympy.physics.vector import dynamicsymbols  # for time-varying symbols

# %%
# These are for pretty printing in Jupyter environments (not terminals)
from sympy.physics.vector.printing import vlatex
from IPython.display import Math, display


def printsym(expr):
    """
    Renders latex from sympy expression when in a Jupyter-like environment. Does
    not print anything useful when running in a terminal. When in a terminal, use
    sp.pretty_print() instead.
    """
    display(Math(vlatex(expr)))


# %%
# These functions are defined in our helper functions file in the public repository
from EL_helper_functions import rotx, roty, rotz, calc_omega, find_coeffs

# Defining necessary symbols and variables to use in the calculations
t, ell_1, ell_2, ell_3x, ell_3y, ell_3z, m1, m2, m3 = sp.symbols(
    "t, ell_1, ell_2, ell_3x, ell_3y, ell_3z, m_1, m_2, m_3"
)

# We have to tell SymPy that these are functions of time
theta = dynamicsymbols("theta")
psi = dynamicsymbols("psi")
phi = dynamicsymbols("phi")

q = sp.Matrix([[phi], [theta], [psi]])
q_dot = q.diff(t)


# %%
# TODO: define the position of each mass in body frame, then rotate
# it into the world or inertial frame.

# p1_in_b =
# p1_in_w =

# p2_in_2 =
# p2_in_w =

# p3_in_1 =
# p3_in_w =


# %%
# TODO: take the time derivative of the position vectors to get the linear velocity,
# then use the "find_coeffs" function to calculate the "V_i" matrices

# v1_in_w =
# V1 =

# v2_in_w =
# V2 =

# v3_in_w =
# V3 =


# %%
# TODO: use the "rotx", "roty", and "rotz" functions to calculate the rotation matrices
# for each rigid body

# R1 =  #rotation to body 1
# R2 =  #rotation to body 2
# R3 =  #rotation to body 3


# %%
# TODO: use the "calc_omega" function with the rotation matrices to calculate the
# angular velocity of each rigid body

# omega_1 =
# omega_2 =
# omega_3 =


# %%
# TODO: use the "find_coeffs" function to calculate the "W_i" matrices

# W1 =
# W2 =
# W3 =


# %%
# TODO: define symbols for the diagonal inertia terms then define the inertia
# tensors for each rigid body

# J1 = sp.zeros(3,3)

# J2 = sp.zeros(3,3)

# J3 = sp.zeros(3,3)


# %%
# TODO: calculate M using the masses and the V, W, R, and J matrices

M = sp.zeros(3, 3)
# M = M +


# %%
# Simplify the result
M = sp.trigsimp(M)

# %% [markdown]
# # Display the result of M
# %%
# Print longer terms separately (matches lab manual more closely)
M22 = M[1, 1]
M23 = M[1, 2]
M33 = M[2, 2]

# The M33 term can be simplified a bit more, but it doesn't quite match the lab manual:
# M33 = sp.collect(M33, M33.free_symbols)
# M33 = sp.trigsimp(M33)

long_terms = {
    M[1, 1]: sp.Symbol("M_22"),
    M[1, 2]: sp.Symbol("M_23"),
    M[2, 2]: sp.Symbol("M_33"),
}
printsym(M.subs(long_terms))
print("M22:")
printsym(M22)
print("M23:")
printsym(M23)
print("M33:")
printsym(M33)

# %%
