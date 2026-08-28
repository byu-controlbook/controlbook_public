# %%
################################################################################
# This file is meant to be run interactively with VSCode's Jupyter extension.
# It works as a regular Python script as well, but the printed results will not
# display as nicely.
################################################################################

# %% [markdown]
# # Step 1: Calculate Kinetic Energy and Potential Energy

# %%
# It is generally not recommended to import * (everything) from any module or
# package, but in this case we are essentially extending the previous lab file...
# they would normally be in the same file, but we are breaking them up to have
# one file per lab assignment.
from h2_generate_KE import *

# %%
# TODO:  define symbols needed for potential energy and the RHS (tau - B@q_dot)
# of the equations of motion


# TODO:  calculate the kinetic energy using the definition with the mass matrix "M"
# and "q_dot"
K =
K = K[0, 0]  # make K a scalar instead of a 1x1 matrix

# %%
# TODO: now calculate the potential energy "P" and make sure it is also a scalar
P =

printsym(P)


# %% [markdown]
# # Step 2: Calculate C, dP/dq, and tau
# ### Generalized Forces (tau):
#%%
# TODO: calculate and define tau
tau =

printsym(tau)


# %% [markdown]
# ### Coriolis Term (C):
# The lab manual does not show what C is directly, but rather the intermediate
# terms of Mdot, dM/dphi, dM/dtheta, and dM/dpsi.
# %%
# TODO: calculate Mdot and verify with lab manual
Mdot = # TODO

# Print longer terms separately like lab manual
print_subs = {
    Mdot[1, 1]: sp.Symbol("Mdot_22"),
    Mdot[2, 2]: sp.Symbol("Mdot_33"),
    Mdot[1, 2]: sp.Symbol("Mdot_23"),
}
printsym(Mdot.subs(print_subs))

# Note: using Python 3.13.7 and SymPy 1.14.0 with these simplifications matched
# the lab manual results pretty well, but other versions may look different.
# You can try other combinations of simplifying functions if these do not match
# the manual for you, but it may be worth moving on if you can't get it to
# match perfectly (finishing this file and then running the test_dynamics.py
# file to do numerical comparisons and see if your tests pass).

Mdot22 = Mdot[1, 1]
Mdot22 = sp.collect(Mdot22, 2 * sp.sin(phi) * sp.cos(phi) * q_dot[0])
printsym(Mdot22)

Mdot23 = Mdot[1, 2]
Mdot23 = sp.trigsimp(sp.factor(Mdot23))
# Mdot23 = sp.simplify(Mdot23)
printsym(Mdot23)

Mdot33 = Mdot[2, 2]
Mdot33 = sp.factor(Mdot33)
Mdot33 = sp.collect(Mdot33, [q_dot[0], q_dot[1] * sp.sin(theta)])
printsym(Mdot33)

# %%
# TODO: calculate the partial derivatives of M with respect to each generalized
# coordinate and verify with lab manual.

# NOTE: Same guidelines apply as Mdot above. Do what you can to simplify and
# match the lab manual, but if you can't get it to match, it may be worth
# moving on and using numerical comparisons in the test_dynamics.py file.

# %%
dM_dphi = # TODO
dM_dphi = sp.simplify(dM_dphi)
printsym(dM_dphi)

# %%
dM_dtheta = # TODO
# substitute N33 just for printing to match the lab manual
printsym(dM_dtheta.subs({dM_dtheta[2, 2]: sp.Symbol("N_33")}))
N33 = sp.collect(dM_dtheta[2, 2], 2 * sp.sin(theta) * sp.cos(theta))
printsym(N33)

# %%
dM_dpsi = # TODO
printsym(dM_dpsi)

# %%
# TODO: calculate C
C =

# Print if you want, but the result is not shown in the lab manual to compare
# printsym(C)

# %% [markdown]
# ### Partial Derivative of Potential Energy (dP/dq):
# %%
# TODO: calculate dP/dq
dP_dq =

# Print if you want, but the result is not shown in the lab manual to compare
# printsym(dP_dq)


# %% [markdown]
# # Step 3: Save functions for M, C, dP/dq, and tau each to a file
# %%
# TODO: now you can either run the code below to generate functions to calculate
# M, C, tau, and dP_dq using lambdify (where you can import the generated
# functions into your dynamics class), or you can hand-code the results you have
# found above into the hummingbirdDynamics.py file.

# Substitute out dynamic variables for prettier function generation
dynamic_subs = {
    phi: sp.Symbol("phi"),
    theta: sp.Symbol("theta"),
    psi: sp.Symbol("psi"),
    q_dot[0]: sp.Symbol("phidot"),
    q_dot[1]: sp.Symbol("thetadot"),
    q_dot[2]: sp.Symbol("psidot"),
}
M = M.subs(dynamic_subs)
C = C.subs(dynamic_subs)
dP_dq = dP_dq.subs(dynamic_subs)
tau = tau.subs(dynamic_subs)

# Define state and input vectors for function generation
state = sp.Matrix([phi, theta, psi, q_dot[0], q_dot[1], q_dot[2]]).subs(dynamic_subs)
u = sp.Matrix([f_l, f_r])

# Create a list of all paramters needed for the functions
# TODO: you may need to modify this to match the variable names you created above
params = list(
    [m1, m2, m3, J1x, J1y, J1z, J2x, J2y, J2z, J3x, J3y, J3z]
    + [ell_1, ell_2, ell_3x, ell_3y, ell_3z, ell_T, d, g]
)

# Generate functions using lambdify (*params unpacks the list into individual arguments)
calc_M = sp.lambdify([state, *params], M, modules="numpy")
calc_C = sp.lambdify([state, *params], C, modules="numpy")
calc_dP_dq = sp.lambdify([state, *params], dP_dq, modules="numpy")
calc_tau = sp.lambdify([state, u, *params], tau, modules="numpy")


# %% Save functions for the EOM matrices

# Method 1: using dill (blackbox method, saves binary files that are not human readable)

# import dill

# dill.settings["recurse"] = True
# dill.dump(calc_M, open("hb_M_func.pkl", "wb"))
# dill.dump(calc_C, open("hb_C_func.pkl", "wb"))
# dill.dump(calc_dP_dq, open("hb_dP_dq_func.pkl", "wb"))
# dill.dump(calc_tau, open("hb_tau_func.pkl", "wb"))

## This is how you could load the functions back in the dynamics file
#     self.M_func = dill.load(open("hb_M_func.pkl", "rb"))
#     self.C_func = dill.load(open("hb_C_func.pkl", "rb"))
#     self.dP_dq_func = dill.load(open("hb_dP_dq_func.pkl", "rb"))
#     self.tau_func = dill.load(open("hb_tau_func.pkl", "rb"))

# %%
# Method 2 (recommended): generate readable Python code
import os
import inspect
import re
import hummingbirdParam as P


# Function to replace dummy variables with x and u
def replace_dummy_vars(expr) -> str:
    # Extract first two dummy variables from function def line
    match = re.search(r"def _lambdifygenerated\(\s*(\S+),\s*(\S+),", expr)
    if not match:
        return expr  # No match found, return original expression
    dummy_1, dummy_2 = match.group(1), match.group(2)
    # Replace all occurrences of dummy_1 and dummy_2 in the entire string
    pattern = re.compile(
        r"\b(" + re.escape(dummy_1) + r"|" + re.escape(dummy_2) + r")\b"
    )

    def replacement(m):
        if m.group(0) == dummy_1 and "_Dummy_" in dummy_1:
            return "x"
        elif m.group(0) == dummy_2 and "_Dummy_" in dummy_2:
            return "u"
        else:
            return m.group(0)

    result = pattern.sub(replacement, expr)
    return result


def sym_expr_to_fn_string(func, func_name, return_name):
    func_str = inspect.getsource(func)
    func_str = re.sub("array", "np.array", func_str)
    func_str = re.sub("cos", "np.cos", func_str)
    func_str = re.sub("sin", "np.sin", func_str)
    func_str = replace_dummy_vars(func_str)
    func_str = re.sub("_lambdifygenerated", func_name, func_str)
    func_str = re.sub("= x", "= np.ravel(x)  # make 1D", func_str)
    func_str = re.sub("= u", "= np.ravel(u)  # make 1D", func_str)
    func_str = re.sub("return", f"{return_name} =", func_str)
    func_str += f"    return {return_name}\n"
    return func_str


def save_generated_functions(filename = "eom_generated.py"):
    # Save file to correct location
    save_dir = os.path.dirname(P.__file__)
    save_file = os.path.join(save_dir, filename)
    cur_file = os.path.split(__file__)[-1]

    # Comment out the if block if you always want to overwrite the exiting file
    if os.path.exists(save_file):
        print("(in VSCode, use command bar at the top of the window to input response)")
        print(f"{filename} already exists...")
        overwrite = input("Overwrite it? (y/n): ")
        if not overwrite.lower() == "y":
            print("Not overwriting.")
            return
        else:
            print("Overwriting now...")

    with open(save_file, "w") as f:
        f.write("#" * 80 + "\n")
        f.write(f"# This file was automatically generated by {cur_file}\n")
        f.write("#" * 80 + "\n")
        f.write("import numpy as np\n")
        f.write("\n\n")
        f.write(sym_expr_to_fn_string(calc_M, "calculate_M", "M"))
        f.write("\n\n")
        f.write(sym_expr_to_fn_string(calc_C, "calculate_C", "C"))
        f.write("\n\n")
        f.write(sym_expr_to_fn_string(calc_dP_dq, "calculate_dP_dq", "dP_dq"))
        f.write("\n\n")
        f.write(sym_expr_to_fn_string(calc_tau, "calculate_tau", "tau"))

    print(f"Saved generated functions to {save_file}")

save_generated_functions("eom_generated.py")

# %% [markdown]
# # Step 4: Example on how to use your generated EOM functions
# %%
# You can use your generated EOM functions as follows:
import numpy as np
import eom_generated as eom


param_vals = {
    "m_1": P.m1,
    "m_2": P.m2,
    "m_3": P.m3,
    "J_1x": P.J1x,
    "J_1y": P.J1y,
    "J_1z": P.J1z,
    "J_2x": P.J2x,
    "J_2y": P.J2y,
    "J_2z": P.J2z,
    "J_3x": P.J3x,
    "J_3y": P.J3y,
    "J_3z": P.J3z,
    "ell_1": P.ell1,
    "ell_2": P.ell2,
    "ell_3x": P.ell3x,
    "ell_3y": P.ell3y,
    "ell_3z": P.ell3z,
    "ell_T": P.ellT,
    "d": P.d,
    "g": P.g,
}

x = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]).T
u = np.array([[0.24468 / 2, 0.22468 / 2]]).T

np.set_printoptions(precision=4, suppress=True)

# NOTE: **param_vals unpacks the dictionary into keyword arguments where they
# become individual arguments in the function call and the order doesn't matter.

M_val = eom.calculate_M(x, **param_vals)
print("M_val:")
print(M_val, "\n")  # should be np.diag([0.0002, 0.0126, 0.0127])

C_val = eom.calculate_C(x, **param_vals)
print("C_val:")
print(C_val, "\n")  # should be all zeros

dP_dq_val = eom.calculate_dP_dq(x, **param_vals)
print("dP_dq_val:")
print(dP_dq_val, "\n")  # should be [[0], [0.0833], [0]]

tau_val = eom.calculate_tau(x, u, **param_vals)
print("tau_val:")
print(tau_val, "\n")  # should be [[0.0012], [0.0833], [0]]

