# use this file to test your dynamics file
import numpy as np
from massDynamics import massDynamics
mass = massDynamics()

flag = True

# ------------------------------------
initial_state = np.array([[0.], [0.]])
input = 0
mass.state = initial_state
y = mass.update(input)
true_state = np.array([[0.], [0.]])
if np.linalg.norm(mass.state - true_state)>0.001:
    print("Test failed for initial state: \n", initial_state.T, "\n and input: \n ", input ) 
    flag = False
    print("\n The state should be: \n", mass.state.T)

# ------------------------------------
initial_state = np.array([[1.], [0.]])
input = 10.
mass.state = initial_state
y = mass.update(input)
true_state = np.array([[1.00006998], [0.01399286]])
if np.linalg.norm(mass.state - true_state)>0.001:
    print("Test failed for initial state: \n", initial_state.T, "\n and input: \n ", input ) 
    flag = False
    print("\n The state should be: \n", mass.state.T)

# ------------------------------------
initial_state = np.array([[1.], [1.]])
input = 100.
mass.state = initial_state
y = mass.update(input)
true_state = np.array([[1.01096457], [1.19287161]])
if np.linalg.norm(mass.state - true_state)>0.001:
    print("Test failed for initial state: \n", initial_state.T, "\n and input: \n ", input ) 
    flag = False
    print("\n The state should be: \n", mass.state.T)

# ------------------------------------
initial_state = np.array([[0.], [1.]])
input = 5.
mass.state = initial_state
y = mass.update(input)
true_state = np.array([[0.01004488], [1.00896542]])
if np.linalg.norm(mass.state - true_state)>0.001:
    print("Test failed for initial state: \n", initial_state.T, "\n and input: \n ", input ) 
    flag = False
    print("\n The state should be: \n", mass.state.T)

# ------------------------------------
initial_state = np.array([[-5.], [0.]])
input = 50.
mass.state = initial_state
y = mass.update(input)
true_state = np.array([[-4.99935022], [ 0.12993372]])
if np.linalg.norm(mass.state - true_state)>0.001:
    print("Test failed for initial state: \n", initial_state.T, "\n and input: \n ", input ) 
    flag = False
    print("\n The state should be: \n", mass.state.T)

# ------------------------------------
initial_state = np.array([[0.], [-5.]])
input = -10.
mass.state = initial_state
y = mass.update(input)
true_state = np.array([[-0.05007447], [-5.0148424 ]])
if np.linalg.norm(mass.state - true_state)>0.001:
    print("Test failed for initial state: \n", initial_state.T, "\n and input: \n ", input ) 
    flag = False
    print("\n The state should be: \n", mass.state.T)

# ------------------------------------
initial_state = np.array([[-1.], [-1.]])
input = -1.
mass.state = initial_state
y = mass.update(input)
true_state = np.array([[-1.00997491], [-0.99497256]])
if np.linalg.norm(mass.state - true_state)>0.001:
    print("Test failed for initial state: \n", initial_state.T, "\n and input: \n ", input ) 
    flag = False
    print("\n The state should be: \n", mass.state.T)

if flag is True:
    print("\n\nCongrats!! \nYour dynamics file has passed all of the tests!\n")
