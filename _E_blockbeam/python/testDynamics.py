# use this file to test your dynamics file
import numpy as np
from blockBeamDynamics import blockBeamDynamics
blockBeam = blockBeamDynamics()

flag = True

# ------------------------------------
initial_state = np.array([[0.], [0.], [0.], [0.]])
input = 0
blockBeam.state = initial_state
y = blockBeam.update(input)
true_state = np.array([[ 1.20295114e-07], [-1.47149987e-03], [ 4.81180391e-05], [-2.94299920e-01]])
if np.linalg.norm(blockBeam.state - true_state)>0.001:
    print("Test failed for initial state: \n", initial_state.T, "\n and input: \n ", input ) 
    flag = False
    print("\n The state should be: \n", blockBeam.state.T)


# ------------------------------------
initial_state = np.array([[1.], [0.], [0.], [0.]])
input = 10.
blockBeam.state = initial_state
y = blockBeam.update(input)
true_state = np.array([[ 1.00000006e+00], [-3.23080640e-04], [ 2.44822171e-05], [-6.46161234e-02]])
if np.linalg.norm(blockBeam.state - true_state)>0.001:
    print("Test failed for initial state: \n", initial_state.T, "\n and input: \n ", input ) 
    flag = False
    print("\n The state should be: \n", blockBeam.state.T)

# ------------------------------------
initial_state = np.array([[0.], [1.], [0.], [0.]])
input = 100.
blockBeam.state = initial_state
y = blockBeam.update(input)
true_state = np.array([[-4.13065294e-04], [ 1.00729563e+00], [-8.26787157e-02], [ 1.45635351e+00]])
if np.linalg.norm(blockBeam.state - true_state)>0.001:
    print("Test failed for initial state: \n", initial_state.T, "\n and input: \n ", input ) 
    flag = False
    print("\n The state should be: \n", blockBeam.state.T)

# ------------------------------------
initial_state = np.array([[0.], [0.], [1.], [0.]])
input = 5.
blockBeam.state = initial_state
y = blockBeam.update(input)
true_state = np.array([[ 1.00000599e-02], [-7.24857374e-04], [ 1.00002420e+00], [-1.45299527e-01]])
if np.linalg.norm(blockBeam.state - true_state)>0.001:
    print("Test failed for initial state: \n", initial_state.T, "\n and input: \n ", input ) 
    flag = False
    print("\n The state should be: \n", blockBeam.state.T)

# ------------------------------------
initial_state = np.array([[1.], [0.], [0.], [1.]])
input = 50.
blockBeam.state = initial_state
y = blockBeam.update(input)
true_state = np.array([[1.00005985], [0.01161207], [0.01302769], [1.32236537]])
if np.linalg.norm(blockBeam.state - true_state)>0.001:
    print("Test failed for initial state: \n", initial_state.T, "\n and input: \n ", input ) 
    flag = False
    print("\n The state should be: \n", blockBeam.state.T)

# ------------------------------------
initial_state = np.array([[-1.], [-10.], [0.], [1.]])
input = -10.
blockBeam.state = initial_state
y = blockBeam.update(input)
true_state = np.array([[-1.00031903], [-9.98947492], [-0.06402969], [ 1.10496488]])
if np.linalg.norm(blockBeam.state - true_state)>0.001:
    print("Test failed for initial state: \n", initial_state.T, "\n and input: \n ", input ) 
    flag = False
    print("\n The state should be: \n", blockBeam.state.T)
    
# ------------------------------------
initial_state = np.array([[0.], [5.], [-5.], [-5.]])
input = -1.
blockBeam.state = initial_state
y = blockBeam.update(input)
true_state = np.array([[-0.04954846], [ 4.94965778], [-4.91159123], [-5.05678365]])
if np.linalg.norm(blockBeam.state - true_state)>0.001:
    print("Test failed for initial state: \n", initial_state.T, "\n and input: \n ", input ) 
    flag = False
    print("\n The state should be: \n", blockBeam.state.T)
    
# ------------------------------------
initial_state = np.array([[1.], [2.], [3.], [4.]])
input = -20.
blockBeam.state = initial_state
y = blockBeam.update(input)
true_state = np.array([[1.03036301], [2.03993544], [3.07353016], [3.9880112 ]])
if np.linalg.norm(blockBeam.state - true_state)>0.001:
    print("Test failed for initial state: \n", initial_state.T, "\n and input: \n ", input ) 
    flag = False
    print("\n The state should be: \n", blockBeam.state.T)
    
# ------------------------------------
initial_state = np.array([[-1.], [2.], [-3.], [4.]])
input = -50.
blockBeam.state = initial_state
y = blockBeam.update(input)
true_state = np.array([[-1.03125809], [ 2.0402445 ], [-3.25282951], [ 4.04795092]])
if np.linalg.norm(blockBeam.state - true_state)>0.001:
    print("Test failed for initial state: \n", initial_state.T, "\n and input: \n ", input ) 
    flag = False
    print("\n The state should be: \n", blockBeam.state.T)
    
# ------------------------------------
initial_state = np.array([[1.], [1.], [1.], [1.]])
input = 50.
blockBeam.state = initial_state
y = blockBeam.update(input)
true_state = np.array([[1.00964205], [1.01079258], [0.92890248], [1.15743875]])
if np.linalg.norm(blockBeam.state - true_state)>0.001:
    print("Test failed for initial state: \n", initial_state.T, "\n and input: \n ", input ) 
    flag = False
    print("\n The state should be: \n", blockBeam.state.T)
    

if flag is True:
    print("\n\nCongrats!! \nYour dynamics file has passed all of the tests!\n")
