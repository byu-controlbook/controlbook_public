# use this file to test your dynamics file
import numpy as np
from hummingbirdDynamics import hummingbirdDynamics
hummingbird = hummingbirdDynamics()

flag = True

# ------------------------------------
initial_state = np.array([[0.], [0.], [0.], [0.], [0.], [0.]])
input = np.array([[0.], [0.]])
hummingbird.state = initial_state.copy()
y = hummingbird.update(input)
true_state = np.array([[ 0., -0.00033088, 0., 0., -0.06616823, 0.]]).T
if np.linalg.norm(hummingbird.state - true_state)>0.001:
    print("Test failed for initial state: \n", initial_state.T, "\n and input: \n ", input.T ) 
    flag = False
    print("The state should be: \n", hummingbird.state.T)
    print("\n")

# ------------------------------------
initial_state = np.array([[1.], [0.], [0.], [0.], [1.], [0.]])
input = np.array([[1.], [1.]])
hummingbird.state = initial_state.copy()
y = hummingbird.update(input)
true_state = np.array([[ 9.99996876e-01, 1.00216033e-02, 5.49387027e-04, -2.30596331e-04, 1.00432053e+00, 1.09865283e-01]]).T
if np.linalg.norm(hummingbird.state - true_state)>0.001:
    print("Test failed for initial state: \n", initial_state.T, "\n and input: \n ", input.T ) 
    flag = False
    print("The state should be: \n", hummingbird.state.T)
    print("\n")

# ------------------------------------
initial_state = np.array([[0.], [1.], [0.], [0.], [0.], [1.]])
input = np.array([[100.], [-100.]])
hummingbird.state = initial_state.copy()
y = hummingbird.update(input)
true_state = np.array([[ 7.31654632e-02, 9.99800348e-01, 9.95134499e-03, 1.45018687e+01, -4.01639444e-02, 9.86353614e-01]]).T
if np.linalg.norm(hummingbird.state - true_state)>0.001:
    print("Test failed for initial state: \n", initial_state.T, "\n and input: \n ", input.T ) 
    flag = False
    print("The state should be: \n", hummingbird.state.T)
    print("\n")

# ------------------------------------
initial_state = np.array([[0.], [0.], [1.], [0.], [-1.], [-1.]])
input = np.array([[5.], [2.]])
hummingbird.state = initial_state.copy()
y = hummingbird.update(input)
true_state = np.array([[0.02199107, -0.0080092, 0.99001172, 4.3592948, -0.60175799, -0.99604677]]).T
if np.linalg.norm(hummingbird.state - true_state)>0.001:
    print("Test failed for initial state: \n", initial_state.T, "\n and input: \n ", input.T ) 
    flag = False
    print("The state should be: \n", hummingbird.state.T)
    print("\n")

# ------------------------------------
initial_state = np.array([[1.], [0.], [0.], [1.], [0.], [-10.]])
input = np.array([[-5.], [-2.]])
hummingbird.state = initial_state.copy()
y = hummingbird.update(input)
true_state = np.array([[ 9.88553674e-01, -1.58600937e-03, -1.01880917e-01, -3.24348773e+00,-3.18605188e-01, -1.03759564e+01]]).T
if np.linalg.norm(hummingbird.state - true_state)>0.001:
    print("Test failed for initial state: \n", initial_state.T, "\n and input: \n ", input.T ) 
    flag = False
    print("The state should be: \n", hummingbird.state.T)
    print("\n")

# ------------------------------------
initial_state = np.array([[-1.], [-10.], [0.], [1.], [50.], [5.]])
input = np.array([[0.5], [0.1]])
hummingbird.state = initial_state.copy()
y = hummingbird.update(input)
true_state = np.array([[-9.83592926e-01, -9.49946956e+00,  4.55301806e-02,  2.39523860e+00, 5.00936012e+01,  4.30805861e+00]]).T
if np.linalg.norm(hummingbird.state - true_state)>0.001:
    print("Test failed for initial state: \n", initial_state.T, "\n and input: \n ", input.T ) 
    flag = False
    print("The state should be: \n", hummingbird.state.T)
    print("\n")

# ------------------------------------
initial_state = np.array([[0.], [5.], [-5.], [-5.], [5.], [-50.]])
input = np.array([[-0.9], [-1.5]])
hummingbird.state = initial_state.copy()
y = hummingbird.update(input)
true_state = np.array( [[ -0.07064212, 5.0763994, -5.47553565, -9.69996898, 10.24893856, -44.55460869]]).T
if np.linalg.norm(hummingbird.state - true_state)>0.001:
    print("Test failed for initial state: \n", initial_state.T, "\n and input: \n ", input.T ) 
    flag = False
    print("The state should be: \n", hummingbird.state.T)
    print("\n")

# ------------------------------------
initial_state = np.array([[1.], [2.], [3.], [4.], [5.], [6.]])
input = np.array([[-20.], [3.]])
hummingbird.state = initial_state.copy()
y = hummingbird.update(input)
true_state = np.array([[ 0.97876981,  2.0503235,   3.05911943, -8.13224847,  5.06405749,  5.83277169]]).T
if np.linalg.norm(hummingbird.state - true_state)>0.001:
    print("Test failed for initial state: \n", initial_state.T, "\n and input: \n ", input.T ) 
    flag = False
    print("The state should be: \n", hummingbird.state.T)
    print("\n")

# ------------------------------------
initial_state = np.array([[-1.], [2.], [-3.], [4.], [-5.], [6.]])
input = np.array([[5.], [5.]])
hummingbird.state = initial_state.copy()
y = hummingbird.update(input)
true_state = np.array([[-0.95579782, 1.95259208, -2.93505771, 4.83014109, -4.4735948, 6.9854602 ]]).T
if np.linalg.norm(hummingbird.state - true_state)>0.001:
    print("Test failed for initial state: \n", initial_state.T, "\n and input: \n ", input.T ) 
    flag = False
    print("The state should be: \n", hummingbird.state.T)
    print("\n")

# ------------------------------------
initial_state = np.array([[1.], [1.], [1.], [1.], [1.], [1.]])
input = np.array([[-2.], [-2.]])
hummingbird.state = initial_state.copy()
y = hummingbird.update(input)
true_state = np.array([[1.00861761, 1.00908968, 1.00861982, 0.72463434, 0.81866481, 0.72311757]]).T
if np.linalg.norm(hummingbird.state - true_state)>0.001:
    print("Test failed for initial state: \n", initial_state.T, "\n and input: \n ", input.T ) 
    flag = False
    print("The state should be: \n", hummingbird.state.T)
    print("\n")

if flag is True:
    print("\n\nCongrats!! \nYour dynamics file has passed all of the tests!\n")
