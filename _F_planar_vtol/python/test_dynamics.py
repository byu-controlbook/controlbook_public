# use this file to test your dynamics file
import numpy as np
from VTOLDynamics import VTOLDynamics
VTOL = VTOLDynamics()

flag = True

# ------------------------------------
initial_state = np.array([[0.], [0.], [0.], [0.], [0.], [0.]])
input = np.array([[0.], [0.]])
VTOL.state = initial_state
y = VTOL.update(input)
true_state = np.array([[ 0.0 ],[ -0.0004905 ],[ 0.0 ],[ 0.0 ],[ -0.0981 ],[ 0.0 ]])
if np.linalg.norm(VTOL.state - true_state)>0.001:
    print("Test failed for initial state: \n", initial_state.T, "\n and input: \n ", input.T ) 
    flag = False
    print("\n The state should be: \n", VTOL.state.T)

# ------------------------------------
initial_state = np.array([[1.], [0.], [0.], [0.], [1.], [0.]])
input = np.array([[1.], [1.]])
VTOL.state = initial_state
y = VTOL.update(input)
true_state = np.array([[ 1.0 ],[ 0.009576166666666667 ],[ 0.0 ],[ 0.0 ],[ 0.9152333333333333 ],[ 0.0 ]])
if np.linalg.norm(VTOL.state - true_state)>0.001:
    print("Test failed for initial state: \n", initial_state.T, "\n and input: \n ", input.T ) 
    flag = False
    print("\n The state should be: \n", VTOL.state.T)

# ------------------------------------
initial_state = np.array([[0.], [1.], [0.], [0.], [0.], [1.]])
input = np.array([[100.], [-100.]])
VTOL.state = initial_state
y = VTOL.update(input)
true_state = np.array([[ 0.0 ],[ 0.9995095 ],[ 0.07097560975609757 ],[ 0.0 ],[ -0.0981 ],[ 13.195121951219514 ]])
if np.linalg.norm(VTOL.state - true_state)>0.001:
    print("Test failed for initial state: \n", initial_state.T, "\n and input: \n ", input.T ) 
    flag = False
    print("\n The state should be: \n", VTOL.state.T)

# ------------------------------------
initial_state = np.array([[0.], [0.], [1.], [0.], [-1.], [-1.]])
input = np.array([[5.], [2.]])
VTOL.state = initial_state
y = VTOL.update(input)
true_state = np.array([[ -0.00019589716663562473 ],[ -0.010363805876177905 ],[ 0.9909146341463415 ],[ -0.03913663487795623 ],[ -1.072701887822407 ],[ -0.8170731707317074 ]])
if np.linalg.norm(VTOL.state - true_state)>0.001:
    print("Test failed for initial state: \n", initial_state.T, "\n and input: \n ", input.T ) 
    flag = False
    print("\n The state should be: \n", VTOL.state.T)

# ------------------------------------
initial_state = np.array([[1.], [0.], [0.], [1.], [0.], [-10.]])
input = np.array([[-5.], [-2.]])
VTOL.state = initial_state
y = VTOL.update(input)
true_state = np.array([[ 1.0099888586413523 ],[ -0.0007236371435574651 ],[ -0.10091463414634147 ],[ 0.99698850366762 ],[ -0.14468785878522158 ],[ -10.182926829268293 ]])
if np.linalg.norm(VTOL.state - true_state)>0.001:
    print("Test failed for initial state: \n", initial_state.T, "\n and input: \n ", input.T ) 
    flag = False
    print("\n The state should be: \n", VTOL.state.T)

# ------------------------------------
initial_state = np.array([[-1.], [-10.], [0.], [1.], [50.], [5.]])
input = np.array([[0.5], [0.1]])
VTOL.state = initial_state
y = VTOL.update(input)
true_state = np.array([[ -0.9900036662421552 ],[ -9.500470504176624 ],[ 0.05012195121951219 ],[ 0.9992334361065514 ],[ 49.90589832744725 ],[ 5.024390243902439 ]])
if np.linalg.norm(VTOL.state - true_state)>0.001:
    print("Test failed for initial state: \n", initial_state.T, "\n and input: \n ", input.T ) 
    flag = False
    print("\n The state should be: \n", VTOL.state.T)

# ------------------------------------
initial_state = np.array([[0.], [5.], [-5.], [-5.], [5.], [-50.]])
input = np.array([[-0.9], [-1.5]])
VTOL.state = initial_state
y = VTOL.update(input)
true_state = np.array([[ -0.049911970789789505 ],[ 5.049474626544621 ],[ -5.499817073170732 ],[ -4.983071399643783 ],[ 4.893792299551563 ],[ -49.96341463414634 ]])
if np.linalg.norm(VTOL.state - true_state)>0.001:
    print("Test failed for initial state: \n", initial_state.T, "\n and input: \n ", input.T ) 
    flag = False
    print("\n The state should be: \n", VTOL.state.T)

# ------------------------------------
initial_state = np.array([[1.], [2.], [3.], [4.], [5.], [6.]])
input = np.array([[-20.], [3.]])
VTOL.state = initial_state
y = VTOL.update(input)
true_state = np.array([[ 1.040056037532009 ],[ 2.050071851678183 ],[ 3.052987804878049 ],[ 4.010212214805676 ],[ 5.0144852681270775 ],[ 4.597560975609756 ]])
if np.linalg.norm(VTOL.state - true_state)>0.001:
    print("Test failed for initial state: \n", initial_state.T, "\n and input: \n ", input.T ) 
    flag = False
    print("\n The state should be: \n", VTOL.state.T)

# ------------------------------------
initial_state = np.array([[-1.], [2.], [-3.], [4.], [-5.], [6.]])
input = np.array([[5.], [5.]])
VTOL.state = initial_state
y = VTOL.update(input)
true_state = np.array([[ -0.9599597170681572 ],[ 1.9491805421518982 ],[ -2.94 ],[ 4.008712395677724 ],[ -5.163777752142141 ],[ 6.0 ]])
if np.linalg.norm(VTOL.state - true_state)>0.001:
    print("Test failed for initial state: \n", initial_state.T, "\n and input: \n ", input.T ) 
    flag = False
    print("\n The state should be: \n", VTOL.state.T)

# ------------------------------------
initial_state = np.array([[1.], [1.], [1.], [1.], [1.], [1.]])
input = np.array([[-2.], [-2.]])
VTOL.state = initial_state
y = VTOL.update(input)
true_state = np.array([[ 1.0101090777687989 ],[ 1.0094378342784316 ],[ 1.01 ],[ 1.021836953465583 ],[ 0.8876043738396339 ],[ 1.0 ]])
if np.linalg.norm(VTOL.state - true_state)>0.001:
    print("Test failed for initial state: \n", initial_state.T, "\n and input: \n ", input.T ) 
    flag = False
    print("\n The state should be: \n", VTOL.state.T)

if flag is True:
    print("\n\nCongrats!! \nYour dynamics file has passed all of the tests!\n")
