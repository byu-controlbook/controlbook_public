# run this file to test your dynamics file
import numpy as np
from blockbeamDynamics import blockbeamDynamics


# x = [z, theta, zdot, thetadot]
x_tests = np.array([
    [0, 0, 0, 0],
    [1, 0, 0, 1],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [1, 0, 0, 1],
    [-1, -10, 0, 1],
    [0, 5, -5, -5],
    [1, 2, 3, 4],
    [-1, 2, -3, 4],
    [1, 1, 1, 1]
], dtype=np.float64).reshape(-1, 4, 1)

# u = F
u_tests = np.array([0, 10, 100, 5, 50, -10, -1, -20, -50, 50], dtype=np.float64)

xdot_tests = np.array([
    [0, 0, 0, -29.43],
    [0, 1, 1, -6.461612903225807],
    [0, 0, -8.254830360965466, 146.18959489874257],
    [1, 0, 0, -14.43],
    [0, 1, 1, 32.248064516129034],
    [0, 1, -6.336847097824718, 10.509776904035345],
    [-5, -5, 9.407047134445389, -9.199164674572426],
    [3, 4, 7.079792242840061, -1.4874023960907632],
    [-3, 4, -24.92020775715994, 5.0632857748213205],
    [1, 1, -7.254830360965465, 16.068864908171633]
], dtype=np.float64).reshape(-1, 4, 1)

num_tests = len(x_tests)
num_passed = 0
system = blockbeamDynamics()
xdot_eqns = np.array(['zd', 'thetad', 'zdd', 'thetadd'])
precision = 6

for i, (x, u, xdot_expected) in enumerate(zip(x_tests, u_tests, xdot_tests), start=1):
    xdot_actual = system.f(x, u)
    error = xdot_actual - xdot_expected
    correct_indices = np.abs(error) < 1e-12
    if correct_indices.all():
        print(f"Test {i:>2}: PASS")
        num_passed += 1
    else:
        incorrect_indices = np.argwhere(~correct_indices)
        print(f"Test {i:>2}: FAIL - check equations for {xdot_eqns[incorrect_indices[:,0]]}")
        for r,c in incorrect_indices:
            print(f'{xdot_eqns[r]:>20}: ', end='')
            print(f'yours = {xdot_actual[r,c]:<{precision+8}.{precision}g}', end='')
            print(f'expected = {xdot_expected[r,c]:.{precision}g}')

if num_passed == num_tests:
    print("\nExcellent work!! The f(x,u) function from your dynamics file has passed all of the tests!\n")
else:
    print(f"\nDarn, {num_passed}/{num_tests} of the tests passed... Keep it up! You'll get it!\n")
