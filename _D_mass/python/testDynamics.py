# run this file to test your dynamics file
import numpy as np
from massDynamics import massDynamics


# x = [z, zdot]
x_tests = np.array([
    [0, 0],
    [1, 0],
    [1, 1],
    [0, 1],
    [1, -1],
    [-1, 1],
    [3, -2],
    [-5, 0],
    [0, -5],
    [-1, -1]
], dtype=np.float64).reshape(-1, 2, 1)

# u = F
u_tests = np.array([0, 10, 100, 5, -5, 1, -3, 50, -10, -1], dtype=np.float64)

xdot_tests = np.array([
    [0, 0],
    [0, 1.4],
    [1, 19.3],
    [1, 0.9],
    [-1, -1.5],
    [1, 0.7],
    [-2, -2.2],
    [0, 13],
    [-5, -1.5],
    [-1, 0.5]
], dtype=np.float64).reshape(-1, 2, 1)

num_tests = len(x_tests)
num_passed = 0
system = massDynamics()
xdot_eqns = np.array(['zd', 'zdd'])
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
