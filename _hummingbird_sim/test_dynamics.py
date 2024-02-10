# use this file to test your dynamics file
import numpy as np
# from hummingbirdDynamics import HummingbirdDynamics as dynamics
from hummingbird_dynamics_solution import HummingbirdDynamics as dynamics
import hummingbirdParam as P
import testCases as TC
import test_matrices as tm

# define test parameters
P.Ts = .01
plant = dynamics()
all_tests_passed = True
numTests = 0

# test all test cases
for initial_state, input, true_state in TC.dynamics_test_cases:
    numTests += 1
    # Check if the returned state is close enough to what it should be
    initial_state = np.array(initial_state)
    true_state = np.array(true_state)
    plant.state = initial_state
    plant.update(np.array(input))
    actual_state = plant.state
    error = np.linalg.norm(actual_state - true_state)
    if error > TC.errorTolerance:
        all_tests_passed = False
        print(f'Test #{numTests} failed with an error magnitude of {error}')
        print(f"\tThe actual output was {actual_state.flatten()}")
        print(f"\tIt should've been: {true_state.flatten()}")
        tm.test_matrices(dynamics, f"Test #{numTests}")
    # Test if the h() function is working
    try:
        midpoint = len(plant.state) // 2
        for i in range(midpoint):
            if plant.state[i][0] != plant.h()[i][0]:
                all_tests_passed = False
                print("Fix your h() function. ",
                      "This function returns your generalized coordinates (the positions) as a 2D array")
    except TypeError:
        all_tests_passed = False
        print("Your h() function probably is not returning a 2D array. ",
              "Return your generalized coordinates in a 2D array.")

    if all_tests_passed:
        print(f"Test #{numTests} passed!")
    else:
        print(f"\nTesting failed on test #{numTests}!")

# display final results of the tests
if all_tests_passed:
    print("\nExcellent work!! Your dynamics file has passed all of the tests!\n")
else:
    print(
        f"\nHmm your dynamics file has not passed all the tests...\n")
