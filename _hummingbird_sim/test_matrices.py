"""
Module Name: test_matrices

Description: This module takes a studnet's HummingbirdDynamics class and tests
each element of each matrix against their correct values. It imports the file
'test_cases_matrices.pkl' which also includes the test cases and correct
matrix values. (List the keys to find the key for these test case inputs.)

You can test your helper function individually. The following is example code
for how to do this using variables and definitions from this file:

```python
# Test M matrix
_test_M(HummingbirdDynamics, tests_dict["Test #1"], "M")

# Test ParialP matrix
_test_partialP(HummingbirdDynamics, tests_dict["Test #4"], "PartialP")
```

Author: Benjamin Arnesen

Version: 1.0
"""

from hummingbird_dynamics_solution import HummingbirdDynamics
from typing import Type
import numpy as np
import pickle
import os

# Get the current script's directory
script_dir = os.path.dirname(os.path.realpath(__file__))

# Define the relative path to the pickle file
pickle_file_path = os.path.join(script_dir, 'test_cases_matrices.pkl')
print(pickle_file_path)

# Unpickle the dictionary
with open(pickle_file_path, 'rb') as file:
    tests_dict = pickle.load(file)


# initial_state, input_force):
def test_matrices(hb_dynamics: Type[HummingbirdDynamics], test: str):
    # Instantiate Hummingbird Dynamics Object and set its state variable
    hummingbird = hb_dynamics()
    hummingbird.state = tests_dict[test]["Init State"]
    single_test_dict = tests_dict[test]
    # Create lists of the functions and corresponding keys
    tests = [_test_M, _test_C, _test_partialP, _test_B, _test_tau]
    keys = ["M", "C", "PartialP", "B", "tau"]
    # Loop through the functions and pass the hummingbird dynamics class
    # to each of them.
    for (test, key) in zip(tests, keys):
        test(hummingbird, single_test_dict, key)


def _test_M(hb: HummingbirdDynamics, single_test_dict: dict, key: str):
    M_correct = single_test_dict[key]
    M_student = hb._M(hb.state)
    compare_matrices(M_correct, M_student, key)


def _test_C(hb: HummingbirdDynamics, single_test_dict: dict, key: str):
    C_correct = single_test_dict[key]
    C_student = hb._C(hb.state)
    compare_matrices(C_correct, C_student, key)


def _test_partialP(hb: HummingbirdDynamics, single_test_dict: dict, key: str):
    partialP_correct = single_test_dict[key]
    partialP_student = hb._partialP(hb.state)
    compare_matrices(partialP_correct, partialP_student, key)


def _test_tau(hb: HummingbirdDynamics, single_test_dict: dict, key: str):
    input_force = single_test_dict["Input"]
    pwm_left = input_force[0][0]
    pwm_right = input_force[1][0]
    force = hb.km * (pwm_left + pwm_right)
    torque = hb.d * hb.km * (pwm_left - pwm_right)
    tau_correct = single_test_dict[key]
    tau_student = hb._tau(hb.state, force, torque)
    compare_matrices(tau_correct, tau_student, key)


def _test_B(hb: HummingbirdDynamics, single_test_dict: dict, key: str):
    B_correct = single_test_dict[key]
    B_student = hb._B()
    try:
        if not np.allclose(B_correct, B_student, 0.0001):
            matrix_error(key)
    except:
        matrix_general_error(key)


def compare_matrices(correct_matrix, student_matrix, matrix_name):
    try:
        if correct_matrix.shape != student_matrix.shape:
            matrix_shape_error(matrix_name)

        for row, (correct_row, student_row) in enumerate(zip(correct_matrix, student_matrix)):
            for column, (correct_val, student_val) in enumerate(zip(correct_row, student_row)):
                if correct_val != student_val:
                    matrix_index_error(matrix_name, row, column)

    except:
        matrix_general_error(matrix_name)


def compare_matrices_dep(correct_matrix, student_matrix, matrix_name: str):
    tolerance = 0.0001
    try:
        if correct_matrix.shape != student_matrix.shape:
            matrix_shape_error(matrix_name)

        for index, (correct_val, student_val) in enumerate(zip(correct_matrix, student_matrix)):
            # if correct_val != student_val:
            if abs(correct_val - student_val) > tolerance:
                matrix_index_error(matrix_name, index, 0)
    except:
        matrix_general_error(matrix_name)


def matrix_index_error(matrix_name: str, row, column):
    print(
        f"\nRow {row} and Column {column} of your {matrix_name} is incorrect. (Matrices are 0-indexed)")


def matrix_error(matrix_name: str):
    print(f"\nYour {matrix_name} matrix is incorrect")


def matrix_general_error(matrix_name: str):
    print(
        f"\nYour {matrix_name} matrix has something... wrong with it. (This is an unanticipated error)")


def matrix_shape_error(matrix_name: str):
    print(f"\nYour {matrix_name} is the wrong shape")


# You can also run each test one at a time. Edit the line below. Specify the
    # function name, the test you are running, and the key
# _test_M(HummingbirdDynamics, tests_dict["Test #1"], "M")
