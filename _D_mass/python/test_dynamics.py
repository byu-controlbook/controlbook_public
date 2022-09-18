#%%[markdown]

# The cell below can be used to double check the output of your dynamics file for hw 03. 

# The output for the block beam dynamics with the given input should be the following:
#
# $\left[ \begin{array}{c} 0.00099966 \\ 0.19989803  \end{array} \right]$


#%%

from hw3.massDynamics import massDynamics
mass = massDynamics()

# this number of "100" for input is just for testing purposes
y = mass.update(100.0)
print("Your state is:\n")
print(mass.state)
