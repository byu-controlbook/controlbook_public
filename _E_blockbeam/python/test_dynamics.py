#%%[markdown]

# The cell below can be used to double check the output of your dynamics file for hw 03. 

# The output for the block beam dynamics with the given input should be the following:
#
# $\left[ \begin{array}{c} 0.25001051 \\ 0.01173116 \\ 0.00420385 \\ 2.34618816 \end{array} \right]$
#%%

from hw3.blockbeamDynamics import blockbeamDynamics
blockbeam = blockbeamDynamics()

# this number of "100" for input is just for testing purposes
y = blockbeam.update(100.0)
print("Your state is:\n")
print(blockbeam.state)


