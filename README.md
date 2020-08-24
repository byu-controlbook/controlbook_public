# Code Examples for Introduction to Feedback Control: Using Design Studies

The content of the book has recently shifted to focus on using mostly Python as a development and learning tool. There are many reasons for this, but you will need to follow the installation instructions below in order to make the most use of the Python examples. Examples for the other platforms (e.g. Matlab and Simulink) remain intact, but may not be updated as often in the future. 

## Prerequisites

It is expected that you have installed python 3 and git. You may also want to install an IDE such as [PyCharm](https://www.jetbrains.com/pycharm/) to help you develop code more effectively. 

## Installing

First, you need to clone the repo. Navigate to the folder where you want to keep this code. You will then need to copy the url found under the blue "clone" button above on this page, and then insert that url into the command below:

```
git clone --recurse-submodules <insert URL>
```

This will check out the example code along with a python module that we require for later examples in the code to run. If you skip this step and execute "git clone" without the "--recurse-submodules" argument, you can always do the following later (after navigating to the main repo folder):

```
git submodule init
git submodule update
```

Once you have pulled down the repo and the submodule (for python-control), and are in the main repo folder, then you can do the following to install the submodule:
```
cd python-control
sudo python setup.py install
```

Now all of the examples in the python folders under each design study should work. Enjoy!
