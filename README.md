# Introduction to Feedback Control Using Design Studies

<img src="./bookcover.jpg" width="100" />

[Randal W. Beard](https://ece.byu.edu/directory/randy-beard), 
[Timothy W. McLain](http://me.byu.edu/faculty/timmclain)
[Cammy Peterson](https://ece.byu.edu/directory/cammy-peterson)
[Marc Killpack](https://www.me.byu.edu/directory/marc-killpack)

[Amazon](https://www.amazon.com/Introduction-Feedback-Control-Design-Studies/dp/1073396711/ref=sr_1_8?crid=36TN6HXOVZL2J&keywords=introduction+to+feedback+control&qid=1563317351&s=gateway&sprefix=introduction+to+feedba%2Caps%2C158&sr=8-8)

[PDF Version of Book](https://drive.google.com/file/d/1DxioCcBOJl-DoIBkDm8J2_ThItXbGx6e/view?usp=sharing)
    - A PDF version of the book is available at this link.
    - When typos are found, they will be fixed and the most recent version of the book will be posted here.
    - Please send typos and other suggestions to beard@byu.edu.


# LECTURE MATERIAL 
(Under construction) The following lecture materials are included as a resource for instructors.  The slides closely follow the book.  We welcome suggestions on how these slides might be improved.//

| Chapter | PDF Slides | Powerpoint | Last Modified |
|---------|------------|------------|----------|
| Chapter 1 - Introduction                  | [chapter1.pdf](https://drive.google.com/file/d/1EEJYRFGhS33oQ6utmfpvlL_AwMsa_qnA/view?usp=sharing)  | [chapter1.pptx](https://docs.google.com/presentation/d/1E64UCiuXwzBCBfGddauZVB_SIuUA60Qf/edit?usp=sharing&ouid=115325376918178448854&rtpof=true&sd=true)  | Sept 2019 |
| Chapter 2 - Kinetic Energy                | [chapter2.pdf]()  | [chapter2.pptx]()  | Sept 2021 |
| Chapter 3 - Euler Lagrange                | [chapter3.pdf]()  | [chapter3.pptx]()  | Sept 2021 |
| Chapter 4 - Linearization                 | [chapter4.pdf]()  | [chapter4.pptx]()  | Sept 2021 |
| Chapter 5 - Transfer Function Models      | [chapter5.pdf]()  | [chapter5.pptx]()  | Sept 2021 |
| Chapter 6 - State Space Models            | [chapter6.pdf]()  | [chapter6.pptx]()  |  |
| Chapter 7 - Second Order Systems          | [chapter7.pdf]()  | [chapter7.pptx]()  |  |
| Chapter 8 - Second Order Design           | [chapter8.pdf]()  | [chapter8.pptx]()  |  |
| Chapter 9 - Integrators                   | [chapter9.pdf]()  | [chapter9.pptx]()  |  |
| Chapter 10 - Digital PID                  | [chapter10.pdf]() | [chapter10.pptx]() |  |
| Chapter 11 - Full State Feedback          | [chapter11.pdf]() | [chapter11.pptx]() |  |
| Chapter 12 - Full State with Integrators  | [chapter12.pdf]() | [chapter12.pptx]() |  |
| Chapter 13 - Observers                    | [chapter13.pdf]() | [chapter13.pptx]() |  |
| Chapter 14 - Disturbance Observers        | [chapter14.pdf]() | [chapter14.pptx]() |  |
| Chapter 15 - Frequency Response           | [chapter15.pdf]() | [chapter15.pptx]() | Nov 2021  |
| Chapter 16 - Frequency Specifications     | [chapter16.pdf]() | [chapter16.pptx]() | Nov 2021  |
| Chapter 17 - Robustness Margins           | [chapter17.pdf]() | [chapter17.pptx]() | Nov 2021 |
| Chapter 18 - Compensator Design           | [chapter18.pdf]() | [chapter18.pptx]() | Nov 2021 |


# Homework Solutions

This Gitlab account contains complete python, matlab, and simulink solutions to the three design problems presented in the book.  We will be actively maintaining the Python solutions.

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

# Hummingbird Lab
[PDF Lab Manual](https://drive.google.com/file/d/1DzFGTip7jdT5Q33OeuZrK28yvU95lOPe/view?usp=sharing)
Whirlybird_ws
Support Files
ROS Tutorial
