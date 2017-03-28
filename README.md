# PR2 Programming by Demonstration
[![Build Status](https://travis-ci.org/PR2/pr2_pbd.svg?branch=indigo-devel)](https://travis-ci.org/PR2/pr2_pbd)
[![Coverage Status](https://coveralls.io/repos/PR2/pr2_pbd/badge.png?branch=hydro-devel)](https://coveralls.io/r/PR2/pr2_pbd?branch=hydro-devel)

This repository contains the work of [Maya Cakmak](http://www.mayacakmak.com/) and the [Human-Centered Robotics Lab](https://hcrlab.cs.washington.edu/) at the University of Washington. Please see those sites for citing publications. We abbreviate Programming by Demonstration with PbD.

## System Requirements
Currently the PbD system has the following requirements:
- Ubuntu 14.04
- ROS Indigo
- [mongo_msg_db](https://github.com/jstnhuang/mongo_msg_db) and [mongo_msg_db_msgs](https://github.com/jstnhuang/mongo_msg_db_msgs)

## Installing
Clone this repository and build on both your desktop machine and on the robot:
```bash
cd ~/catkin_ws/src
git clone https://github.com/hcrlab/blinky.git
git clone https://github.com/jstnhuang/mongo_msg_db_msgs.git
git clone https://github.com/jstnhuang/mongo_msg_db.git
git clone https://github.com/jstnhuang/stf.git
git clone https://github.com/jstnhuang/rapid.git
git clone https://github.com/PR2/pr2_pbd.git
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro=indigo -y
catkin_make
```

## Running
### PR2
#### Commands on PR2 (`c1`)
```bash
robot claim
robot start
source ~/catkin_ws/devel/setup.bash
roslaunch pr2_pbd_interaction pbd_backend.launch
```

### Desktop
```bash
setrobot <ROBOT_NAME>
roslaunch pr2_pbd_interaction pbd_frontend.launch
roslaunch pr2_pbd_interaction pbd_frontend.launch  # rviz, rqt, speech

# Optionally open PR2 dashboard in another terminal window
setrobot <ROBOT_NAME>
rosrun rqt_pr2_dashboard rqt_pr2_dashboard # Optional
```

Plug in a microphone to your computer.
Speak into the microphone to issue speech commands to the robot.
The voice commands are not currently documented.

## Running in simulation (untested)
```bash
roslaunch pr2_pbd_interaction pbd_simulation_stack.launch
```

## Running tests (not currently working)
### Desktop
```bash
rostest pr2_pbd_interaction test_endtoend.test
```

### PR2
```bash
roscd pr2_pbd_interaction
python test/test_endtoend_realrobot.py
```

### Code coverage (not currently working)
After running the tests, you can view code coverage by opening `~/.ros/htmlcov/index.html` with a web browser. Note that you can also view code coverage for normal execution by passing `coverage:=true` when launching `pr2_pbd_backend`.

With an account setup at [Coveralls](https://coveralls.io), edit the `.coveralls.yml` with your repo_token, and track coverage there by running `coveralls --data_file ~/.ros/.coverage`.

## Contributing
Before creating a pull request, please do the following things:

0. Lint your Python to pep8 standards by running `pep8 file1 file2 ...`.
0. Optionally format all your Python code with `yapf` and all your C++ code with `clang-format`. See the HCR Lab's [auto code formatting guide](https://github.com/hcrlab/wiki/blob/master/development_environment_setup/auto_code_formatting.md).
0. Run the tests on your desktop (see above).

(Untested) To lint all python files in common directories, run the tests on the desktop, open up code coverage with Google Chrome, and send the results to Coveralls (assuming Coveralls account and `.coveralls.yml` correctly setup), we have provided a script:
```bash
$ roscd pr2_pbd_interaction; ./scripts/test_and_coverage.sh
```

## Questions, feedback, improvements
Please use the Github issues page for this project.
