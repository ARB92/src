# Installation and usage guidelines

mkdir catkin_ws

git clone https://github.com/ARB92/src.git

cd catkin_ws/src

catkin_make

source the setup files
 
for launching the ur5 simulation and joint values publisher
```bash
  roslaunch ur5_joint_publisher ur5.launch
  ```
 

 
for launching the kinematic solver and random force generator

```bash
  roslaunch ur5_kdl_solver ur5.launch
  ```
  


for verifying the state machine:

```bash
rosrun admittance_controller_smach random_signal.py
rosrun admittance_controller_smach state_machine.py
```
