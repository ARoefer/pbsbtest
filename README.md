The pbsbtest package
====================
This package generates sensor readouts for different wiping motions executed by a robotic arm on a table. It uses a [UR5](https://github.com/code-iai/iai_table_robot_description) as robot, [giskardpy](https://github.com/SemRoCo/giskardpy) to generate trajectories and [iai_bullet_sim](https://github.com/ARoefer/iai_bullet_sim) to simulate the execution.


Installation
------------
The package can be installed using *wstool*. Note, the installation instructions for [giskardpy](https://github.com/SemRoCo/giskardpy) and [iai_bullet_sim](https://github.com/ARoefer/iai_bullet_sim).


Usage
-----
The package provides four launch files.
 - force_test_wipe.launch
 - force_test_bolt.launch
 - force_test_box.launch
 - force_test_box_no_wipe.launch

All of them create a wiping trajectory for the robot, play the trajectory in a loop and publish the measured forces at the wrist to a topic.
*force_test_wipe.launch* has the robot wiping over an empty table.
*force_test_bolt.launch* places the head of a bolt in the robot's wiping path.
*force_test_box.launch* puts a box in the robot's wiping path.
*force_test_box_no_wipe.launch* again puts a box in the robot's path, but the robot is hovering over table instead of wiping across it.