# mobile manipulator planning stack 
this is the source code of the algorithm for the mobile mainpulator planning<br>

## mm_manipulator
in the mm_manipulator package, we use a sampling based method to sample in the joint space 
of the manipulator(KUKA iiwa14) to compute its reachability and corresponding manipulability<br>
as for the kinematics of iiwa, we use [iiwa_stack](https://github.com/IFL-CAMP/iiwa_stack) and moveit to make it<br>
'''
roslaunch iiwa_moveit move_group.launch
'''
and get into the scripts directory and run
'''
./reachability.py
'''
the results are show in figures in */figure. the sampled results are not uploaded because it's to large<br>
the reachability_fitting.py is used to get the fitted bound of the sampled results<br>

## mm_planning
in this package, we use a cost function and the gradient desent method to compute the trajectory of the mobile base<br>
F=Fdistance+Fsmoothness +Fbound<br>
the results is show via opencv4<br>
