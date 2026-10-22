ISS Description
================

Package that has a simulation of the ISS and the Mobile Servicing System (MSS) composed of 3 parts:

1. Mobile Base System (MBS): Rack that travels along the trusses outside the ISS
2. Canadarm2: 7-DOF arm that can be attached to the MBS
3. Dextre: 15 DOF (2 arms with 7DOFs each and one roll hip joint) that can be attached to Canadarm2, MBS or the ISS surface.


Instructions
=============

1. To visualize the system in Rviz and move the joints using a Joint State Publisher GUI:

    ```
    ros2 launch iss_description view_mobile_servicing_system.launch.py
    ```
    
    ![Mobile Servicing System in Rviz](assets/images/mobile_servicing_system_rviz.png)
    
2. To simulate the system in Gazebo: 

   ```
    ros2 launch iss_description mobile_servicing_system_gz.launch.py
   ```
   
   To run a motion demo, add `move_demo:=true`. This starts `scripts/move_mss`, which sends a few preset `FollowJointTrajectory` goals to the Canadarm2 and Mobile Base System controllers and then exits:

   ```
   ros2 launch iss_description mobile_servicing_system_gz.launch.py move_demo:=true
   ```

   To command your own poses, send `FollowJointTrajectory` goals to `/canadarm2_joint_trajectory_controller/follow_joint_trajectory` or `/mobile_base_system_joint_trajectory_controller/follow_joint_trajectory`.
   
   ![Mobile Servicing System in Gazebo](assets/images/mobile_servicing_system_gazebo.png)

