# Stereo Vision Based Robotic Manipulator
## Projected Goals
* Build robotic manipulator that has 4 or more DOFs
* Implement motion planning
* Implement path control that controls the end effector twist
* Detect Objects' position and orientation information (jenga blocks for this project) with stereo vision camera
* Automatically stack jenga blocks with the robotic manipulator aided by stereo vision


# Main Equipments and Material Used
* Arbotix-M controller
* PC
* 4x Dynamixel ax-12a servo motors
* FTDI cable
* 12v power supply
* ZED camera
# Simulation
## *Inverse Kinematics of a 3 Linkages Manipulator*
The pesudo inverse of the jacobian matrix is used here, thus the model can be generalized to 4 or more DOFs. The desired joint angles are solved with numerical Newton Raphson algorithm.
3D parametric curves are being traced by the end effector of a 3 linkages robotic manipulator in the simulation. The results are shown below (The green curves are the desired path)

<p float="left">
   <img src="https://github.com/SamoaChen/Stereo-Vision-Based-Robotic-Manipulator/blob/main/image/SPIRAL.gif" width="45%" height="45%">
   <img src="https://github.com/SamoaChen/Stereo-Vision-Based-Robotic-Manipulator/blob/main/image/HEART.gif" width="45%" height="45%">
</p>


## *Inverse Velocity Control of a 3 linkages manipulator*
The inverse kinematics is only a feedforward algorithm, there is no way to correct for errors. Additionally, inverse kinematics algorithm couldn't incorporate the mass information corresponding to each linkages, and inverse kinematics only provides discrete configuration solution for the arm without considering the velocity of the end effector. In order to avoid jerky motion and to be able to incorporate mass information to decrease controlling effort, inverse velocity algorithm aided by pid control is used here.(mass information can be incorporate into the algorithm in the future by optimizing the potential energy change and kinematics energy of the manipulator given inverse velocity constrain equation).

To test the eligibility of the inverse velocity algorithm, errors in angles and arms' length measurements are added into the model. The first GIF demonstrate the controlling result with only feedforward components, the second GIF is the path controlling result when the kp and ki components are added. (The green curves are the desired path and the blue curve is the actual end effector trajectory)

<p float="left">
   <img src="https://github.com/SamoaChen/Stereo-Vision-Based-Robotic-Manipulator/blob/main/image/NO_CONTROL.gif" width="45%" height="45%">
   <img src="https://github.com/SamoaChen/Stereo-Vision-Based-Robotic-Manipulator/blob/main/image/WITH_CONTROL.gif" width="45%" height="45%">
</p>


