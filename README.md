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
3D parametric curves are being traced by the end effector of a 3 linkages robotic manipulator in the simulation. The results are shown below

<p float="left">
   <img src="https://github.com/SamoaChen/Stereo-Vision-Based-Robotic-Manipulator/blob/main/image/SPIRAL.gif" width="45%" height="45%">
   <img src="https://github.com/SamoaChen/Stereo-Vision-Based-Robotic-Manipulator/blob/main/image/HEART.gif" width="45%" height="45%">
</p>


## *Inverse Velocity Control of a 3 linkages manipulator*
Because the inverse kinematics is a feedforward algorithm, there is 
