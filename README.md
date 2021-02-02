# Stereo Vision Based Robotic Manipulator
## Projected Goal
The project is concentrated on designing and implementing an arduino based educational robotic arm platform that is flexible with its software and hardware implementation. Two types of hardware design goals, an accurate version and a cheap version, will be implemented.

## Projected Delieverables
* Build robotic manipulator that has 5 or more DOFs
* Implement motion planning
* Implement path control that controls the end-effector twist
* Detect Objects' position and orientation information (Jenga blocks for this project) with stereo vision camera
* Automatically stack Jenga blocks with the robotic manipulator aided by stereo vision
* Design and implement a cheap version of the stereo-arm system
* Design and implement a open source platform for people to replicate works


# Main Equipments and Material Used
## Accurate Version
* Arbotix-M controller
* PC
* 4x Dynamixel ax-12a servo motors
* FTDI cable
* 12v power supply
* ZED camera

## Cheap Version
* Card Stock
* Micro Servo
* Arduino Uno

# Simulation
## Inverse Kinematics of a 3 Linkages Manipulator
The pseudo inverse of the jacobian matrix is used here, thus the model can be generalized to 4 or more DOFs. The desired joint angles are solved with the numerical Newton Raphson algorithm.
3D parametric curves are being traced by the end effector of a 3 linkages robotic manipulator in the simulation. The results are shown below (The green curves are the desired path)

<p float="left">
   <img src="https://github.com/SamoaChen/Stereo-Vision-Based-Robotic-Manipulator/blob/main/image/SPIRAL.gif" width="45%" height="45%">
   <img src="https://github.com/SamoaChen/Stereo-Vision-Based-Robotic-Manipulator/blob/main/image/HEART.gif" width="45%" height="45%">
</p>


## Inverse Velocity Control of a 3 linkages manipulator
The inverse kinematics is only a feedforward algorithm, there is no way to correct for errors. Additionally, the inverse kinematics algorithm couldn't incorporate the mass information corresponding to each linkage, and inverse kinematics only provides a discrete configuration solution for the arm without considering the velocity of the end effector. In order to avoid jerky motion and to be able to incorporate mass information to decrease controlling effort, an inverse velocity algorithm aided by PID control is used here.(mass information can be incorporated into the algorithm in the future by optimizing the potential energy change and kinematics energy of the manipulator given inverse velocity constrain equation).

To test the eligibility of the inverse velocity algorithm, errors in angles and arms' length measurements are added into the model. The first GIF demonstrates the controlling result with only feedforward components, the second GIF is the path controlling result when the kp and ki components are added. (The green curves are the desired path and the blue curve is the actual end-effector trajectory)

<p float="left">
   <img src="https://github.com/SamoaChen/Stereo-Vision-Based-Robotic-Manipulator/blob/main/image/NO_CONTROL.gif" width="45%" height="45%">
   <img src="https://github.com/SamoaChen/Stereo-Vision-Based-Robotic-Manipulator/blob/main/image/WITH_CONTROL.gif" width="45%" height="45%">
</p>

# Prototype Implementation
## Accurate Hardware Design

A 3 linkages robotic arm prototype is built for implementing the algorithms mentioned above. The prototype is shown in the following picture. Given the currently limited manufacturing methods accessibility, the prototype is built out of chopsticks and zip ties.

<img src="https://github.com/SamoaChen/Stereo-Vision-Based-Robotic-Manipulator/blob/main/image/stereo_robot_arm.JPG" width="80%" height="80%">

### Inverse Velocity Control implementation of a 3 linkages manipulator

The gif on the left demonstrates the simulation of the 3 linkages robotic arm to scale executing a circular path control, the gif on the right demonstrates the circular path control of the actual 3 linkages robotic arm

<p float="left">
   <img src="https://github.com/SamoaChen/Stereo-Vision-Based-Robotic-Manipulator/blob/main/image/Circular_Path_Control_Simulation.gif" width="40%" height="40%">
   <img src="https://github.com/SamoaChen/Stereo-Vision-Based-Robotic-Manipulator/blob/main/image/Circular_Path_Control_Implementation.gif" width="53%" height="53%">
</p>

## Cheap Hardware Design

<p float="left">
   <img src="https://github.com/SamoaChen/Stereo-Vision-Based-Robotic-Manipulator/blob/main/image/paper_arm_pattern_design.jpg" width="40%" height="40%">
   <img src="https://github.com/SamoaChen/Stereo-Vision-Based-Robotic-Manipulator/blob/main/image/paper_arm_first_arm.jpg" width="53%" height="53%">
   <img src="https://github.com/SamoaChen/Stereo-Vision-Based-Robotic-Manipulator/blob/main/image/paper_arm_assembled.jpg" width="53%" height="53%">
</p>
