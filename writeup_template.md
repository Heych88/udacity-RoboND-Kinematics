## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code.


[//]: # (Image References)

[image1]: ./misc_images/Modified_DH.jpg
[image2]: ./misc_images/robot_arm.png
[image3]: ./misc_images/Original_DH.jpg
[image4]: ./misc_images/transform_formula.png
[image5]: ./misc_images/calcs.jpg
[image6]: ./misc_images/robot_arm_axis.png
[image7]: ./misc_images/IK_calculations.jpg

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Below shows an example of the Kuka KR210 6DOF arm used for this project.

![Kuka KR210 6DOF arm][image2]

From the above image the following Modified and original Denavit-Hartenberg parameters have been derived. Both model methods are shown but the method implemented in the code is from the original DH method. The two methods differ in coordinate frame assignment and parameter derivation. The original method was choosen as i have learnt and used this method in the past and are more comfortable with it.

#### Modified Denavit-Hartenberg parameters

![Modified DH model][image1]

Link | a | alpha | d | theta
--- | --- | --- | --- | ---
0 | 0 | 0 | d1 | 0
1 | a1 | -pi/2 | 0 | theta1*
2 | a2 | 0 | 0 | theta2*
3 | -a3 | -pi/2 | d3 | theta3*
4 | 0 | pi/2 | 0 | theta4*
5 | 0 | -pi/2 | 0 | theta5*
6 | 0 | 0 | d6 | theta6*

#### Original Denavit-Hartenberg parameters

![Original DH model][image3]

below is the model parameters derivation using the original Denavit-Hartenberg

Link | a | alpha | d | theta
--- | --- | --- | --- | ---
1 | a1 | pi/2 | d1 | theta1*
2 | a2 | 0 | 0 | theta2*
3 | -a3 | pi/2 | 0 | theta3*
4 | 0 | -pi/2 | d4 | theta4*
5 | 0 | pi/2 | 0 | theta5*
6 | 0 | 0 | d6 | theta6*

The values for *a* and *d* have been obtained from the `./kuka_arm/urdf/kr210.urdf.xacro` file.

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

The Modified Transformation formula for each link is:

![Modified DH model transform matrix][image4]

Substituting the modified DH parameters for each link and simplifing, we obtain the following link matrices.

![Modified DH model transform matrix][image5]


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Below is an outline of the mathematics for each joint angles.
To determine the joint angles, the Kr210 arm was able to be divided into two sections. The coordinate frames of the robot are, upon startup, fixed at the base of the robot with the X-axis parralel to the link 3, y-axis is perpendicular the robot arms and the z-axis is towards the sky.
The following image shows the world coordinates. x-axis is red, y-axis is green and the z-axis is blue.

 ![axis coordinates][image6]

The first containing joints 1, 2, and 3, as well as the base link, arm 1 and arm 2. This 3DOF (RRR) setup can be considered as the elbow section of the arm.
The inverse joint angle calculation of this elbow can then be derived by using basic trigonemetry and the [law of cosine](http://mathworld.wolfram.com/LawofCosines.html).

##### overview
* Joint 1: Angle from position in the x-y plane
* Joint 2: [Law of cosine](http://mathworld.wolfram.com/LawofCosines.html) in the x, y, z planes and perpendicular to the z-plane.
* Joint 3: The difference in the [law of cosine](http://mathworld.wolfram.com/LawofCosines.html) angle a continuation of link 2.

The remaining joints make up the wrist section of the arm and joint angles have been derived using a trigonometetric laws and current robot orientation and Position.

##### overview
* Joint 4: Tangent to a center point in the center of the pickup shelve location. i.e joint 3 will rotate around the center point as if it was the tangent of the circle with a zero angle when at the bottom, pi/2 rotation angle on the left side, pi rotation when verticle and 3*pi/2 on the right side.
* Joint 5: Angle is derived from the most direct line to the target pointing the gripper at the target.
* Joint 6: negitive of Joint 4's angle.

Below are the calculations of all 6 joint angles.

 ![angle calculations][image7]


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  
