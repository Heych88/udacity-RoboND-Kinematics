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

[image1]: ./misc_images/robot_DH.jpg
[image2]: ./misc_images/robot_arm2.png
[image3]: ./misc_images/Original_DH.jpg
[image4]: ./misc_images/transform_formula.png
[image5]: ./misc_images/calcs.jpg
[image6]: ./misc_images/robot_arm_axis.png
[image7]: ./misc_images/IK_calculations.jpg
[image8]: ./misc_images/Mod_DH_parameters.jpg

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

From the above image, the following Modified Denavit-Hartenberg parameters have been derived.

#### Modified Denavit-Hartenberg parameters

![Modified DH model][image1]

Link | a | alpha | d | theta
--- | --- | --- | --- | ---
1 | a1 | pi/2 | d1 | theta1*
2 | a2 | 0 | 0 | theta2* + pi/2
3 | -a3 | pi/2 | 0 | theta3*
4 | 0 | -pi/2 | d4 | theta4*
5 | 0 | pi/2 | 0 | theta5*
6 | 0 | 0 | 0 | theta6*
g (gripper) | 0 | 0 | dg | 0

The above parameters have been obtained by the following method.
* a(i) is the distance from Z(i-1) to Z(i) measured along X(i-1).
* alpha(i) is the angle from Z(i-1) to Z(i) measured about X(i-1).
* d(i) is the distance from X(i-1) to X(i) measured along Z(i).
* theta(i) is the angle from X(i-1) to X(i) measured about Z(i). **

** On a revolute joint, theta is the variable parameter of the angle of rotation.

The values for *a* and *d* have been obtained from the `./kuka_arm/urdf/kr210.urdf.xacro` file.

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

The Modified Transformation formula for each link is:

![Modified DH model transform matrix][image4]

Substituting the modified DH parameters for each link and simplifying, we obtain the following link matrices.

![Modified DH model transform matrix][image5]


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Below is an outline of the mathematics for each joint angles.
To determine the joint angles, the Kr210 arm was able to be divided into two sections. The coordinate frames of the robot are, upon startup, fixed at the base of the robot with the X-axis parallel to the link 3, the y-axis is perpendicular the robot arms and the z-axis is towards the sky.
The following image shows the world coordinates. The x-axis is red, the y-axis is green and the z-axis is blue.

 ![axis coordinates][image6]

The first containing joints 1, 2, and 3, as well as the base link, arm 1 and arm 2. This 3DOF (RRR) setup can be considered as the elbow section of the arm.
The inverse joint angle calculation of this elbow can then be derived by using basic trigonometry and the [law of cosine](http://mathworld.wolfram.com/LawofCosines.html).

##### overview
* Joint 1: Angle from position in the x-y plane
* Joint 2: [Law of cosine](http://mathworld.wolfram.com/LawofCosines.html) in the x, y, z planes and perpendicular to the z-plane.
* Joint 3: The difference in the [law of cosine](http://mathworld.wolfram.com/LawofCosines.html) angle a continuation of link 2.

The remaining joints make up the wrist section of the arm, and joint angles have been derived using trigonometric laws and current robot orientation and Position.

##### overview
* Joint 4: Tangent to a centre point in the centre of the pickup shelve location. I.e., joint three will rotate around the centre point as if it was the tangent of the circle with a zero angle when at the bottom, pi/2 rotation angle on the left side, pi rotation when verticle and 3*pi/2 on the right side.
* Joint 5: Angle is derived from the most direct line to the target pointing the gripper at the target.
* Joint 6: negative of Joint 4's angle.

Below are the calculations of all 6 joint angles.

 ![angle calculations][image7]

The homogeneous transform matrix from base_link to gripper_link using the roll, pitch and yaw of the gripper is below.

The code for the homogeneous transform matrix calculation is located in the file `IK_server.py`, Lines 247 to 314. The code calculates the forward kinematics of the arm, and the returned transform matrix is then multiplied by rotation in the y then z axes. The rotation is required as the gripper reference frame is not in the same orientation as the base coordinate frame. The rotation order required is.

* pi/2 about the y-axis.
* pi about the z-axis

The rotation matrices code in implemented in the file `IK_server.py`, lines 50 to 84. the forward kinematics and rotation calculation is located at line 359.

**roll**:  0.000746790549589  
**pitch**:  -0.000894914789619   
**yaw**:  0.000628117059808

[ 0.908286873705491,   0.32661881925477, -0.261409835245205,  0.934852577652669],
[ 0.376701966815537, -0.910326263542498,  0.171469303672875,   0.48870185330118],
[-0.181963137067955, -0.254216916851288, -0.949875347577257, -0.714489604691983],
 [0, 0, 0, 1]]


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.

The code mentioned in the following outline can be found in the `IK_server.py` file located `./kuka_arm/scripts/`.

The `IK_server.py` file is called externally with the desired gripper trajectory by the function `handle_calculate_IK()`, function lines 316 to 364. Once a valid arm trajectory has been passed into the function, the trajectory x, y, & z positions for specific points along the trajectory are first passed into the function `arm_IK()`, line 340, where the first three angles of the arm are calculated. These angles correspond to the rotation around the room, *theta 1*, arm 1 position relative to the base frame, *theta 2*, and the arm 2 position relative to arm 1, *theta 3*.

The code for the function call `arm_IK` is located between lines 127 and 195. The code implements the calculations derived above with *theta 1* calculated on line 144.

Before *theta 2* and *theta 3* are calculated, a new reference frame is calculated, removing the offsets from the base frame and the wrist. The code for the reference frame is on lines 146 to 154.

On the *KR210* model there is an offset observed on arm 2 due to the mechanical positioning of the wrist motors. The correction of this offsets length and angle are located on lines 167 & 168.

*Theta 2* angle calculations can now be calculated, as derived earlier, between lines 170 and 186.   

*Theta 3* angle calculation follows on from this and the code is located between lines 189 and 193.

After the calculation of the arm angles, the wrist angles are then calculated by calling the function `wrist_IK()` at line 343. The code for this function is located between lines 197 and 245.

The wrist rotation angle *theta 4* is calculated by creating a reference plane that represents the target shelf space, herein called target-frame, with an origin at the height and world frames y-axis value when the *kr210* is at rest, theta 1 = 2 = 3 = 4 = 5 = 6 => zero. *Theta 4* is calculated so to keep *theta 5*'s z-axis perpendicular to the target-frame origin point where angles closer to the ground will have an angle = 0, and the rotation angle will increase as the arm moves in a clockwise direction around the target-frame origin point amounting to a full rotation angle of 2*pi. That is as the robot looks to the left side of the target frame, *theta 4* rotation angle will be pi/2. The code is located between lines 223 and 226.   

*Theta 5* angle calculation follows on from this and uses the calculated arm end position to keep the gripper level at all times. The code is located between lines 229 and 238.

*Theta 6* is the inverse of *theta 4* to keep the gripper claws level when possible and the code is located at lines 242.
