 **Udacity-RoboND-Project2-Pick-and-Place**


#### **Project Goals:**

 1. Set Up Environment
 2. Explore forward kinematics with Kuka KR210 to learn more about the robot's geometry.
 3. Derive Modified DH parameters.
 4. Run Pick & Place in demo mode
 5. Perform kinematic analysis of robot and derive equations for individual joint angles.
 6. Write inverse kinematics code inside IK_server.py
 7. Make brief write up that includes all the rubric points and how you addressed each one.  The writeup / README should include a statement and supporting figures / images that explain how each rubric item was addressed, and specifically where in the code each step was handled. 
___

###
**Kinematic Analysis**

<strong> 
Start with the Kuka KR210 zero-configuration graph (all joint angles are assumed = 0), add gripper frame.  
</strong>  

![Kuka KR210 Graph](/images/IMG_0084%20(2).jpg) 

Hand-drawn labeling of: joints, joint axes, links, positive and <strong>x</strong> axes(common normals between <strong>z<sub>i-1</sub></strong> and <strong>z<sub>i</sub></strong>), and reference frame origins (intersection of <strong>x<sub>i</sub></strong> and <strong>z<sub>i</sub></strong>) on graph, non-zero link lengths (<strong>a</strong> values), link offsets (<strong>d</strong> values) and <strong>alpha</strong> (<strong>z</strong> twist angle), <strong>q</strong> values.  

![GitHub Logo](/images/AllLabels.jpg)  

<strong>Modified DH Parameter Table</strong>

**i** | **alpha<sub>i-1</sub>** | **a<sub>i-1</sub>** | **d<sub>i</sub>** | **theta (q<sub>i</sub>)**
:--: | :-----: | :-: | :-: | :-----:
1 | 0 | 0 | 0.75 | q1
2 | -pi/2 | 0.35 | 0 | q2-pi/2
3 | 0 | 1.25 | 0 | q2
4 | -pi/2 | -0.054 | 1.50 | q3
5 | pi/2 | 0 | 0 | q4
6 | -pi/2 | 0 | 0 | q5
G | 0 | 0 | 0.303 |  q6

<strong>a<sub>i-1</sub></strong> = distance from <strong>z<sub>i-1</sub></strong> and <strong>z<sub>i</sub></strong> measured along <strong>x<sub>i-1</sub></strong> axis  
<strong>d<sub>i</sub></strong> = signed distance between <strong>x<sub>i-1</sub></strong> and <strong>x<sub>i</sub></strong> measured along the <strong>z<sub>i</sub></strong> axis  
<strong>alpha<sub>i</sub></strong> = angle between <strong>z<sub>i-1</sub></strong> and <strong>z<sub>i</sub></strong> measured along the <strong>x<sub>i-1</sub></strong> axis according to the right hand rule  
<strong>theta<sub>i</sub></strong> = angle between <strong>x<sub>i-1</sub></strong> and <strong>x<sub>i</sub></strong> measured about <strong>z<sub>i</sub></strong> axis using right hand rule  
for <strong>i</strong>=2 there is a -90 degree constant offset between <strong>x<sub>1</sub></strong> and <strong>x<sub>2</sub></strong>.

Note: <strong>a</strong>, <strong>d</strong>, and <strong>alpha</strong>values were obtained from kr210.urdf.xacro file.  
___

The Modified DH parameter table above was plugged into the DH Transformation Matrix below to derive individual link to link transform matrices.

#### DH Transformation Matrix:
    Matrix([[           cos(q),           -sin(q),           0,             a],  
            [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
            [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],  
            [                0,                 0,           0,             1]])  

#### Individual Transformation Matrices
(Plug Modified DH Parameters into DH Transformation Matrix)

    T0_1  = Matrix([[cos(q1), -sin(q1), 0, 0], [sin(q1), cos(q1), 0, 0], [0, 0, 1, 0.75], [0, 0, 0, 1]])  
    T1_2  = Matrix([[cos(q2-pi/2), -sin(q2-pi/2), 0, 0.35], [0, 0, 1, 0], [-sin(q2-pi/2), -cos(q2-pi/2), 0, 0], [0, 0, 0, 1]])  
    T2_3  = Matrix([[cos(q3), -sin(q3), 0, 1.25], [sin(q3), cos(q3), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])  
    T3_6  = Matrix([[cos(q6), -sin(q6), 0, 0], [0, 0, 1, 0], [-sin(q6), -cos(q6), 0, 0], [0, 0, 0, 1]])  
    T6_EE = Matrix([[1, 0, 0, 0],[0, 1, 0, 0], [0, 0, 1, 0.303], [0, 0, 0, 1]])  

    T0_EE = T0_1 * T1_2 * T2_3 * T3_6 * T6_EE

#### Correction Needed to Account for Orientation Difference between Definition of Gripper Link in URDF vs. DH Convention
(Rotate about <strong>z</strong> axis 180 degrees, then <strong>y</strong> axis -90 degrees)

    R_z = Matrix([[cos(pi), -sin(pi), 0, 0], [sin(pi), cos(pi), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])  
    R_y = Matrix([[cos(-pi/2), 0, sin(-pi/2), 0],[0, 1, 0, 0], [-sin(-pi/2), 0, cos(-pi/2), 0], [0, 0, 0, 1]])  
    R_corr = R_z * R_y  

#### Total Homogeneous Transform Between Base_Link and Gripper_link with Orientation Correction Applied
    T_Total = T0_EE * R_corr  
___
##
<Strong>
**Inverse Kinematics
</strong>

<strong>
Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.
 
Based on the geometric Inverse Kinematics method described here, breakdown the IK problem into Position and Orientation problems. Derive the equations for individual joint angles. Your writeup must contain details about the steps you took to arrive at those equations. Add figures where necessary. If any given joint has multiple solutions, select the best solution and provide explanation about your choice (Hint: Observe the active robot workspace in this project and the fact that some joints have physical limits).
</strong>

Since the last three joints are revolute and their joint axes intersect at a single point, we have a case of spherical wrist with joint_5 being the common intersection point and hence the wrist center.  We can kinematically decouple the IK problem into Inverse Position and Inverse Orientation 



#### End-Effector position given by Px, Py, Pz
    EE = Matrix([[px], [py], [pz]])

#### End-Effector Orientation given by r, p, y (plug r, p y into Modified DH Transformation Matrix)
    ROT_x = Matrix([[1, 0, 0], [0, cos(r), -sin(r)], [0, sin(r), cos(r)]])  # ROLL
    ROT_y = Matrix([[cos(p), 0, sin(p)], 0, 1, 0], [ -sin(p), 0, cos(p)]])  # PITCH
    ROT_z = Matrix([[cos(y), -sin(y), 0],[ sin(y), cos(y), 0], [0, 0, 1]])  # YAW
    ROT_EE = ROT_z * ROT_y * ROT_x

#### Wrist Center Location
    WC = EE - (0.303) * ROT_EE[:,2]

Insert equations for individual joint angles.
![GitHub Logo](/images/logo.png)
___

<strong>
Project Implementation
 
Fill in the IK_server.py file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. A screenshot of the completed pick and place process is included.
 
IK_server.py must contain properly commented code. The robot must track the planned trajectory and successfully complete pick and place operation. Your writeup must include explanation for the code and a discussion on the results, and a screenshot of the completed pick and place process.
</strong>

Insert code explanations
![GitHub Logo](/images/logo.png)
Insert discussion of results
![GitHub Logo](/images/logo.png)
Insert screenshot of completed process
![GitHub Logo](/images/logo.png)
