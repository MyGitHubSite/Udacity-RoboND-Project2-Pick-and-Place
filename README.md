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

Hand-drawn labeling of: joints, joint axes, links, positive and <strong>x</strong> axes (common normals between <strong>z<sub>i-1</sub></strong> and <strong>z<sub>i</sub></strong>), and reference frame origins (intersection of <strong>x<sub>i</sub></strong> and <strong>z<sub>i</sub></strong>) on graph, non-zero link lengths (<strong>a</strong> values), link offsets (<strong>d</strong> values), <strong>alpha</strong> (<strong>z</strong> twist angles), and <strong>q</strong> values.  

![GitHub Logo](/images/AllLabels.jpg)  

<strong>Modified DH Parameter Table</strong>

**i** | **alpha<sub>i-1</sub>** | **a<sub>i-1</sub>** | **d<sub>i</sub>** | **theta (q<sub>i</sub>)**
:--: | :-----: | :-: | :-: | :-----:
1 | 0 | 0 | 0.75 | q1
2 | -pi/2 | 0.35 | 0 | q2-pi/2
3 | 0 | 1.25 | 0 | q3
4 | -pi/2 | -0.054 | 1.50 | q4
5 | pi/2 | 0 | 0 | q5
6 | -pi/2 | 0 | 0 | q6
G | 0 | 0 | 0.303 |  q7

<strong>a<sub>i-1</sub></strong> = distance from <strong>z<sub>i-1</sub></strong> and <strong>z<sub>i</sub></strong> measured along <strong>x<sub>i-1</sub></strong> axis  
<strong>d<sub>i</sub></strong> = signed distance between <strong>x<sub>i-1</sub></strong> and <strong>x<sub>i</sub></strong> measured along the <strong>z<sub>i</sub></strong> axis  
<strong>alpha<sub>i</sub></strong> = angle between <strong>z<sub>i-1</sub></strong> and <strong>z<sub>i</sub></strong> measured along the <strong>x<sub>i-1</sub></strong> axis according to the right hand rule  
<strong>theta<sub>i</sub></strong> = angle between <strong>x<sub>i-1</sub></strong> and <strong>x<sub>i</sub></strong> measured about <strong>z<sub>i</sub></strong> axis using right hand rule  
for <strong>i</strong>=2 there is a -90 degree constant offset between <strong>x<sub>1</sub></strong> and <strong>x<sub>2</sub></strong>.

Note: <strong>a</strong>, <strong>d</strong>, and <strong>alpha</strong> values were obtained from the kr210.urdf.xacro file.  
___

The Modified DH parameter table above was plugged into the DH Transformation Matrix below to derive individual link to link transform matrices.

#### DH Transformation Matrix:
    Matrix([[           cos(q),           -sin(q),           0,             a],  
            [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
            [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],  
            [                0,                 0,           0,             1]])  

#### Individual Transformation Matrices
(Plug Modified DH Parameters into DH Transformation Matrix)

    T0_1 = Matrix([[cos(q1), -sin(q1), 0, 0], [sin(q1), cos(q1), 0, 0], [0, 0, 1, 0.75], [0, 0, 0, 1]])  
    T1_2 = Matrix([[cos(q2 - 0.5*pi), -sin(q2 - 0.5*pi), 0, 0.35], [0, 0, 1, 0], [-sin(q2 - 0.5*pi), -cos(q2 - 0.5*pi), 0, 0], [0, 0, 0, 1]])  
    T2_3 = Matrix([[cos(q3), -sin(q3), 0, 1.25], [sin(q3), cos(q3), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])  
    T3_4 = Matrix([[cos(q4), -sin(q4), 0, -0.054], [0, 0, 1, 1.5], [-sin(q4), -cos(q4), 0, 0], [0, 0, 0, 1]])  
    T4_5 = Matrix([[cos(q5), -sin(q5), 0, 0], [0, 0, -1, 0], [sin(q5), cos(q5), 0, 0], [0, 0, 0, 1]])  
    T5_6 = Matrix([[cos(q6), -sin(q6), 0, 0], [0, 0, 1, 0], [-sin(q6), -cos(q6), 0, 0], [0, 0, 0, 1]])  
    T6_EE = Matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.303], [0, 0, 0, 1]])  

#### Total Homogeneous Transform Between Base_Link and Gripper_link

    T0_EE = T0_1 * T1_2 * T2_3 * T3_6 * T6_EE

#### Correction Needed to Account for Orientation Difference between Definition of Gripper Link in URDF vs. DH Convention
(Rotate about <strong>z</strong> axis 180 degrees, then <strong>y</strong> axis -90 degrees)

    ROT_z = Matrix([[cos(pi), -sin(pi), 0, 0], [sin(pi), cos(pi), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])  
    ROT_y = Matrix([[cos(-pi/2), 0, sin(-pi/2), 0],[0, 1, 0, 0], [-sin(-pi/2), 0, cos(-pi/2), 0], [0, 0, 0, 1]])  
    ROT_corr = ROT_z * ROT_y  
___
###
**Inverse Kinematics**  

Since the last three joints are revolute and their joint axes intersect at a single point, we have a spherical wrist with Joint 5 being the common intersection point and hence the wrist center.  We can kinematically decouple the IK problem into Inverse Position and Inverse Orientation 

The position of the wrist center is governed by the first three joints while the last three joints orient the end-effector as needed.  

![Spherical Wrist](/images/Spherical%20Wrist.jpg) 

#### End-Effector Orientation given by r, p, y (plug r, p y into Modified DH Transformation Matrix)
    ROT_x = Matrix([[1, 0, 0], [0, cos(r), -sin(r)], [0, sin(r), cos(r)]])  # ROLL
    ROT_y = Matrix([[cos(p), 0, sin(p)], 0, 1, 0], [ -sin(p), 0, cos(p)]])  # PITCH
    ROT_z = Matrix([[cos(y), -sin(y), 0],[ sin(y), cos(y), 0], [0, 0, 1]])  # YAW
    
#### If we include the correction needed to account for the orientation difference between the definition of the gripper link in URDF vs. DH Convention, the total rotation becomes:

    ROT_EE = ROT_z * ROT_y * ROT_x * ROT_corr

#### The Wrist Center (WC) location is given by:

![Wrist Center Location](/images/WristCenter.jpg) 

Therefore, if:  

#### End-Effector position given by Px, Py, Pz
    EE = Matrix([[px], [py], [pz]])

then:  

    WC = EE - (0.303) * ROT_EE[:,2], where 0.303 comes from the Modified DH Parameter Table for dG
___
#### Calcuation of individual joint angles:

**Theta 1:**  

![Theta1](/images/Theta1.jpg)

The point <strong>z<sub>c</sub></strong> could be considered to be the wrist center of a spherical wrist. To find theta1, project <strong>z<sub>c</sub></strong> onto the ground plane.  Thus, 

    theta1 = atan2(yc, xc)  [Note: per the kr210.urdf.xacro file the angle is limited to +/- 185 degrees.]

**Theta 2 and 3:**  

![Theta2 Theta3](/images/Theta2Theta3.jpg) 

We can use the Law of Cosines to calculate theta 2 and theta 3:

    Sides:
       A = 1.501
       B = sqrt(pow((sqrt(WC[0]*WC[0] + WC[1]*WC[1]) - 0.35), 2) + pow((WC[2] - 0.75), 2))
       C = 1.25
       
    Angles:
       a = acos((B*B + C*C - A*A) / (2*B*C))
       b = acos((A*A + C*C - B*B) / (2*A*C))
       c = acos((A*A + B*B - C*C) / (2*A*B))

       theta2 = (pi/2 - a - atan2(WC[2]-0.75, sqrt(WC[0]*WC[0]+WC[1]*WC[1])-0.35) [limited between -45 and 85 degrees]
       theta3 = (pi/2 - b + 0.036) [limited between -210 and 65 degrees]

**Theta 4, 5, and 6:**  

#### Extract rotation matrix R0_3 from transformation matrix T0_3 then substitute angles q1-3  
        R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
        R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3:theta3})
        R3_6 = R0_3.inv(method="LU") * ROT_EE

#### Euler angles from rotation matrix  
        theta4 = atan2(R3_6[2,2], -R3_6[0,2]), limited between -350 and 350 degrees
        theta5 = atan2(sqrt(R3_6[0, 2]*R3_6[0, 2] + R3_6[2, 2]*R3_6[2, 2]), R3_6[1, 2]), limited between -125 and 125 degrees
        theta6 = atan2(-R3_6[1,1],R3_6[1,0]), limited between -350 and 350 degrees
___

<strong>
Project Implementation:    
</strong>

I was able to get the VM Ware environment set up on my local machine.  First off, I ran the demo and observed the behavior of the robot.  Also, I fiddled around with the parameters in Rviz to visualize some extra movements.

For the test, I streamlined my IK_server.py code by creating functions for the DH transforms and the rotational matrices.  I grapped the DH parameters from the urdf file and plugged them into the code.  I then worked out all the other pieces for the project and split  them into a forward kinematics section and an inverse kinetamtics section. My forward kinematics section put all code that did not need pose information in order to reduce computations in the inverse kinematics loop.  The individual transforms between L3 and link G were not needed so I took out those calculations.  

Using Rviz and Gazebo I was able to test out my code.  Fortunately, I was able to get 9/10 cylinders in the bucket.  Here is a screenshot after 10 runs:  

![Final Run](/images/FinalRun.jpg)  

I did not have any problems with calcualtions taking to long or the simuations running slowly, perhaps because I took advantage of the helpful coding tips from the lecture.  I did find that the robot seemed to move very inefficiently  (i.e., many of the moves needed to get to the target and drop zone seemed wasteful).  However, the robot did successfully complete the pick and place the vast majority of the time.







