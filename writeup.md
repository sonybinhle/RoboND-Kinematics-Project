## Project: Kinematics Pick & Place

[//]: # (Image References)

[joints-links]: ./misc_images/joints-links.png
[calculate-DH]: ./misc_images/calculate-DH.png
[transform-formula]: ./misc_images/transform-formula.png
[wc]: ./misc_images/wc.png
[theta23]: ./misc_images/theta23.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

After running the demo and viewing the geometry shape of Kuka Arm, we are able to define the joints and links as the image below.
There are 6 links and 6 joints, the joint axes of (2, 3, 5) are parallel and (4, 6) are coincident.


![alt text][joints-links]

Next, we can define the position and direction of z-axes and x-axes, it will affect the way DH parameters calculated.
So usually, in order to facilitate the calculation, we could try to maximize the number of coincident, intersected X axes. 

![alt text][calculate-DH]

From the picture above, we can now calculate DH parameters. 

The twist angle(alpha) is the easiest to calculated, for example, the angle between Z0 and Z1 is 0 because they are parallel.

The link length(ai) is calculated based on URDF file, for example:
a1 = joint2(X) - joint1(X) = 0.35 - 0 = 0.35

The link offset(di) is also calculated based on URDF file, for example:
d1 = joint1(Z) + joint2(Z) = 0.75

The joint angle(theta) will be dynamic parameters based on time, theta2 is special because there is pi/2 offset.

```xml
<joint name="joint_1" type="revolute">
    <origin xyz="0 0 0.33" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="link_1"/>
    <axis xyz="0 0 1"/>
    <limit lower="${-185*deg}" upper="${185*deg}" effort="300" velocity="${123*deg}"/>
</joint>
```
```xml
<joint name="joint_2" type="revolute">
    <origin xyz="0.35 0 0.42" rpy="0 0 0"/>
    <parent link="link_1"/>
    <child link="link_2"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-45*deg}" upper="${85*deg}" effort="300" velocity="${115*deg}"/>
</joint>
```

After calculation, the final result of DH table will be:

Links | alpha(i-1) | a(i-1)  | d(i-1) | theta(i)
---   | ---------- | ------- | ------ | ---
0->1  | 0          | 0       | 0.75   | q1
1->2  | - pi/2     | 0.35    | 0      | -pi/2 + q2
2->3  | 0          | 1.25    | 0      | q3
3->4  | - pi/2     | - 0.054 | 1.5    | q4
4->5  | pi/2       | 0       | 0      | q5
5->6  | - pi/2     | 0       | 0      | q6
6->EE | 0          | 0       | 0.303  | 0

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

We have the general formula of transform between adjacent links:

![alt text][transform-formula]

In python the transform matrix will be in the form of:

```python
def getTfMatrix(alpha, a, d, q):
    return Matrix([[             cos(q),           -sin(q),            0,              a],
                   [  sin(q)*cos(alpha), cos(q)*cos(alpha),  -sin(alpha),  -sin(alpha)*d],
                   [  sin(q)*sin(alpha), cos(q)*sin(alpha),   cos(alpha),   cos(alpha)*d],
                   [                  0,                 0,            0,              1]])
```

We then could calculate the transformation matrix from base link to gripper link
```python
    T0_1 = getTfMatrix(alpha0, a0, d1, q1).subs(DH_Params)
    T1_2 = getTfMatrix(alpha1, a1, d2, q2).subs(DH_Params)
    T2_3 = getTfMatrix(alpha2, a2, d3, q3).subs(DH_Params)
    T3_4 = getTfMatrix(alpha3, a3, d4, q4).subs(DH_Params)
    T4_5 = getTfMatrix(alpha4, a4, d5, q5).subs(DH_Params)
    T5_6 = getTfMatrix(alpha5, a5, d6, q6).subs(DH_Params)
    T6_E = getTfMatrix(alpha6, a6, d7, q7).subs(DH_Params)

    T0_E = simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_E)
```
Next, we need to calculate the rotation matrix between end-effector's original pose and next pose.
```python
    # Define rotation matrix along with x, y, z axes
    R_x = Matrix([[1, 0, 0],
                  [0, cos(roll), -sin(roll)],
                  [0, sin(roll), cos(roll)]])
    R_y = Matrix([[cos(pitch), 0, sin(pitch)],
                  [0, 1, 0],
                  [-sin(pitch), 0, cos(pitch)]])
    R_z = Matrix([[cos(yaw), -sin(yaw), 0],
                  [sin(yaw), cos(yaw), 0],
                  [0, 0, 1]])
    R_E = R_z * R_y * R_x

    # Rotate z 180 degree and then rotate y -90 degree in order to align the different in orientation of gripper frame in URDF and DH parameters
    R_E = R_E * R_z.subs(y, pi) * R_y.subs(p, -pi / 2)
```

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

By using decoupling Inverse Kinematics problem, it is possible to solve two independent problems, the position of wrist center and the orientation of the end effector.
The desired position of WC will be calculated by the below formula:
![alt text][wc]
```python
    # Desired position of end-effector
    EE = Matrix([[px], 
                 [py], 
                 [pz]])
                
    # d7 = 0.303, R_E[:, 2] represent translation part only
    WC = EE - (0.303) * R_E[:, 2] 
```
To simplify problem, we could project WC to ground and calculate theta1 based on the projection:
```python
   theta1 = atan2(WC[1], WC[0])
```
Next, we can calculate theta2, theta3 based on the projection to XZ axis
![alt text][theta23]
```python
    # Distance between joint2 vs WC in XO direction
    joint2ToWcX = sqrt(WC[0] ** 2 + WC[1] ** 2) - 0.35 # a1 = 0.35
    
    # Distance between joint2 vs WC in ZO direction
    joint2ToWcY = WC[2] - 0.75 # d1 = 0.75
    
    A = 1.50097168528 # sqrt(a3 ** 2 + d4 ** 2) = sqrt(-0.054 ** 2 + 1.5 ** 2)
    B = sqrt(joint2ToWcX ** 2 + joint2ToWcY ** 2)
    C = 1.25 # a2
```


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


