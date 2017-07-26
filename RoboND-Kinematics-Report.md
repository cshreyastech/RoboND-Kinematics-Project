## Project: Kinematics Pick & Place
Goal of this project is to pick and place objects from a shell and place it to a bin. This could be done by calculating DH parameters and further apply them in calculating Forwared and inverse Kinematics. This project uses Ros and Move-it to simulate these actions using 6 joint Kuka arm KR 210.

---
[//]: # (Image References)

[DH]: ./support-docs/images/01-DH.jpg
[WCOrientation]: ./support-docs/images/02-WC-Orientation.jpg
[URDFvalues]: ./support-docs/images/03-URDFvalues.jpg
[WCForwardKinematics]: ./support-docs/images/04-WC-ForwardKinematics.jpg
[teeta1]: ./support-docs/images/05-teeta1.jpg
[teeta21]: ./support-docs/images/06-teeta2.1.jpg
[teeta22]: ./support-docs/images/06-teeta2.2.jpg
[teeta3]: ./support-docs/images/07-teeta3.jpg
[R361]: ./support-docs/images/08-R3_6.1.jpg
[R362]: ./support-docs/images/08-R3_6.2.jpg
[R363]: ./support-docs/images/08-R3_6.3.jpg
[R364]: ./support-docs/images/08-R3_6.4.jpg
[R365]: ./support-docs/images/08-R3_6.5.jpg
[R366]: ./support-docs/images/08-R3_6.6.jpg
[theta4]: ./support-docs/images/09-theta4.jpg
[theta5]: ./support-docs/images/10-theta5.jpg
[theta6]: ./support-docs/images/11-theta6.jpg

**Summary of steps to complete the project:**  

1. DH parameters
2. Transformation Matrix with respespect to previous joint
3. Transformation Matrix with respespect to base frame
4. Calculate Wrist Center
5. Calculate theta1 through theta3
6. Calculate theta4 through theta6
7. Conclusion

---
### Detail Explanations
#### 1. Calculate DH parameters
DH parameters are calculated using the values provided in urdf file of kuka arm. 
Below is the orientation of the the axis for the the various joints
![alt text][DH]

Values from URDF file of the arm
![alt text][WCOrientation]

DH parameters values from derived from URDF file

Joint	| alpha	|	a	|  d	|  q
--- 	| --- 	| --- 	| ---	| ---
1 		|   0 	|  0	| 0.75	|
2 		| -pi/2 |  0.35 | 0		| -pi/2
3 		|   0 	|  1.25	| 0		|
4 		| -pi/2 |-0.054 | 1.5	|
5 		| pi/2 	|   0  	| 0		|
6 		| -pi/2 |   0  	| 0		|
7 		|   0  	|   0  	| 0.303	| 0

---
#### 2. Transformation Matrix with respespect to previous joint
T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0 ],
               [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1 ],
               [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1 ],
               [                   0,                   0,            0,               1 ]])
T0_1 = T0_1.subs(s)

Similary Transformation Matrix for other joints were created
T0_1
T1_2
T2_3
T3_4
T4_5
T5_6
T6_G

---
#### 3. Transformation Matrix with respespect to base frame
As a need a for common frame of reference for further calculations, Transformation matrix with respect to base frame was calculated
T0_2 = T0_1 * T1_2
T0_3 = T0_2 * T2_3
T0_4 = T0_3 * T3_4
T0_5 = T0_4 * T4_5
T0_6 = T0_5 * T5_6
T0_G = T0_6 * T6_G

The gripper orientation is different to that of base frame. To account for this the correction matrix is multipled with the end effector roll, pitch and yaw values.
R_z = rot_z(pi)
R_y = rot_y(-pi/2)
R_corr = R_z * R_y

R_roll = rot_x(roll)
R_pitch = rot_y(pitch)
R_yaw = rot_z(yaw)
R0_6 = R_roll * R_pitch * R_yaw * R_corr
----

#### 4. Calculate Wrist Center
As seen from the Arm orientation diagram, Wrist Center is a displacement of d7 from the end effector
P_EE = Matrix([px, py, pz])
WC = P_EE - R0_6 * Matrix([0, 0, s[d7]])

---
#### 5. Calculate theta1 through theta3
As discribed in the lessons these angles will have to calculated using geometry.
##### theta1 calculations
![alt text][teeta1]
theta1 = atan2(WC[1], WC[0]).evalf()

##### theta2 calculations
![alt text][teeta21]
![alt text][teeta22]
s1 = sqrt(WC[0]**2 + WC[1]**2) - s[a1]
s2 = WC[2] - s[d1]
s3 = sqrt(s1**2 + s2**2)
s4 = sqrt(s[a3]**2 + s[d4]**2)

beeta1 = atan2(s2, s1)

cos_beeta2 = (s[a2]**2 + s3**2 - s4**2)/(2*s[a2]*s3)
sin_beeta2 = sqrt(1-cos_beeta2**2)
beeta2 = atan2(sin_beeta2, cos_beeta2)

theta2 = (pi/2 - beeta1 - beeta2).evalf()

##### theta3 calculations
![alt text][teeta3]
cos_beeta3 = (s[a2]**2 + s4**2 - s3**2)/(2*s[a2]*s4)
sin_beeta3 = sqrt(1 - cos_beeta3**2)
beeta3 = atan2(sin_beeta3, cos_beeta3)

beeta4 = atan2(-s[a3], s[d4])

theta3 = (pi/2 - beeta4 - beeta3).evalf()
theta1_3_end = datetime.datetime.now()
theta1_3_delta = theta1_3_end - theta1_3_start

----
#### 6. Calculate theta4 through theta6
From the leassons, theta4 through theta were calculated using R3_6
R3_6 = inv(R0_3) * Rrpy

##### R3_6 Calculations
R3_6 = R3_4 * R4_5 * R5_6
![alt text][R361]
![alt text][R362]
![alt text][R363]
![alt text][R364]
![alt text][R365]
![alt text][R366]

R0_3 is the rotation part of T0_3
R0_3 = T0_3[0:3, 0:3]
R0_3_inv = R0_3**(-1)
R3_6 = R0_3_inv * R0_6
R3_6_num = R3_6.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

##### theta4 calculations
![alt text][theta4]
theta4 = atan2(R3_6_num[2, 2], -R3_6_num[0, 2]).evalf()

##### theta5 calculations
![alt text][theta5]
theta5 = atan2(sqrt(1 - R3_6_num[1, 2]**2), R3_6_num[1, 2]).evalf()

##### theta6 calculations
![alt text][theta6]
theta6 = atan2(-R3_6_num[1, 1], R3_6_num[1, 0]).evalf()

----
#### 7. Conclusion
