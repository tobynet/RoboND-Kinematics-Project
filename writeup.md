## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**

1. ✅ Set up your ROS Workspace.
2. ✅ Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.
  → **This repository.**
3. ✅ Experiment with the forward_kinematics environment and get familiar with the robot.
  → **I tlearned it in the course and tried demo**.
4. ✅ Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
  → **I tried it.**
5. DOING ✍️🤔: Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png
[Rx]: ./misc_images/R_x(alpha_{i-1}).png
[Dx]: ./misc_images/D_x(a_{i-1}).png
[Rz]: ./misc_images/R_z(theta_i).png
[Dz]: ./misc_images/D_z(d_i).png
[Ti-eq00]: ./misc_images/Ti-eq00.png "T^{i-1}_{i}=R_x(\alpha_{i-1}) D_x(a_{i-1}) R_z(\theta_i) D_z(d_i)"

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Here is an example of how to include an image in your writeup.

* DONE: Try demo.

* TODO: Upload own image.

![alt text][image1]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

![Ti-eq00]

|     | ![Rx]      | ![Dx]  | ![Rz]    | ![Dz]
---   | ---        | ---    | ---      | ---
Links | alpha(i-1) | a(i-1) | theta(i) | d(i)
0->1  | 0          | 0      | θ₁       | 0.33+0.42=0.75
1->2  | -π/2       | 0.35   | θ₂-π/2   | 0
2->3  | 0          | 1.25   | θ₃       | 0
3->4  | -π/2       | -0.054 | θ₄       | 0.96+0.54=1.5
4->5  | π/2        | 0      | θ₅       | 0
5->6  | -π/2       | 0      | θ₆       | 0
6->EE | 0          | 0      | 0        | 0.193+0.11=0.303

> ⚠ **Swapped columns between `theta(i)` and `d_i`** to coincident with the `Ti` expression.

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles. 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


