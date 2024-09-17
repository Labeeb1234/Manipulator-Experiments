'''
Franka Emika D-H Parameters for a basic analysis (simulation only)

D-H Table


^
|(z-axis)
-----> (x-axis)

i     |alpha_i-1 | a_i-1 | theta_i  |  d_i |  
--------------------------------------------
1     |    0     |   0   |   q1     |  d1  |
2     |   3*pi/2 |   0   |   q2     |  0   |
3     |  pi/2    |   0   |   q3     |  d3  |
4     | pi/2     |  a3   |   q4     |  0   |
5     | 3*pi/2   |   0   |   q5     |  d5  |
6     | pi/2     |   0   |   q6     |  0   |
7     | pi/2     |  a6   |   q7     |  0   |
8(end)| 0        |   0   |   0      |  de  |


'''


from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
from math import cos, sin
from sympy import symbols, cos, sin, pi, Matrix, simplify



class FrankaEmika():
    def __init__(self, q, a, d):
        self.a = np.array(a)
        self.q = np.array(q)
        self.d = np.array(d)
        self.alpha = np.array([
            0, -np.pi/2, np.pi/2, np.pi/2, -np.pi/2, np.pi/2, np.pi/2, 0
        ])

    def arm_matrix(self, qi, ai_1, alphai_1, di):
        TF_arm = np.array([
            [cos(qi), -sin(qi), 0, ai_1],
            [sin(qi)*cos(alphai_1), cos(qi)*cos(alphai_1), -sin(alphai_1), -di*sin(alphai_1)],
            [sin(qi)*sin(alphai_1), cos(qi)*sin(alphai_1), cos(alphai_1), di*cos(alphai_1)],
            [0, 0, 0, 1]
        ]).reshape(4,4) 
        return TF_arm


    ################# Kinematics ################
    def forward_kinematics(self):
        # from 0->1
        T01 = self.arm_matrix(self.q[0], self.a[0], self.alpha[0], self.d[0])
        # from 1->2
        T12 = self.arm_matrix(self.q[1], self.a[1], self.alpha[1], self.d[1])
        # from 2->3
        T23 = self.arm_matrix(self.q[2], self.a[2], self.alpha[2], self.d[2])
        # from 3->4
        T34 = self.arm_matrix(self.q[3], self.a[3], self.alpha[3], self.d[3])
        # from 4->5
        T45 = self.arm_matrix(self.q[4], self.a[4], self.alpha[4], self.d[4])
        # from 5->6
        T56 = self.arm_matrix(self.q[5], self.a[5], self.alpha[5], self.d[5])
        # from 6->7
        T67 = self.arm_matrix(self.q[5], self.a[5], self.alpha[5], self.d[5])
        # from 7->8(end-effector)
        T78 = self.arm_matrix(self.q[5], self.a[5], self.alpha[5], self.d[5])
        
        T02 = np.dot(T01, T12)
        T03 = np.dot(T02, T23)
        T04 = np.dot(T03, T34)
        T05 = np.dot(T04, T45)
        T06 = np.dot(T05, T56)
        T07 = np.dot(T06, T67)
        T08 = np.dot(T07, T78) # Fwd Kinematics 

        # T08_simplified = simplify(T08)
        return T08


# Define symbolic variables for joint angles (q), link lengths (a), and link offsets (d)
q = [0, 0, 0, 0, 0, 0, 0, 0]
a = [0, 0, 0, 1.0, 0, 0, 1.0, 0]
d = [1.0, 0, 0.5, 0, 0.5, 0, 0, 0.5]

# Create an instance of the FrankaEmika class with symbolic inputs
robot = FrankaEmika(q,a,d)


fwd_kinematics_matrix = robot.forward_kinematics()


end_effector_pose = fwd_kinematics_matrix[:, 3]
print(f"{end_effector_pose}")
