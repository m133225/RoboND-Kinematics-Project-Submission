from sympy import *
from time import time
from mpmath import radians
import tf

def getJointHomogenousTransform(alpha, a, d, angle):
  hTMatrix = Matrix([[cos(angle), -sin(angle), 0, a],
                     [sin(angle)*cos(alpha), cos(angle)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                     [sin(angle)*sin(alpha), cos(angle)*sin(alpha), cos(alpha), cos(alpha)*d],
                     [0, 0, 0, 1]])
  return hTMatrix


def rot_x(q):
    R_x = Matrix([[1, 0, 0],
                  [0, cos(q), -sin(q)],
                  [0, sin(q), cos(q)]])
    
    return R_x
    
def rot_y(q):              
    R_y = Matrix([[cos(q), 0, sin(q)],
                  [0, 1, 0],
                  [-sin(q), 0, cos(q)]])
    
    return R_y

def rot_z(q):    
    R_z = Matrix([[cos(q), -sin(q), 0],
                  [sin(q), cos(q), 0],
                  [0, 0, 1]])
    
    return R_z

alpha, angle = symbols('alpha angle')
a, d = symbols('a d')
hTMatrix = getJointHomogenousTransform(alpha, a, d, angle)
theta1, theta2, theta3, theta4, theta5, theta6 = symbols("theta1:7")

# Fill this line with the DH parameters
alpha_v, a_v, d_v, angle_v = 0*pi/180, 0, 0.33 + 0.42, theta1
resultingMatrix_1 = hTMatrix.subs([(alpha, alpha_v),(a, a_v),(d, d_v),(angle, angle_v)])

# Fill this line with the DH parameters
alpha_v, a_v, d_v, angle_v = 90*pi/180, 0.35, 0, theta2 + 90*pi/180
resultingMatrix_2 = hTMatrix.subs([(alpha, alpha_v),(a, a_v),(d, d_v),(angle, angle_v)])

# Fill this line with the DH parameters
alpha_v, a_v, d_v, angle_v = 0*pi/180, 1.25, 0, theta3 + 180*pi/180
resultingMatrix_3 = hTMatrix.subs([(alpha, alpha_v),(a, a_v),(d, d_v),(angle, angle_v)])

# Fill this line with the DH parameters
alpha_v, a_v, d_v, angle_v = -90*pi/180, 0.054, 0.96+0.54, theta4 + 180*pi/180
resultingMatrix_4 = hTMatrix.subs([(alpha, alpha_v),(a, a_v),(d, d_v),(angle, angle_v)])

# Fill this line with the DH parameters
alpha_v, a_v, d_v, angle_v = -90*pi/180, 0, 0, theta5
resultingMatrix_5 = hTMatrix.subs([(alpha, alpha_v),(a, a_v),(d, d_v),(angle, angle_v)])

# Fill this line with the DH parameters
alpha_v, a_v, d_v, angle_v = 90*pi/180, 0, 0, theta6
resultingMatrix_6 = hTMatrix.subs([(alpha, alpha_v),(a, a_v),(d, d_v),(angle, angle_v)])

# Fill this line with the DH parameters
alpha_v, a_v, d_v, angle_v = 0*pi/180, 0, 0.193+0.11, 0
resultingMatrix_7 = hTMatrix.subs([(alpha, alpha_v),(a, a_v),(d, d_v),(angle, angle_v)])

#angles = [[theta1, 0], [theta2, 0], [theta3, 0], [theta4, 0], [theta5, 0], [theta6, 0]]
#angles = [[theta1, 1.77313925102116], [theta2, 0.498909584231360], [theta3, 2.52119797775612], [theta4, -0.121456566847005], [theta5, 0.000863607580256405], [theta6, -0.203293037854362]]
result = resultingMatrix_1*resultingMatrix_2*resultingMatrix_3*resultingMatrix_4*resultingMatrix_5*resultingMatrix_6*resultingMatrix_7
print resultingMatrix_1
print resultingMatrix_2
print resultingMatrix_3
print resultingMatrix_4
print resultingMatrix_5
print resultingMatrix_6
print resultingMatrix_7

px, py, pz = symbols('px py pz')
roll, pitch, yaw = symbols('roll pitch yaw')
rotationMatrix = rot_z(yaw) * rot_y(pitch) * rot_x(roll) * rot_y(pi/2) * rot_z(pi)
generalisedHomogenousTransform = rotationMatrix.row_join(Matrix([px, py, pz])).col_join(Matrix([[0, 0, 0, 1]]))
print generalisedHomogenousTransform

