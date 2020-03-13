import numpy as np
from math import sqrt
import transforms3d.quaternions as quat
import transforms3d.affines as aff
import transforms3d.euler as eul

#q = [0, 1, 0, 0] 
#R = quat.quat2mat(q)
pi = 3.1415926
R = eul.euler2mat(0.35*pi, 0, -0.2*pi, 'sxyz')
T = [0.05, -0.04, 0]
A = aff.compose(T, R, np.ones(3))

p1 = np.array([-0.021, -0.021, 0, 1]).reshape((4,1))
p2 = np.array([0.021, -0.021, 0, 1]).reshape((4,1))
p3 = np.array([0.021, 0.021, 0, 1]).reshape((4,1))
p4 = np.array([-0.021, 0.021, 0, 1]).reshape((4,1))

tvec = np.dot(A, p1) 

tvec[0,0] = -1*tvec[0,0]
temp = -1*tvec[1,0]
tvec[1,0] = -1*tvec[2,0]
tvec[2,0] = temp
#tvec[1,0] = tvec[1,0]

ori = np.array([0.0425, -0.025, -0.0014, 1]).reshape((4,1))

tvec = tvec + ori
tvec = tvec*1000


print tvec





