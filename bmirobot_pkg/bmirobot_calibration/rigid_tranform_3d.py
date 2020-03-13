from numpy import *
from math import sqrt
import transforms3d.quaternions as quat
import transforms3d.affines as aff
import transforms3d.euler as eul
# Input: expects Nx3 matrix of points
# Returns R,t
# R = 3x3 rotation matrix
# t = 3x1 column vector
# B = R*A + t

def rigid_transform_3D(A, B):
    assert len(A) == len(B)

    N = A.shape[0]; # total points

    centroid_A = mean(A, axis=0)
    centroid_B = mean(B, axis=0)
    
    # centre the points
    AA = A - tile(centroid_A, (N, 1))
    BB = B - tile(centroid_B, (N, 1))

    # dot is matrix multiplication for array
    H = transpose(AA) * BB

    U, S, Vt = linalg.svd(H)

    R = Vt.T * U.T

    # special reflection case
    if linalg.det(R) < 0:
       print "Reflection detected"
       Vt[2,:] *= -1
       R = Vt.T * U.T

    t = -R*centroid_A.T + centroid_B.T

    #print t

    return R, t
'''
# Test with random data

# Random rotation and translation
R = mat(random.rand(3,3))
t = mat(random.rand(3,1))

# make R a proper rotation matrix, force orthonormal
U, S, Vt = linalg.svd(R)
R = U*Vt

# remove reflection
if linalg.det(R) < 0:
   Vt[2,:] *= -1
   R = U*Vt

# number of points
n = 10

A = mat(random.rand(n,3));
B = R*A.T + tile(t, (1, n))
B = B.T;
'''
#read in data
file = open('record.txt', 'r')
data = file.readlines()
n = int(data[0].split()[0])
A =[ [ 0 for i in range(3) ] for j in range(n) ]
B =[ [ 0 for i in range(3) ] for j in range(n) ]
for i in range(n):
   for j in range(3):
      A[i][j] = float(data[i*2+1].split()[j])
      B[i][j] = int(float(data[i*2+2].split()[j])*10000000)/10000000.0
A = mat(A)
B = mat(B)
#print A
#print B
# recover the transformation
ret_R, ret_t = rigid_transform_3D(A,B)
pi = 3.1415926
deg2rad=pi/180;
theta1 = [0,-90,90]
R1_world = eul.euler2mat(theta1[0]*deg2rad, theta1[1]*deg2rad, theta1[2]*deg2rad, 'rzyx')

A2 = (ret_R*A.T) + tile(ret_t, (1, n))
A2 = A2.T

# Find the error
err = A2 - B

err = multiply(err, err)
errs = power(err, 0.5)
print "error"
print errs
print ""

err = sum(err)
rmse = sqrt(err/n);
q = quat.mat2quat(ret_R*R1_world)

print "Points camera"
print A
print ""

print "transformated robot"
print A2
print ""

print "Points robot"
print B
print ""



#print "Rotation w x y z"
#print quat.mat2quat(ret_R)
#print "" 

#print "Translation"
#print ret_t
#print ""

print "RMSE(m):", rmse
print "If RMSE is near zero, the calibration is correct!"
#print("rosrun tf static_transform_publisher %f %f %f %f %f %f %f /base_link /camera_link 100" % (ret_t[0,0],ret_t[1,0],ret_t[2,0],q[1],q[2],q[3],q[0]))
print("%f %f %f %f %f %f %f /base_link /camera_link 100" % (ret_t[0,0],ret_t[1,0],ret_t[2,0],q[1],q[2],q[3],q[0]))

