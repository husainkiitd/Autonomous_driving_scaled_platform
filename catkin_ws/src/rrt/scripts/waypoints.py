import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d



def getCurvature(xRef, yRef):
    # Calculate gradient by the gradient of the X and Y vectors
    DX = np.gradient(xRef)
    D2X = np.gradient(DX)
    DY = np.gradient(yRef)
    D2Y = np.gradient(DY)
    curvature = (DX*D2Y - DY*D2X) / (DX**2 + DY**2)**(3/2)
    return curvature


R = 2
D = 3

x1 = np.array([0])
y1 = np.array([0])

th1 = np.arange(0, 180, 1)

x2 = D/2 + R*np.sin(np.deg2rad(th1))
y2 = R - R*np.cos(np.deg2rad(th1))

x3 = -D/2 - R*np.sin(np.deg2rad(th1))
y3 = R + R*np.cos(np.deg2rad(th1))

x4 = np.array([-0.10])
y4 = np.array([0])

x = np.concatenate((x1, x2, x3, x4))
y = np.concatenate((y1, y2, y3, y4))

refPose = np.vstack((x, y)).T
xRef = refPose[:,0]
yRef = refPose[:,1]

# calculate distance vector
distancematrix = np.zeros((len(refPose), len(refPose)))
for i in range(len(refPose)):
    for j in range(len(refPose)):
        distancematrix[i][j] = np.linalg.norm(refPose[i] - refPose[j])
distancesteps = np.zeros(len(refPose)-1)
for i in range(1, len(refPose)):
    distancesteps[i-1] = distancematrix[i][i-1]
totalDistance = np.sum(distancesteps) # Total distance travelled
distbp = np.cumsum(np.concatenate(([0], distancesteps))) # Distance for each waypoint
gradbp = np.linspace(0, distbp[-1], 1000) # Linearize distance

# linearize X and Y vectors based on distance
xRef2 = interp1d(distbp, xRef)(gradbp)
yRef2 = interp1d(distbp, yRef)(gradbp)
yRef2s = interp1d(gradbp, yRef2, kind='cubic')(gradbp) # smooth waypoints
xRef2s = interp1d(gradbp, xRef2, kind='cubic')(gradbp) # smooth waypoints

plt.plot(gradbp, xRef2s)
plt.xlabel('distance')
plt.ylabel('xRef2s')
plt.show()

plt.plot(gradbp, yRef2s)
plt.xlabel('distance')
plt.ylabel('yRef2s')
plt.show()

plt.plot(xRef2s, yRef2, 'b')
plt.xlabel('xRef2s')
plt.ylabel('yRef2s')
plt.show()

xRef = xRef2s
yRef = yRef2s
thetaRef = np.zeros((len(gradbp), 1))
for i in range(1, len(gradbp)):
    thetaRef[i, 0] = np.arctan2((yRef2[i]-yRef2[i-1]), (xRef2[i]-xRef2[i-1]))
    if (thetaRef[i, 0] < 0):
        thetaRef[i, 0] = np.pi + (np.pi + thetaRef[i, 0])
    else:
        pass
thetaRefs = np.interp(gradbp, gradbp, thetaRef.flatten(), period=totalDistance)
psi_o = thetaRefs[0]*(np.pi/180)

plt.plot(gradbp, thetaRefs)
plt.xlabel('distance')
plt.ylabel('thetaRefs')
plt.show()


curvature = getCurvature(xRef2, yRef2)
gradbp = np.linspace(0, distbp[-1], 1000)
plt.plot(gradbp, curvature)
plt.xlabel('distance')
plt.ylabel('curvature')
plt.show()

path = np.column_stack((xRef2s, yRef2s, thetaRefs)).T

# Assuming path is a 2D NumPy array
path_shape = path.shape
print(path_shape)  # prints the shape of the path array

# To get the number of rows or the number of waypoints, you can access the first dimension of the shape tuple
num_waypoints = path_shape[0]
print(num_waypoints)  # prints the number of waypoints in the path

path_length = len(path)
print(path_length)  # prints the length of the path list