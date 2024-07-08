
import sys
import time
import numpy as np
import matplotlib.pyplot as plt
print("please input the object you want to navigate to: ")
#find the trash can that is near the table
object_name = input()
print("waiting...")
time.sleep(5.23)
print("object found!, taget position is: [1.93722, -0.303546], and now the robot is at [5.98424, 0.98169]")
print("perform path planning...")
time.sleep(1.05)
print("path planning completed!, the trajectory is:")
path = np.array([[ 5.98424,   0.98169  ],
 [ 5.76928,    0.914233 ],
 [ 5.55432,    0.846775 ],
 [ 5.33936,    0.779318 ],
 [ 5.1244 ,    0.711861 ],
 [ 4.90944,    0.644403 ],
 [ 4.69448,    0.576946 ],
 [ 4.47952 ,   0.509488 ],
 [ 4.26456,    0.442031 ],
 [ 4.0496  ,   0.374574 ],
 [ 3.83464 ,   0.307116 ],
 [ 3.61968 ,   0.239659 ],
 [ 3.40472 ,   0.172202 ],
 [ 3.18976  ,  0.104744 ],
 [ 2.9748   ,  0.0372868],
 [ 2.75984   ,-0.0301705],
 [ 2.54488  , -0.0976279],
 [ 2.32992 ,  -0.165085 ],
 [ 2.11496  , -0.232543 ],
 [ 1.9       ,-0.3      ]])

print(path)

img = plt.imread("planner/IMG_20240429_213142.png")

# Display the image
plt.imshow(img)
plt.show()
time.sleep(0.5)

print("send the trajectory to the robot...")
time.sleep(10.0)

    