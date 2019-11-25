import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.axes3d import Axes3D
import numpy as np
# f = open("../build/test/trajectory.txt",'r')
# f = open("../build/test/looped_t.txt",'r')
# f = open("../build/test/gt_positions.txt",'r')
# f = open("../build/test/ba.txt",'r')
char = f.readline()
x=[]
y=[]
z=[]
while char:
    print(char)
    tmp = list(filter(None,char.replace("[",'').replace("]'",'').split(" ")))
    z.append(eval(tmp[2]))
    y.append(eval(tmp[1]))
    x.append(eval(tmp[0]))
    char = f.readline()
f.close()
fig = plt.figure(figsize=(8,6))

axes3d = Axes3D(fig)

axes3d.plot(x,y,z)

plt.xlabel('X',size = 30)
plt.ylabel('Y',size = 30)
axes3d.set_zlabel('Z',color = 'r',size=30)

# axes3d.set_yticks([-1,0,1])
# axes3d.set_yticklabels(['min',0,'max'])

plt.show()
# plt.plot(z, x, 'r')
# plt.show()
