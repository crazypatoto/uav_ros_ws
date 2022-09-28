from cProfile import label
from cmath import pi
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import rosbag
import numpy as np
import math
from geometry_msgs.msg import PoseStamped

GTH_POSE_TOPIC = '/ground_truth/pose'
REF_POSE_TOPIC = '/reference/pose'
OFFSET = 100/180*math.pi
OMEGA = 2.0


plt.rcParams['text.usetex'] = True
plt.rcParams["font.family"] = "Times New Roman"

# bag = rosbag.Bag('/home/user/Documents/thesis/lemniscate_2.bag')
bag = rosbag.Bag('/home/user/Documents/thesis/circle_wind_2.bag')

gth_poses = []
ref_poses = []

for topic, msg, t in bag.read_messages(topics=[GTH_POSE_TOPIC, REF_POSE_TOPIC]):
    if topic == GTH_POSE_TOPIC:
        gth_poses.append(msg)
    elif topic == REF_POSE_TOPIC:
        ref_poses.append(msg)

# startPoint = np.array([gth_poses[0].pose.position.x,gth_poses[0].pose.position.y,gth_poses[0].pose.position.z]) 
# startPoint = ((startPoint * SCALE).astype(int) /SCALE).astype(float)
# print(startPoint)
# index = -1
# for i in range(1,len(gth_poses)):
#     point = np.array([gth_poses[i].pose.position.x,gth_poses[i].pose.position.y,gth_poses[i].pose.position.z]) 
#     point = ((point * SCALE).astype(int) /SCALE).astype(float)
#     print(point)
#     if np.array_equal(point,startPoint):
#         index = i
#         break
# print(index)

start_index = int(OFFSET * 100 / OMEGA)
end_index = start_index + int(2 * math.pi * 100 / OMEGA) - 5


gth_poses = gth_poses[start_index:end_index]
ref_poses = ref_poses[start_index:end_index]

gth_xs = np.array([msg.pose.position.x for msg in gth_poses])
gth_ys = np.array([msg.pose.position.y for msg in gth_poses])
gth_zs = np.array([msg.pose.position.z for msg in gth_poses])

ref_xs = np.array([msg.pose.position.x for msg in ref_poses])
ref_ys = np.array([msg.pose.position.y for msg in ref_poses])
ref_zs = np.array([msg.pose.position.z for msg in ref_poses])

mse = ((gth_xs - ref_xs)**2+(gth_ys - ref_ys)**2+(gth_zs - ref_zs)**2).mean()
rmse = math.sqrt(mse)
print('MSE:',end='')
print(mse)
print('RMSE:',end='')
print(rmse)

errors = np.sqrt((gth_xs-ref_xs)**2 + (gth_ys-ref_ys)**2 + (gth_zs-ref_zs)**2)
print('MAE:',end='')
print(errors.mean())
print('STD:',end='')
print(errors.std())

fig = plt.figure(figsize=(6, 6))
ax = fig.gca(projection='3d')
ax.plot(ref_xs, ref_ys, ref_zs,'--',label='reference trajectroy')
ax.plot(gth_xs, gth_ys, gth_zs,label='actual trajectroy')
# ax.set_title(r'$\textrm{Lemniscate Trajectory,}\ \omega$=%.1f' % OMEGA, fontsize=14)
ax.set_title(r'$\textrm{Circle Trajectory,}\ \omega$=%.1f' % OMEGA, fontsize=14)
ax.set_xlabel(r'$\mathit{x}\textrm{-coordinate}\ (\mathrm{m})$', fontsize=14)
ax.set_ylabel(r'$\mathit{y}\textrm{-coordinate}\ (\mathrm{m})$', fontsize=14)
ax.set_zlabel(r'$\mathit{z}\textrm{-coordinate}\ (\mathrm{m})$', fontsize=14)
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
# ax.set_zlim([1.5, 2.5])
ax.set_zlim([1, 3])
ax.legend(bbox_to_anchor=(0,0.2,1,0.2), loc="lower left")




plt.show()
# for topic, msg, t in bag.read_messages(topics=['/ground_truth/pose', '/reference/pose']):
#     print(msg)

bag.close()
