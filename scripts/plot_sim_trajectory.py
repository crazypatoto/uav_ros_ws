from cProfile import label
from cmath import pi
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import rosbag
import numpy as np
import math
from geometry_msgs.msg import PoseStamped

UAV0_POSE_TOPIC = '/uav0/uav_controller/pose'
UAV1_POSE_TOPIC = '/uav1/uav_controller/pose'
UAV2_POSE_TOPIC = '/uav2/uav_controller/pose'
UAV3_POSE_TOPIC = '/uav3/uav_controller/pose'
UAV4_POSE_TOPIC = '/uav4/uav_controller/pose'
UAV5_POSE_TOPIC = '/uav5/uav_controller/pose'
UAV6_POSE_TOPIC = '/uav6/uav_controller/pose'
UAV7_POSE_TOPIC = '/uav7/uav_controller/pose'
UAV8_POSE_TOPIC = '/uav8/uav_controller/pose'
UAV9_POSE_TOPIC = '/uav9/uav_controller/pose'

plt.rcParams['text.usetex'] = True
plt.rcParams["font.family"] = "Times New Roman"

# bag = rosbag.Bag('/home/user/Documents/thesis/lemniscate_2.bag')
# bag = rosbag.Bag('/home/user/Documents/thesis/UCA_no_wind_2022-09-07-08-11-43.bag') 
bag = rosbag.Bag('/home/user/Documents/thesis/simulation_traj.bag') 

uav0_poses = []
uav1_poses = []
uav2_poses = []
uav3_poses = []
uav4_poses = []
uav5_poses = []
uav6_poses = []
uav7_poses = []
uav8_poses = []
uav9_poses = []


for topic, msg, t in bag.read_messages(topics=[UAV0_POSE_TOPIC, UAV1_POSE_TOPIC, UAV2_POSE_TOPIC, UAV3_POSE_TOPIC, UAV4_POSE_TOPIC, UAV5_POSE_TOPIC, UAV6_POSE_TOPIC, UAV7_POSE_TOPIC, UAV8_POSE_TOPIC, UAV9_POSE_TOPIC]):
    if topic == UAV0_POSE_TOPIC:
        uav0_poses.append(msg)
    elif topic == UAV1_POSE_TOPIC:
        uav1_poses.append(msg)
    elif topic == UAV2_POSE_TOPIC:
        uav2_poses.append(msg)
    elif topic == UAV3_POSE_TOPIC:
        uav3_poses.append(msg)
    elif topic == UAV4_POSE_TOPIC:
        uav4_poses.append(msg)
    elif topic == UAV5_POSE_TOPIC:
        uav5_poses.append(msg)
    elif topic == UAV6_POSE_TOPIC:
        uav6_poses.append(msg)
    elif topic == UAV7_POSE_TOPIC:
        uav7_poses.append(msg)
    elif topic == UAV8_POSE_TOPIC:
        uav8_poses.append(msg)
    elif topic == UAV9_POSE_TOPIC:
        uav9_poses.append(msg)    


# start_index = int(OFFSET * 100 / OMEGA)
# end_index = start_index + int(2 * math.pi * 100 / OMEGA) - 5


# gth_poses = gth_poses[start_index:end_index]
# ref_poses = ref_poses[start_index:end_index]

uav0_xs = np.array([msg.pose.position.x for msg in uav0_poses])
uav0_ys = np.array([msg.pose.position.y for msg in uav0_poses])
uav0_zs = np.array([msg.pose.position.z for msg in uav0_poses])

uav1_xs = np.array([msg.pose.position.x for msg in uav1_poses])
uav1_ys = np.array([msg.pose.position.y for msg in uav1_poses])
uav1_zs = np.array([msg.pose.position.z for msg in uav1_poses])

uav2_xs = np.array([msg.pose.position.x for msg in uav2_poses])
uav2_ys = np.array([msg.pose.position.y for msg in uav2_poses])
uav2_zs = np.array([msg.pose.position.z for msg in uav2_poses])

uav3_xs = np.array([msg.pose.position.x for msg in uav3_poses])
uav3_ys = np.array([msg.pose.position.y for msg in uav3_poses])
uav3_zs = np.array([msg.pose.position.z for msg in uav3_poses])

uav4_xs = np.array([msg.pose.position.x for msg in uav4_poses])
uav4_ys = np.array([msg.pose.position.y for msg in uav4_poses])
uav4_zs = np.array([msg.pose.position.z for msg in uav4_poses])

uav5_xs = np.array([msg.pose.position.x for msg in uav5_poses])
uav5_ys = np.array([msg.pose.position.y for msg in uav5_poses])
uav5_zs = np.array([msg.pose.position.z for msg in uav5_poses])

uav6_xs = np.array([msg.pose.position.x for msg in uav6_poses])
uav6_ys = np.array([msg.pose.position.y for msg in uav6_poses])
uav6_zs = np.array([msg.pose.position.z for msg in uav6_poses])

uav7_xs = np.array([msg.pose.position.x for msg in uav7_poses])
uav7_ys = np.array([msg.pose.position.y for msg in uav7_poses])
uav7_zs = np.array([msg.pose.position.z for msg in uav7_poses])

uav8_xs = np.array([msg.pose.position.x for msg in uav8_poses])
uav8_ys = np.array([msg.pose.position.y for msg in uav8_poses])
uav8_zs = np.array([msg.pose.position.z for msg in uav8_poses])

uav9_xs = np.array([msg.pose.position.x for msg in uav9_poses])
uav9_ys = np.array([msg.pose.position.y for msg in uav9_poses])
uav9_zs = np.array([msg.pose.position.z for msg in uav9_poses])



fig = plt.figure(figsize=(6, 6))
ax = fig.gca(projection='3d')
ax.plot(uav0_xs, uav0_ys, uav0_zs,'-',label='UAV0 trajectroy')
ax.plot(uav1_xs, uav1_ys, uav1_zs,'-',label='UAV1 trajectroy')
ax.plot(uav2_xs, uav2_ys, uav2_zs,'-',label='UAV2 trajectroy')
ax.plot(uav3_xs, uav3_ys, uav3_zs,'-',label='UAV3 trajectroy')
ax.plot(uav4_xs, uav4_ys, uav4_zs,'-',label='UAV4 trajectroy')
ax.plot(uav5_xs, uav5_ys, uav5_zs,'-',label='UAV5 trajectroy')
ax.plot(uav6_xs, uav6_ys, uav6_zs,'-',label='UAV6 trajectroy')
ax.plot(uav7_xs, uav7_ys, uav7_zs,'-',label='UAV7 trajectroy')
ax.plot(uav8_xs, uav8_ys, uav8_zs,'-',label='UAV8 trajectroy')
ax.plot(uav9_xs, uav9_ys, uav9_zs,'-',label='UAV9 trajectroy')
# ax.set_title(r'$\textrm{Lemniscate Trajectory,}\ \omega$=%.1f' % OMEGA, fontsize=14)
ax.set_title(r'$\textrm{UAV Trajectories}$', fontsize=14)
ax.set_xlabel(r'$\mathit{x}\textrm{-coordinate}\ (\mathrm{m})$', fontsize=14)
ax.set_ylabel(r'$\mathit{y}\textrm{-coordinate}\ (\mathrm{m})$', fontsize=14)
ax.set_zlabel(r'$\mathit{z}\textrm{-coordinate}\ (\mathrm{m})$', fontsize=14)
# ax.set_xlim([-1, 1])
# ax.set_ylim([-1, 1])
# # ax.set_zlim([1.5, 2.5])
# ax.set_zlim([0, 9])
ax.legend(bbox_to_anchor=(0,0.2,1,0.2), loc="lower left")




plt.show()
# for topic, msg, t in bag.read_messages(topics=['/ground_truth/pose', '/reference/pose']):
#     print(msg)

bag.close()
