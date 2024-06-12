#!/usr/bin/env python3
print("importing libraries")

import rosbag
import rospy
import re
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from sensor_msgs.msg import Imu
from collections import defaultdict

# 读取bag文件
# bag = rosbag.Bag('/home/tam/DataSet/IMU/ch0x0/slam_record_1703531660.bag')
bag = rosbag.Bag('/home/tam/DataSet/IMU/icm/slam_record_1703540681.bag')
# bag = rosbag.Bag('/home/tam/DataSet/ch0x0_imu_2024-01-03-19-53-12.bag', 'r')

# 办公室 
# 方框 
# path_bag: /home/tam/DataSet/IMU/ch0x0/green_work_1703536804.bag
# 原地转
# path_bag: /home/tam/DataSet/IMU/ch0x0/slam_record_1703531660.bag   
# 方框 
# path_bag: /home/tam/DataSet/IMU/ch0x0/slam_record_1703532005.bag

# 草地 方框  原地转  斜转 
# path_bag: /home/tam/DataSet/IMU/ch0x0/slam_record_1703532071.bag
# path_bag: /home/tam/DataSet/IMU/ch0x0/slam_record_1703532699.bag  
# path_bag: /home/tam/DataSet/IMU/ch0x0/slam_record_1703533141.bag

# 初始化数据存储字典
data = defaultdict(list)

print("importing bag ++++++++++ ")

# 定义你感兴趣的时间段
start_time = bag.get_start_time()  # 开始时间
# end_time = start_time + 2000  # 结束时间，单位为秒
end_time = bag.get_end_time()
    
# 从bag中提取IMU数据
for topic, msg, t in bag.read_messages(topics=['/camera/imu']):
    # if isinstance(msg, Imu):
    if start_time <= t.to_sec() <= end_time:
        data['time'].append(t.to_sec())
        data['acc_x'].append(msg.linear_acceleration.x)
        data['acc_y'].append(msg.linear_acceleration.y)
        data['acc_z'].append(msg.linear_acceleration.z)
        data['gyro_x'].append(msg.angular_velocity.x)
        data['gyro_y'].append(msg.angular_velocity.y)
        data['gyro_z'].append(msg.angular_velocity.z)
        # print("Accelerometer data: x = {:.6f}, y = {:.6f}, z = {:.6f}".format(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z))
        # print("Gyroscope data: x = {:.6f}, y = {:.6f}, z = {:.6f}".format(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z))
bag.close()
print("imported bag =========== ")

# 将时间和加速度数据转换为numpy数组
time = np.array(data['time'])
acc_x = np.array(data['acc_x'])
acc_y = np.array(data['acc_y'])
acc_z = np.array(data['acc_z'])
gyro_x = np.array(data['gyro_x'])
gyro_y = np.array(data['gyro_y'])
gyro_z = np.array(data['gyro_z'])

# 计算每10秒的平均值和标准差
avg = defaultdict(list)
std_dev = defaultdict(list)
time_avg = []

window = 50
# 时间窗口（秒）
# for i in range(0, len(time), window):
#     if i + window < len(time):
#         avg['acc_x'].append(np.mean(acc_x[i:i+window]))
#         std_dev['acc_x'].append(np.std(acc_x[i:i+window]))
#         time_avg.append(np.mean(time[i:i+window]))
# 绘制均值和标准差
# plt.errorbar(time_avg, avg, yerr=std_dev, fmt='-o')
# plt.xlabel('Time (s)')
# plt.ylabel('Acceleration (m/s^2)')
# plt.title('IMU x acceleration over time')
# plt.show()


for i in range(0, len(time), window):
# for i in range(10, 1010, window):
    if i + window < len(time):
        avg['acc_x'].append(np.mean(acc_x[i:i+window]))
        avg['acc_y'].append(np.mean(acc_y[i:i+window]))
        avg['acc_z'].append(np.mean(acc_z[i:i+window]))
        avg['gyro_x'].append(np.mean(gyro_x[i:i+window]))
        avg['gyro_y'].append(np.mean(gyro_y[i:i+window]))
        avg['gyro_z'].append(np.mean(gyro_z[i:i+window]))
        std_dev['acc_x'].append(np.std(acc_x[i:i+window]))
        std_dev['acc_y'].append(np.std(acc_y[i:i+window]))
        std_dev['acc_z'].append(np.std(acc_z[i:i+window]))
        std_dev['gyro_x'].append(np.std(gyro_x[i:i+window]))
        std_dev['gyro_y'].append(np.std(gyro_y[i:i+window]))
        std_dev['gyro_z'].append(np.std(gyro_z[i:i+window]))
        time_avg.append(np.mean(time[i:i+window]))

# # 绘制均值和标准差
# for key in avg.keys():
#     plt.figure()
#     plt.errorbar(time_avg, avg[key], yerr=std_dev[key], fmt='-o')
#     plt.xlabel('Time (s)')
#     plt.ylabel(f'{key} value')
#     plt.title(f'IMU {key} over time')
#     plt.show()

plt.figure(1)
plt.plot(time_avg, avg['acc_x'], 'b+-', markersize=2, label='ori')
plt.xlabel('Time (s)')
plt.ylabel(f'acc_x value')
plt.title(f'IMU acc_x over time')
# plt.savefig("acc_x.png", bbox_inches='tight', dpi=300)
plt.figure(2)
plt.plot(time_avg, avg['acc_y'], 'b+-', markersize=2, label='ori')
plt.xlabel('Time (s)')
plt.ylabel(f'acc_y value')
plt.title(f'IMU acc_y over time')
# plt.savefig("acc_y.png", bbox_inches='tight', dpi=300)
plt.figure(3)
plt.plot(time_avg, avg['acc_z'], 'b+-', markersize=2, label='ori')
plt.xlabel('Time (s)')
plt.ylabel(f'acc_z value')
plt.title(f'IMU acc_z over time')
# plt.savefig("acc_z.png", bbox_inches='tight', dpi=300)
plt.figure(4)
plt.plot(time_avg, avg['gyro_x'], 'b+-', markersize=2, label='ori')
plt.xlabel('Time (s)')
plt.ylabel(f'gyro_x value')
plt.title(f'IMU gyro_x over time')
# plt.savefig("gyro_x.png", bbox_inches='tight', dpi=300)
plt.figure(5)
plt.plot(time_avg, avg['gyro_y'], 'b+-', markersize=2, label='ori')
plt.xlabel('Time (s)')
plt.ylabel(f'gyro_y value')
plt.title(f'IMU gyro_y over time')
# plt.savefig("gyro_y.png", bbox_inches='tight', dpi=300)
plt.figure(6)
plt.plot(time_avg, avg['gyro_z'], 'b+-', markersize=2, label='ori')
plt.xlabel('Time (s)')
plt.ylabel(f'gyro_z value')
plt.title(f'IMU gyro_z over time')
# plt.savefig("gyro_z.png", bbox_inches='tight', dpi=300)
plt.show()



# plot imu data
# # acc-x
# plt.figure(1)
# plt.plot(time_avg, avg['acc_x'][:, 0], 'b+', markersize=8, label='ori')
# plt.xlabel('frame time(subtract the first frame time)/s')
# plt.ylabel('measurements')
# plt.legend()
# plt.title('acc-x')
# plt.savefig("acc_x.png", bbox_inches='tight', dpi=300)
# # acc-y
# plt.figure(2)
# plt.plot(imu_msg_times_arr, ori_imu_arr[:, 1], 'b+', markersize=8, label='ori')
# plt.xlabel('frame time(subtract the first frame time)/s')
# plt.ylabel('measurements')
# plt.legend()
# plt.title('acc-y')
# plt.savefig("acc_y.png", bbox_inches='tight', dpi=300)
# # acc-z
# plt.figure(3)
# plt.plot(imu_msg_times_arr, ori_imu_arr[:, 2], 'b+', markersize=8, label='ori')
# plt.xlabel('frame time(subtract the first frame time)/s')
# plt.ylabel('measurements')
# plt.legend()
# plt.title('acc-z')
# plt.savefig("acc_z.png", bbox_inches='tight', dpi=300)
# # gyr
# # gyr-x
# plt.figure(4)
# plt.plot(imu_msg_times_arr, ori_imu_arr[:, 3], 'b+', markersize=8, label='ori')
# plt.xlabel('frame time(subtract the first frame time)/s')
# plt.ylabel('measurements')
# plt.legend()
# plt.title('gyr-x')
# plt.savefig("gyro_x.png", bbox_inches='tight', dpi=300)
# # gyr-y
# plt.figure(5)
# plt.plot(imu_msg_times_arr, ori_imu_arr[:, 4], 'b+', markersize=8, label='ori')
# plt.xlabel('frame time(subtract the first frame time)/s')
# plt.ylabel('measurements')
# plt.legend()
# plt.title('gyr-y')
# plt.savefig("gyro_y.png", bbox_inches='tight', dpi=300)
# # gyr-z
# plt.figure(6)
# plt.plot(imu_msg_times_arr, ori_imu_arr[:, 5], 'b+', markersize=8, label='ori')
# plt.xlabel('frame time(subtract the first frame time)/s')
# plt.ylabel('measurements')
# plt.legend()
# plt.title('gyr-z')
# plt.savefig("gyro_z.png", bbox_inches='tight', dpi=300)
plt.show()