#!/usr/bin/env python3
print("importing libraries")

import rosbag
import rospy
import re
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from sensor_msgs.msg import Imu

# bag = rosbag.Bag('/home/tam/DataSet/ch0x0_imu_2024-01-03-19-53-12.bag', 'r')
bag = rosbag.Bag('/home/tam/DataSet/IMU/ch0x0/slam_record_1703534158.bag', 'r')

# Load data into a pandas dataframe
# df = pd.DataFrame(columns=['timestamp', 'acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z'])
df = pd.DataFrame(columns=['timestamp', 'acc_x'])

# accel_x_list = []
# accel_y_list = []
# accel_z_list = []
# gyro_x_list = []
# gyro_y_list = []
# gyro_z_list = []
        
imu_data = []

print("importing bag ++++++++++ ")
for topic, msg, t in bag.read_messages(topics=['/camera/imu']):
    # imu_data.append(msg)
    
    # 首先将IMU数据存储在一个pandas dataframe中
    # df = df.append({'timestamp': msg.header.stamp.to_sec(), 'acc_x': msg.linear_acceleration.x, 'acc_y': msg.linear_acceleration.y, 'acc_z': msg.linear_acceleration.z, 'gyro_x': msg.angular_velocity.x, 'gyro_y': msg.angular_velocity.y, 'gyro_z': msg.angular_velocity.z}, ignore_index=True)
    # df = df.append({'timestamp': msg.header.stamp.to_sec(), 'acc_x': msg.linear_acceleration.x}, ignore_index=True)
    new_data = pd.DataFrame({'timestamp': [msg.header.stamp.to_sec()], 'acc_x': [msg.linear_acceleration.x]})
    df = pd.concat([df, new_data], ignore_index=True)

    # accel_x_list.append(msg.linear_acceleration.x)
    # accel_y_list.append(msg.linear_acceleration.y)
    # accel_z_list.append(msg.linear_acceleration.z)
    # gyro_x_list.append(msg.angular_velocity.x)
    # gyro_y_list.append(msg.angular_velocity.y)
    # gyro_z_list.append(msg.angular_velocity.z)
    # print(f"IMU数据：\n{imu_data}")
    # print("Accelerometer data: x = {:.6f}, y = {:.6f}, z = {:.6f}".format(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z))
    # print("Gyroscope data: x = {:.6f}, y = {:.6f}, z = {:.6f}".format(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z))

bag.close()
print("imported bag =========== ")


# 将时间戳转换为datetime对象 Convert timestamp to datetime object
df['timestamp'] = pd.to_datetime(df['timestamp'], unit='s')

# 将其设置为索引 Set timestamp as index
df.set_index('timestamp', inplace=True)

# 代码使用resample()方法将数据按10秒间隔重新采样，并计算每个10秒间隔的平均值和标准差。 Resample data every 10 seconds
accel_x_10s = df['acc_x'].resample('100S').mean()

# Calculate standard deviation for each 10-second period
accel_x_std_10s = df['acc_x'].resample('100S').std()

# # 绘制图像
plt.errorbar(np.arange(len(accel_x_10s)), accel_x_10s[:,0], yerr=accel_x_std_10s[:,0], fmt='o', capsize=3)

# 添加标签和标题
plt.xlabel('10 Second Intervals')
plt.ylabel('IMU Data')
plt.title('Average IMU Data with Error Bars')

# 显示图像
plt.show()

# # 每10秒求平均
# # imu_data_10s = np.mean(imu_data.reshape(-1, 10, 6), axis=1)

# # Convert list to NumPy array
# accel_x_array = np.array(accel_x_list)

# # 每100秒求平均
# imu_data_100s = np.mean(accel_x_array.reshape(-1, 100, 6), axis=1)
# # imu_data_100s = np.array(imu_data_100s)

# # 每100秒求标准差
# imu_data_std_100s = np.std(accel_x_array.reshape(-1, 100, 6), axis=1)
# # imu_data_std_100s = np.array(imu_data_std_100s)

# # 计算全部数据标准差
# # imu_data_std = np.std(imu_data_100s, axis=0)

# # print(f"IMU数据：\n{imu_data}")

# # 绘制图像
# plt.errorbar(np.arange(len(imu_data_100s)), imu_data_100s[:,0], yerr=imu_data_std_100s[:,0], fmt='o', capsize=3)

# # 添加标签和标题
# plt.xlabel('10 Second Intervals')
# plt.ylabel('IMU Data')
# plt.title('Average IMU Data with Error Bars')

# # 显示图像
# plt.show()

# class IMUData:
#     def __init__(self, accelerometer_x, accelerometer_y, accelerometer_z, gyroscope_x, gyroscope_y, gyroscope_z):
#         self.accelerometer_x = accelerometer_x
#         self.accelerometer_y = accelerometer_y
#         self.accelerometer_z = accelerometer_z
#         self.gyroscope_x = gyroscope_x
#         self.gyroscope_y = gyroscope_y
#         self.gyroscope_z = gyroscope_z