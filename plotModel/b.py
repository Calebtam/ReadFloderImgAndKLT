import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import matplotlib.animation as animation

# 假设装置的形状为正方形，带有一条圆形边
def generate_device_shape(side_length, radius):
    # 正方形的四个角
    square_corners = np.array([
        [-side_length / 2, -side_length / 2],
        [side_length / 2, -side_length / 2],
        [side_length / 2, side_length / 2],
        [-side_length / 2, side_length / 2]
    ])
    
    # 圆的路径
    angle = np.linspace(0, np.pi, 100)
    arc_x = radius * np.cos(angle)
    arc_y = radius * np.sin(angle)
    
    # 装置的边界
    device_shape = np.concatenate((square_corners, np.column_stack((arc_x, arc_y))), axis=0)
    
    return device_shape

# 计算旋转和平移后的外边界
def rotate_and_translate(shape, angle, translation):
    # 旋转矩阵
    rotation_matrix = np.array([
        [np.cos(angle), -np.sin(angle)],
        [np.sin(angle), np.cos(angle)]
    ])
    
    # 旋转后的点
    rotated_shape = np.dot(shape, rotation_matrix.T)
    
    # 平移
    rotated_shape += translation
    
    return rotated_shape

# 绘制外边界
def plot_boundary(shape, boundary_color='blue'):
    plt.fill(shape[:, 0], shape[:, 1], color=boundary_color, alpha=0.5)

# 动画更新函数
def update(frame, device_shape, side_length, radius, ax):
    ax.clear()
    
    # 计算当前的旋转角度和位置
    angle = np.radians(frame)
    translation = np.array([np.cos(angle) * 2, np.sin(angle) * 2])  # 简单的平移路径
    
    # 获取旋转和平移后的形状
    moved_shape = rotate_and_translate(device_shape, angle, translation)
    
    # 绘制外边界
    plot_boundary(moved_shape, boundary_color='orange')
    
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_aspect('equal')
    ax.set_title("装置外边界运动区域")
    
    return ax,

# 初始化参数
side_length = 4  # 正方形边长
radius = 2  # 圆弧半径
device_shape = generate_device_shape(side_length, radius)

# 创建动画
fig, ax = plt.subplots()
ani = animation.FuncAnimation(fig, update, frames=360, fargs=(device_shape, side_length, radius, ax), interval=50)

plt.show()