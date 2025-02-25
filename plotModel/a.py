import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import matplotlib.animation as animation

from matplotlib import rcParams

# 使用凸包计算最小外接边界
from scipy.spatial import ConvexHull

# 保存图像的回调函数，在动画结束时保存
def save_frame(fig, name='bullet_motion.jpg'):
    fig.savefig(name, format='jpg', dpi=300)  # 保存图片
        
# 生成复合形状：半圆加上正方形
def generate_bullet_shape(radius, square_side):
    # 半圆部分：角度从0到π
    angle = np.linspace(0, np.pi, 30)
    arc_x = radius * np.cos(angle)  # 开始是 (0, 1) 结束是 (-1, 0)
    arc_y = radius + radius * np.sin(angle)
    
    # 正方形的顶边与半圆的直径对接
    square_x = np.array([-radius, -radius, 0.0,  radius,    radius])  # 两个角与直径对齐
    square_y = np.array([radius,  0.0,     0.0,     0.0,    radius])
    
    # 合并所有部分，形成整体形状
    shape_x = np.concatenate([arc_x, square_x])
    shape_y = np.concatenate([arc_y, square_y])
    
    # 创建图形并绘制静态初始状态
    fig, ax = plt.subplots()

    # 在动画开始前绘制初始的复合形状
    plot_boundary(shape_x, shape_y, boundary_color='blue')
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_aspect('equal')
    ax.set_title("半圆与正方形外边界初始状态")

    # 显示静态图像
    # plt.show()
    save_frame(fig, 'bullet_motion.jpg')
    
    # 旋转 90 度（顺时针旋转）
    rotation_matrix = np.array([[0, 1], [-1, 0]])  # 90度顺时针旋转矩阵
    points = np.vstack([shape_x, shape_y]).T  # 合并x和y坐标
    rotated_points = np.dot(points, rotation_matrix.T)  # 旋转后的坐标

    # 返回旋转后的坐标
    rotated_x, rotated_y = rotated_points[:, 0], rotated_points[:, 1]
    
    return rotated_x, rotated_y

def generate_bullet_shape_with_density(radius, square_side, angle_step=5, distance_step=0.1):
    """
    根据角度和距离设置插值密度，生成半圆和正方形的复合形状。
    
    参数:
        radius (float): 半圆的半径。
        square_side (float): 正方形的边长。
        angle_step (float): 半圆部分的角度步长，用来控制采样密度。
        distance_step (float): 正方形边的插值步长，用来控制采样密度。
        
    返回:
        shape_x (ndarray): 生成的形状的x坐标。
        shape_y (ndarray): 生成的形状的y坐标。
    """
    # 半圆部分：角度从0到π，步长根据角度控制
    angle = np.arange(0, np.pi + angle_step * np.pi / 180, angle_step * np.pi / 180)  # 角度步长
    arc_x = radius * np.cos(angle)
    arc_y = radius + radius * np.sin(angle)

    # 正方形的四个角点，首先根据四条边的长度进行插值
    square_corners = np.array([
        [-radius, radius],  # 左上角
        [-radius, 0],       # 左下角
        [radius, 0],        # 右下角
        [radius, radius]    # 右上角
    ])
    
    square_x = []
    square_y = []
    
    # 遍历每一条边，计算每条边的长度并根据距离步长插值
    for i in range(4):
        x0, y0 = square_corners[i]
        x1, y1 = square_corners[(i + 1) % 4]
        
        # 计算边的长度
        edge_length = np.linalg.norm([x1 - x0, y1 - y0])
        
        # 根据边的长度决定插值点的数量，距离步长
        num_samples = int(np.ceil(edge_length / distance_step))
        
        # 在每条边上生成插值点
        edge_x = np.linspace(x0, x1, num_samples)
        edge_y = np.linspace(y0, y1, num_samples)
        
        square_x.extend(edge_x)
        square_y.extend(edge_y)
    
    # 合并半圆和正方形的部分
    shape_x = np.concatenate([arc_x, square_x])
    shape_y = np.concatenate([arc_y, square_y])
    
    # 旋转 90 度（顺时针旋转）
    rotation_matrix = np.array([[0, 1], [-1, 0]])  # 90度顺时针旋转矩阵
    points = np.vstack([shape_x, shape_y]).T  # 合并x和y坐标
    rotated_points = np.dot(points, rotation_matrix.T)  # 旋转后的坐标

    # 返回旋转后的坐标
    rotated_x, rotated_y = rotated_points[:, 0], rotated_points[:, 1]
    
    return rotated_x, rotated_y


# 计算旋转和平移后的外边界
def rotate_and_translate(shape_x, shape_y, angle, translation):
    # 旋转矩阵
    rotation_matrix = np.array([
        [np.cos(angle), -np.sin(angle)],
        [np.sin(angle), np.cos(angle)]
    ])
    
    # 旋转后的点
    rotated_points = np.dot(np.vstack([shape_x, shape_y]).T, rotation_matrix.T)
    
    # 平移
    rotated_points += translation
    
    return rotated_points[:, 0], rotated_points[:, 1]

# 生成一段连续的位姿（旋转和平移），返回每一帧的位置和角度
def generate_poses(angle_step, distance_step, total_frames):
    poses = []
    for frame in range(total_frames):
        angle = np.radians(frame * angle_step)  # 根据角度步长计算角度
        translation = np.array([frame * distance_step, 0])  # 根据距离步长计算平移
        poses.append((angle, translation))
    return poses

def generate_motion_poses(start_angle, angle_step, distance_step, num_frames):
    poses = []
    
    # 计算每一帧的位姿
    for frame in range(num_frames):
        angle = start_angle + frame * angle_step
        translation = np.array([np.cos(angle) * frame * distance_step, np.sin(angle) * frame * distance_step])
        poses.append((angle, translation))
    
    return poses

def apply_poses_to_shape(shape_x, shape_y, poses):
    all_shapes = []  # 保存每一帧变换后的形状
    
    for angle, translation in poses:
        moved_x, moved_y = rotate_and_translate(shape_x, shape_y, angle, translation)
        all_shapes.append((moved_x, moved_y))  # 保存变换后的形状
        
    return all_shapes
# ========================================================
def get_convex_hull(x, y):
    points = np.vstack([x, y]).T
    hull = ConvexHull(points)
    return points[hull.vertices]

def fit_convex_hull(all_shapes):
    """
    拟合整个运动过程中形状覆盖的区域，得到一个凸包。
    
    参数:
        all_shapes (list): 形状的列表，每个形状是由(x, y)点组成的元组。
        
    返回:
        hull (ConvexHull): 返回计算出的凸包。
    """
    # 合并所有帧的点
    all_points = []
    
    # 创建一个黑色的空白图像，用于OpenCV显示
    img = np.zeros((1600, 900, 3), dtype=np.uint8)
    
    # for moved_x, moved_y in all_shapes:
    for i, (moved_x, moved_y) in enumerate(all_shapes):
        all_points.append(np.column_stack([moved_x, moved_y]))  # 合并每一帧的点
        
        all_points_flat = np.vstack(all_points)  # 合并所有帧的点集

        # 清空图像并绘制所有点
        img.fill(0)  # 重置图像
        for point in all_points_flat:
            cv2.circle(img, (int(point[0] * 50 + 300), int(300 - point[1] * 50)), 2, (255, 0, 0), -1)  # 缩放并绘制点

        # 计算凸包
        hull = ConvexHull(all_points_flat)

        # 绘制凸包的外边界
        for simplex in hull.simplices:
            pt1 = tuple(all_points_flat[simplex[0]].astype(int) * 50 + [300, 300])
            pt2 = tuple(all_points_flat[simplex[1]].astype(int) * 50 + [300, 300])
            cv2.line(img, pt1, pt2, (0, 0, 255), 2)  # 绘制红色线条

        # 显示图像
        cv2.imshow('Convex Hull', img)
        
        # 等待键盘事件，按键则继续，等待 100 毫秒
        if cv2.waitKey(100) & 0xFF == ord('q'):
            break  # 按'q'退出
        
    all_points = np.vstack(all_points)  # 合并所有帧的点

    # 使用ConvexHull拟合外边界
    hull = ConvexHull(all_points)
    
    return hull

def plot_convex_hull(hull, ax):
    """
    绘制拟合的凸包外边界。
    
    参数:
        hull (ConvexHull): 计算出的凸包。
        ax (Axes): 绘图的Axes对象。
    """
    # 绘制凸包的外边界
    hull_points = np.column_stack([hull.vertices, hull.vertices])  # 获取凸包顶点
    ax.fill(hull_points[:, 0], hull_points[:, 1], color='orange', alpha=0.3)
    ax.plot(np.append(hull_points[:, 0], hull_points[0, 0]), 
            np.append(hull_points[:, 1], hull_points[0, 1]), 
            color='blue', lw=2, label="拟合的外边界")

    ax.set_aspect('equal')
    # ax.set_title("运动过程中的拟合外边界")
    ax.set_title("Convex Hull of the Moving Object")
    ax.legend()
    
# 计算每一帧的物体轮廓的最小外接边界（这里使用一个凸包的方法）
def calculate_motion_boundaries(shape_x, shape_y, num_frames, translation_radius):
    boundaries = []
    
    for frame in range(num_frames):
        angle = np.radians(frame)
        translation = np.array([np.cos(angle) * translation_radius, np.sin(angle) * translation_radius])
        
        # 获取旋转和平移后的物体形状
        moved_x, moved_y = rotate_and_translate(shape_x, shape_y, angle, translation)
        
        # 获取当前轮廓的最小外接多边形
        convex_hull = get_convex_hull(moved_x, moved_y)
        
        boundaries.append(convex_hull)
    
    return boundaries

# 计算运动过程中可能的碰撞区域（每一帧的边界）
def calculate_motion_boundaries_from_poses(shape_x, shape_y, poses):
    boundaries = []
    for translation, rotation in poses:
        # 获取旋转和平移后的物体形状
        moved_x, moved_y = rotate_and_translate(shape_x, shape_y, rotation, translation)

        # 获取当前轮廓的最小外接多边形
        convex_hull = get_convex_hull(moved_x, moved_y)
        boundaries.append(convex_hull)
    return boundaries

# 绘制外边界
def plot_boundary(shape_x, shape_y, boundary_color='blue'):
    plt.fill(shape_x, shape_y, color=boundary_color, alpha=0.5)
    
# 绘制运动范围
def plot_motion_range(all_shapes, ax):
    all_points = []  # 用于存储所有帧的所有点
    
    for moved_x, moved_y in all_shapes:
        all_points.append(np.column_stack([moved_x, moved_y]))  # 合并每一帧的点
    
    all_points = np.vstack(all_points)  # 合并所有帧的点
    
    # 计算最小/最大边界
    min_x, min_y = np.min(all_points, axis=0)
    max_x, max_y = np.max(all_points, axis=0)
    
    # 设置显示的边界
    ax.set_xlim(min_x - 1, max_x + 1)
    ax.set_ylim(min_y - 1, max_y + 1)
    
    # 绘制所有帧的外边界
    for moved_x, moved_y in all_shapes:
        plot_boundary(moved_x, moved_y, boundary_color='orange')
    
    # 绘制包围区域的矩形
    ax.plot([min_x, max_x, max_x, min_x, min_x], 
            [min_y, min_y, max_y, max_y, min_y], 
            color='blue', lw=2, label="运动范围边界")
    
    ax.set_aspect('equal')
    # ax.set_title("运动过程中的外边界范围")
    ax.set_title("Motion Range Boundary")
    ax.legend()

def update(frame, all_shapes, ax):
    ax.clear()
    
    # 显示当前帧的形状
    moved_x, moved_y = all_shapes[frame]
    
    # 绘制当前形状
    plot_boundary(moved_x, moved_y, boundary_color='orange')
    
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_aspect('equal')
    ax.set_title("复合形状的运动")
    
    return ax,


# ==============================================
# 生成位姿
step = np.pi/180
poses = generate_motion_poses(start_angle=0, angle_step= 0.2*step, distance_step=0.3, num_frames=20)

# 生成初始形状
# shape_x, shape_y = generate_bullet_shape(radius=2, square_side=4)
shape_x, shape_y = generate_bullet_shape_with_density(2, 4, angle_step=20, distance_step=0.5)

# 应用位姿并保存每帧的形状
all_shapes = apply_poses_to_shape(shape_x, shape_y, poses)

# 创建动画
fig, ax = plt.subplots()
ani = animation.FuncAnimation(fig, update, frames=len(all_shapes), fargs=(all_shapes, ax), interval=50)

# 保存动画
ani.save('motion_animation.gif', writer='imagemagick', fps=30)

# # 计算所有帧的碰撞区域
# motion_boundaries = calculate_motion_boundaries_from_poses(shape_x, shape_y, poses)

# 绘制并保存碰撞区域
fig2, ax2 = plt.subplots()
plot_motion_range(all_shapes, ax2)
save_frame(fig2, 'motion_boundary.jpg')


# 计算整个运动过程中形状覆盖的区域的凸包
hull = fit_convex_hull(all_shapes)

# 绘制凸包
fig, ax = plt.subplots()
plot_convex_hull(hull, ax)
plt.show()