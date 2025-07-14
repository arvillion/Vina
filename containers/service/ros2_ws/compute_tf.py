import numpy as np

def compute_similarity_transform(src_pts, dst_pts):
    """
    给定源坐标系和目标坐标系中的三个点，求相似变换矩阵。
    src_pts: np.array([[x1, y1], [x2, y2], [x3, y3]])
    dst_pts: np.array([[x1', y1'], [x2', y2'], [x3', y3']])
    返回: s, R (2x2), t (2x1)
    """

    # 转为 numpy 数组
    src_pts = np.asarray(src_pts)
    dst_pts = np.asarray(dst_pts)

    # 计算质心
    src_center = np.mean(src_pts, axis=0)
    dst_center = np.mean(dst_pts, axis=0)

    # 去中心化
    src_demean = src_pts - src_center
    dst_demean = dst_pts - dst_center

    # 计算缩放（scale）
    src_norm = np.linalg.norm(src_demean)
    dst_norm = np.linalg.norm(dst_demean)
    scale = dst_norm / src_norm

    # 计算旋转（使用SVD求解）
    H = src_demean.T @ dst_demean
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # 修正可能出现的反射
    if np.linalg.det(R) < 0:
        Vt[1,:] *= -1
        R = Vt.T @ U.T

    # 计算平移
    t = dst_center.T - scale * R @ src_center.T

    return scale, R, t

# 示例输入（三个点）
dst_points = [[2.90, 3.81], [5.39, 7.02], [6.82, 1.83], [0.53,0.75]]               # 目标坐标系中的对应点
src_points = [[6.73, -2.1], [3.04, -0.34], [7.81, 2.13], [10.21,-3.81]]               # 原坐标系中的点

scale, R, t = compute_similarity_transform(src_points, dst_points)

# 输出结果
print("Scale:", scale)
print("Rotation matrix:\n", R)
print("Translation vector:", t)

# 也可以拼接成一个 3x3 仿射矩阵
T = np.eye(3)
T[0:2, 0:2] = scale * R
T[0:2, 2] = t
print("Full 2D transformation matrix:\n", T)


position = np.array([[10.91, -4.64],
                    [0.41, -0.05,],
                    [7.96, -1.00],
                    [9.23, -0.94],
                    [3.51, 0.10],
                    [11.47, 0.31]]).T
converted = (R@position).T + t
print(converted)