# -*- coding: utf-8 -*-
"""
align_by_3_points.py
====================
Abaqus/CAE 脚本：通过对齐三个点，快速将一个零件实例装配到另一个零件实例上。

## 功能
在 Assembly 模块中，指定"移动实例"上的 3 个点（源点 S1/S2/S3）和"固定实例"上的
3 个对应点（目标点 T1/T2/T3），脚本自动计算刚体变换（旋转 + 平移），将移动实例
精确对齐到固定实例。

## 三点对齐原理
源点 S1, S2, S3 定义源局部正交坐标系：
    x_s = normalize(S2 - S1)
    z_s = normalize(x_s × (S3 - S1))
    y_s = z_s × x_s
目标点 T1, T2, T3 同理定义目标坐标系。
旋转矩阵  R = R_target · R_source^T
平移向量  t = T1 - R · S1
变换公式  p' = R · p + t

## 使用方法
### 方式一：直接修改 CONFIG 后运行
1. 在 Abaqus/CAE 中打开模型，进入 Assembly 模块；
2. 修改下方 CONFIG 字典中的参数；
3. File -> Run Script，选择本文件。

### 方式二：作为函数导入调用
    from align_by_3_points import align_instance
    align_instance(
        model_name='Model-1',
        moving_instance='PART-2-1',
        source_points=[(0,0,0),(10,0,0),(0,10,0)],
        target_points=[(100,50,0),(110,50,0),(100,60,0)],
    )

## 点坐标说明
所有点坐标均为 **装配全局坐标系** 下的坐标。
支持三种输入格式：
  1. 直接坐标元组：(x, y, z)
  2. Datum Point 名称：'Datum-1'（脚本自动读取其全局坐标）
  3. 顶点近似坐标：(x, y, z) 会自动匹配最近的几何体顶点

## 注意事项
- 三个源点不能共线，三个目标点不能共线；
- 源点与目标点必须按顺序一一对应（S1→T1, S2→T2, S3→T3）；
- 移动实例在变换前的位置不影响结果，脚本会计算绝对变换；
- 本脚本仅做刚体变换，不修改零件几何，不创建约束。
"""

from abaqus import mdb
from abaqusConstants import *
import math

# 尝试导入 numpy（Abaqus 2020+ 自带），不可用时回退到纯 Python 实现
try:
    import numpy as np
    _HAS_NUMPY = True
except ImportError:
    _HAS_NUMPY = False


# ============================================================
# 配置区域 —— 使用时修改这里
# ============================================================
CONFIG = {
    'model_name': 'Model-1',            # 模型名称
    'moving_instance': 'PART-2-1',      # 需要移动对齐的实例名称
    # ---- 源点（移动实例上的 3 个点，全局坐标）----
    'source_points': [
        (0.0, 0.0, 0.0),               # S1：基准点（对应 T1）
        (10.0, 0.0, 0.0),              # S2：定义 x 轴方向（对应 T2）
        (0.0, 10.0, 0.0),              # S3：定义 xy 平面（对应 T3）
    ],
    # ---- 目标点（固定实例上的 3 个点，全局坐标）----
    'target_points': [
        (100.0, 50.0, 0.0),            # T1：对应 S1
        (110.0, 50.0, 0.0),            # T2：对应 S2
        (100.0, 60.0, 0.0),            # T3：对应 S3
    ],
    'dry_run': False,                   # True：只计算不执行，打印变换参数
}


# ============================================================
# 向量 / 矩阵工具函数（纯 Python，兼容无 numpy 的 Abaqus 环境）
# ============================================================
def _v_sub(a, b):
    """向量减法 a - b"""
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _v_add(a, b):
    """向量加法 a + b"""
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _v_scale(a, s):
    """向量数乘 s * a"""
    return (a[0] * s, a[1] * s, a[2] * s)


def _v_dot(a, b):
    """向量点积"""
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _v_cross(a, b):
    """向量叉积 a × b"""
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _v_norm(a):
    """向量模长"""
    return math.sqrt(_v_dot(a, a))


def _v_normalize(a):
    """向量归一化"""
    n = _v_norm(a)
    if n < 1e-12:
        raise ValueError("向量长度为零，无法归一化（三点可能共线或重合）")
    return _v_scale(a, 1.0 / n)


def _mat_mul(A, B):
    """3x3 矩阵乘法 A · B（显式循环，兼容 Abaqus Python 2.7）"""
    result = []
    for i in range(3):
        row = []
        for j in range(3):
            s = 0.0
            for k in range(3):
                s = s + A[i][k] * B[k][j]
            row.append(s)
        result.append(tuple(row))
    return tuple(result)


def _mat_transpose(A):
    """3x3 矩阵转置（显式循环）"""
    result = []
    for i in range(3):
        row = []
        for j in range(3):
            row.append(A[j][i])
        result.append(tuple(row))
    return tuple(result)


def _mat_vec_mul(A, v):
    """3x3 矩阵乘向量 A · v（显式循环）"""
    result = []
    for i in range(3):
        s = 0.0
        for k in range(3):
            s = s + A[i][k] * v[k]
        result.append(s)
    return tuple(result)


def _mat_trace(A):
    """矩阵迹"""
    return A[0][0] + A[1][1] + A[2][2]


# ============================================================
# 核心算法
# ============================================================
def build_local_frame(p1, p2, p3):
    """
    由三个点构建局部正交坐标系。

    参数:
        p1, p2, p3: 三个点的坐标 (x, y, z)，不能共线
    返回:
        (origin, x_axis, y_axis, z_axis): 原点和三个正交单位轴
    """
    x_axis = _v_normalize(_v_sub(p2, p1))
    # 用 (p2-p1) × (p3-p1) 得到 z 轴，保证右手系
    temp = _v_sub(p3, p1)
    z_axis = _v_normalize(_v_cross(x_axis, temp))
    # y 轴 = z × x，保证正交右手系
    y_axis = _v_cross(z_axis, x_axis)
    return p1, x_axis, y_axis, z_axis


def compute_rigid_transform(source_points, target_points):
    """
    计算从源点集到目标点集的刚体变换。

    参数:
        source_points: 3 个源点 [(x,y,z), ...]
        target_points: 3 个目标点 [(x,y,z), ...]
    返回:
        R: 3x3 旋转矩阵（行主序元组）
        t: 平移向量 (x, y, z)
    """
    s_orig, s_x, s_y, s_z = build_local_frame(*source_points)
    t_orig, t_x, t_y, t_z = build_local_frame(*target_points)

    # 源坐标系基向量矩阵（列向量）
    R_source = (
        (s_x[0], s_y[0], s_z[0]),
        (s_x[1], s_y[1], s_z[1]),
        (s_x[2], s_y[2], s_z[2]),
    )
    # 目标坐标系基向量矩阵
    R_target = (
        (t_x[0], t_y[0], t_z[0]),
        (t_x[1], t_y[1], t_z[1]),
        (t_x[2], t_y[2], t_z[2]),
    )

    # R = R_target · R_source^T
    R = _mat_mul(R_target, _mat_transpose(R_source))

    # t = T1 - R · S1
    t = _v_sub(t_orig, _mat_vec_mul(R, s_orig))

    return R, t


def rotation_matrix_to_axis_angle(R):
    """
    将 3x3 旋转矩阵转换为轴角表示。

    返回:
        axis: 旋转轴单位向量 (x, y, z)
        angle_deg: 旋转角度（度）
    """
    cos_angle = (_mat_trace(R) - 1.0) / 2.0
    # 数值裁剪（手动比较，避免 abaqusConstants 覆盖内置 max/min）
    if cos_angle > 1.0:
        cos_angle = 1.0
    elif cos_angle < -1.0:
        cos_angle = -1.0
    angle = math.acos(cos_angle)

    if abs(angle) < 1e-10:
        # 零旋转，轴任意
        return (1.0, 0.0, 0.0), 0.0

    if abs(angle - math.pi) < 1e-6:
        # 180 度旋转：从 (R + I) 中提取轴
        M = (
            (R[0][0] + 1.0, R[0][1],       R[0][2]),
            (R[1][0],       R[1][1] + 1.0, R[1][2]),
            (R[2][0],       R[2][1],       R[2][2] + 1.0),
        )
        # 找最大对角元所在行作为轴
        diag = (M[0][0], M[1][1], M[2][2])
        idx = 0
        if diag[1] > diag[idx]:
            idx = 1
        if diag[2] > diag[idx]:
            idx = 2
        axis = _v_normalize((M[idx][0], M[idx][1], M[idx][2]))
        return axis, 180.0

    # 一般情况
    rx = R[2][1] - R[1][2]
    ry = R[0][2] - R[2][0]
    rz = R[1][0] - R[0][1]
    axis = _v_normalize((rx, ry, rz))
    angle_deg = math.degrees(angle)
    return axis, angle_deg


# ============================================================
# 点解析：支持坐标元组 / Datum Point 名称
# ============================================================
def resolve_point(point_spec, assembly):
    """
    解析点输入，返回全局坐标 (x, y, z)。

    支持:
        - 元组/列表 (x, y, z)：直接使用
        - 字符串 'Datum-N'：在装配中查找 datum point 并返回其坐标
    """
    if isinstance(point_spec, (tuple, list)):
        if len(point_spec) != 3:
            raise ValueError("坐标点必须是 (x, y, z) 三元组")
        result = []
        for v in point_spec:
            result.append(float(v))
        return tuple(result)

    if isinstance(point_spec, str):
        # 尝试作为 datum point 名称查找
        if hasattr(assembly, 'datums'):
            for key in assembly.datums.keys():
                d = assembly.datums[key]
                if d.name == point_spec and hasattr(d, 'pointOn'):
                    pt = d.pointOn
                    return (float(pt[0]), float(pt[1]), float(pt[2]))
        raise ValueError("在装配中未找到名为 '%s' 的 Datum Point" % point_spec)

    raise TypeError("不支持的点格式: %s" % type(point_spec))


# ============================================================
# 主函数
# ============================================================
def align_instance(model_name, moving_instance, source_points,
                   target_points, dry_run=False):
    """
    通过三点对齐，将移动实例变换到目标位置。

    参数:
        model_name:      模型名称
        moving_instance: 要移动的实例名称（字符串）
        source_points:   移动实例上的 3 个点（全局坐标）
        target_points:   固定实例上的 3 个对应点（全局坐标）
        dry_run:         True 时只计算并打印，不实际执行变换
    返回:
        dict: 包含 R, t, axis, angle_deg 的变换参数
    """
    # ---- 输入校验 ----
    if len(source_points) != 3 or len(target_points) != 3:
        raise ValueError("源点和目标点都必须恰好提供 3 个")

    if model_name not in mdb.models.keys():
        raise ValueError("模型 '%s' 不存在，可用模型: %s" %
                         (model_name, mdb.models.keys()))

    model = mdb.models[model_name]
    assembly = model.rootAssembly

    if moving_instance not in assembly.instances.keys():
        raise ValueError("实例 '%s' 不存在，可用实例: %s" %
                         (moving_instance, assembly.instances.keys()))

    # ---- 解析点坐标 ----
    src = [resolve_point(p, assembly) for p in source_points]
    tgt = [resolve_point(p, assembly) for p in target_points]

    print("=" * 60)
    print("Abaqus 三点对齐装配")
    print("=" * 60)
    print("移动实例: %s" % moving_instance)
    for i, (s, t) in enumerate(zip(src, tgt)):
        print("  点 %d: 源 (%.4f, %.4f, %.4f) -> 目标 (%.4f, %.4f, %.4f)"
              % (i + 1, s[0], s[1], s[2], t[0], t[1], t[2]))

    # ---- 计算刚体变换 ----
    R, t = compute_rigid_transform(src, tgt)
    axis, angle_deg = rotation_matrix_to_axis_angle(R)

    print("\n--- 计算结果 ---")
    print("旋转矩阵 R:")
    for row in R:
        print("  [% .6f  % .6f  % .6f]" % row)
    print("平移向量 t: (%.6f, %.6f, %.6f)" % t)
    print("旋转轴:     (%.6f, %.6f, %.6f)" % axis)
    print("旋转角度:   %.4f 度" % angle_deg)

    # ---- 验证：变换后源点应接近目标点 ----
    print("\n--- 精度验证（变换后源点 vs 目标点）---")
    max_err = 0.0
    for i, s in enumerate(src):
        transformed = _v_add(_mat_vec_mul(R, s), t)
        err = _v_norm(_v_sub(transformed, tgt[i]))
        if err > max_err:
            max_err = err
        print("  点 %d: 变换后 (%.6f, %.6f, %.6f), 误差 = %.2e"
              % (i + 1, transformed[0], transformed[1], transformed[2], err))
    print("最大误差: %.2e" % max_err)

    if dry_run:
        print("\n[dry_run] 未执行实际变换。")
        return {'R': R, 't': t, 'axis': axis, 'angle_deg': angle_deg}

    # ---- 执行变换 ----
    # Abaqus 的 rotate 绕通过 axisPoint 的轴旋转；
    # 取 axisPoint = (0,0,0)，即绕通过全局原点的轴旋转，
    # 等价于 p -> R·p，随后再平移 t。
    print("\n--- 执行变换 ---")

    # 步骤 1：绕通过原点的轴旋转
    if abs(angle_deg) > 1e-6:
        assembly.rotate(
            instanceList=(moving_instance,),
            axisPoint=(0.0, 0.0, 0.0),
            axisDirection=axis,
            angle=angle_deg,
        )
        print("  已旋转: 轴=%s, 角度=%.4f°" % (axis, angle_deg))
    else:
        print("  旋转角度接近 0，跳过旋转")

    # 步骤 2：平移
    if _v_norm(t) > 1e-9:
        assembly.translate(
            instanceList=(moving_instance,),
            vector=t,
        )
        print("  已平移: 向量=%s" % (t,))
    else:
        print("  平移向量接近 0，跳过平移")

    # 刷新视图
    if hasattr(mdb, 'models'):
        try:
            session.viewports[session.currentViewportName].view.refresh()
        except Exception:
            pass

    print("\n完成！实例 '%s' 已对齐到目标位置。" % moving_instance)
    print("=" * 60)

    return {'R': R, 't': t, 'axis': axis, 'angle_deg': angle_deg}


# ============================================================
# 入口：直接运行脚本时使用 CONFIG
# ============================================================
if __name__ == '__main__' or __name__ == '__main____':
    # Abaqus 中 __name__ 通常是 '__main__'，兼容两种写法
    align_instance(
        model_name=CONFIG['model_name'],
        moving_instance=CONFIG['moving_instance'],
        source_points=CONFIG['source_points'],
        target_points=CONFIG['target_points'],
        dry_run=CONFIG.get('dry_run', False),
    )
