# -*- coding: utf-8 -*-
"""
align_by_3_points_kernel.py
===========================
三点对齐装配的 kernel 端逻辑。
由 GUI 端通过 AFXGuiCommand 调用 align_by_3_points_function()。
"""

from __future__ import print_function
from abaqus import *
from abaqusConstants import *
import math


# ============================================================
# 向量 / 矩阵工具（纯 Python，兼容 Abaqus Python 2.7）
# ============================================================
def _v_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])

def _v_add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])

def _v_scale(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)

def _v_dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

def _v_cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )

def _v_norm(a):
    return math.sqrt(_v_dot(a, a))

def _v_normalize(a):
    n = _v_norm(a)
    if n < 1e-12:
        raise ValueError("Vector zero length (points may be collinear)")
    return _v_scale(a, 1.0 / n)

def _mat_mul(A, B):
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
    result = []
    for i in range(3):
        row = []
        for j in range(3):
            row.append(A[j][i])
        result.append(tuple(row))
    return tuple(result)

def _mat_vec_mul(A, v):
    result = []
    for i in range(3):
        s = 0.0
        for k in range(3):
            s = s + A[i][k] * v[k]
        result.append(s)
    return tuple(result)

def _mat_trace(A):
    return A[0][0] + A[1][1] + A[2][2]


# ============================================================
# 核心算法
# ============================================================
def build_local_frame(p1, p2, p3):
    x_axis = _v_normalize(_v_sub(p2, p1))
    temp = _v_sub(p3, p1)
    z_axis = _v_normalize(_v_cross(x_axis, temp))
    y_axis = _v_cross(z_axis, x_axis)
    return p1, x_axis, y_axis, z_axis


def compute_rigid_transform(source_points, target_points):
    s_orig, s_x, s_y, s_z = build_local_frame(*source_points)
    t_orig, t_x, t_y, t_z = build_local_frame(*target_points)

    R_source = (
        (s_x[0], s_y[0], s_z[0]),
        (s_x[1], s_y[1], s_z[1]),
        (s_x[2], s_y[2], s_z[2]),
    )
    R_target = (
        (t_x[0], t_y[0], t_z[0]),
        (t_x[1], t_y[1], t_z[1]),
        (t_x[2], t_y[2], t_z[2]),
    )
    R = _mat_mul(R_target, _mat_transpose(R_source))
    t = _v_sub(t_orig, _mat_vec_mul(R, s_orig))
    return R, t


def rotation_matrix_to_axis_angle(R):
    cos_angle = (_mat_trace(R) - 1.0) / 2.0
    if cos_angle > 1.0:
        cos_angle = 1.0
    elif cos_angle < -1.0:
        cos_angle = -1.0
    angle = math.acos(cos_angle)

    if abs(angle) < 1e-10:
        return (1.0, 0.0, 0.0), 0.0

    if abs(angle - math.pi) < 1e-6:
        M = (
            (R[0][0] + 1.0, R[0][1],       R[0][2]),
            (R[1][0],       R[1][1] + 1.0, R[1][2]),
            (R[2][0],       R[2][1],       R[2][2] + 1.0),
        )
        diag = (M[0][0], M[1][1], M[2][2])
        idx = 0
        if diag[1] > diag[idx]:
            idx = 1
        if diag[2] > diag[idx]:
            idx = 2
        axis = _v_normalize((M[idx][0], M[idx][1], M[idx][2]))
        return axis, 180.0

    rx = R[2][1] - R[1][2]
    ry = R[0][2] - R[2][0]
    rz = R[1][0] - R[0][1]
    axis = _v_normalize((rx, ry, rz))
    return axis, math.degrees(angle)


# ============================================================
# 从拾取对象中提取坐标
# ============================================================
def _flatten_coord(val):
    """
    从可能嵌套的 tuple/list 中安全提取 (x, y, z)。
    处理 (x,y,z)、((x,y,z),)、(((x,y,z),),) 等嵌套情况。
    """
    # 尝试直接作为三个数
    try:
        if hasattr(val, '__len__') and len(val) == 3:
            return (float(val[0]), float(val[1]), float(val[2]))
    except (TypeError, ValueError):
        pass

    # 嵌套结构：取第一个元素递归
    try:
        if hasattr(val, '__len__') and len(val) > 0:
            return _flatten_coord(val[0])
    except (TypeError, ValueError):
        pass

    raise ValueError("Cannot flatten coordinate from %r" % (val,))


def extract_coord(picked_obj):
    """
    从 AFXPickStep 选中的对象中提取 (x, y, z) 坐标。
    支持 Vertex、DatumPoint、Node 等，兼容嵌套元组。
    """
    if picked_obj is None:
        return None

    # AFXObjectKeyword + sequenceStyle=TUPLE 可能包了多层 tuple，
    # 循环展开单元素 tuple 直到得到实际对象
    obj = picked_obj
    while True:
        if not hasattr(obj, '__len__'):
            break
        if hasattr(obj, 'pointOn') or hasattr(obj, 'coordinates') \
                or hasattr(obj, 'instanceName'):
            break
        try:
            if len(obj) == 1:
                obj = obj[0]
            else:
                break
        except TypeError:
            break

    # DatumPoint / ReferencePoint: .pointOn
    if hasattr(obj, 'pointOn'):
        try:
            return _flatten_coord(obj.pointOn)
        except Exception:
            pass

    # Node: .coordinates
    if hasattr(obj, 'coordinates'):
        try:
            return _flatten_coord(obj.coordinates)
        except Exception:
            pass

    # Vertex geometry: 通过 instanceName + index 访问实际 vertex
    if hasattr(obj, 'instanceName') and hasattr(obj, 'index'):
        try:
            vpName = session.currentViewportName
            modelName = session.sessionState[vpName]['modelName']
            ass = mdb.models[modelName].rootAssembly
            vert = ass.instances[obj.instanceName].vertices[obj.index]
            if hasattr(vert, 'pointOn'):
                return _flatten_coord(vert.pointOn)
        except Exception:
            pass

    # 直接可索引（对象本身就是坐标序列）
    try:
        return _flatten_coord(obj)
    except Exception:
        pass

    raise ValueError("Cannot extract coordinates from %s" % type(obj))


# ============================================================
# 主函数（由 GUI 端 AFXGuiCommand 调用）
# ============================================================
def align_by_3_points_function(
    kw_moving_instance=None,
    kw_src1=None, kw_src2=None, kw_src3=None,
    kw_tgt1=None, kw_tgt2=None, kw_tgt3=None,
    kw_dry_run=None,
):
    """
    GUI 调用入口。参数名必须与 plugin 文件中 AFXKeyword 的名称一致。
    """
    # ---- 校验 ----
    if kw_moving_instance is None or kw_moving_instance == '':
        getWarningReply(
            message='Please select a moving instance first!',
            buttons=(CANCEL,))
        return

    src_picked = [kw_src1, kw_src2, kw_src3]
    tgt_picked = [kw_tgt1, kw_tgt2, kw_tgt3]

    for i, p in enumerate(src_picked):
        if p is None:
            getWarningReply(
                message='Please pick all 3 source points (missing Src %d)!' % (i + 1),
                buttons=(CANCEL,))
            return
    for i, p in enumerate(tgt_picked):
        if p is None:
            getWarningReply(
                message='Please pick all 3 target points (missing Tgt %d)!' % (i + 1),
                buttons=(CANCEL,))
            return

    # ---- 提取坐标 ----
    try:
        src_points = [extract_coord(p) for p in src_picked]
        tgt_points = [extract_coord(p) for p in tgt_picked]
    except Exception as e:
        getWarningReply(message='Failed to extract coordinates:\n%s' % str(e),
                        buttons=(CANCEL,))
        return

    for p in src_points + tgt_points:
        if p is None:
            getWarningReply(message='Failed to extract coordinates from a picked point.',
                            buttons=(CANCEL,))
            return

    # ---- 获取当前模型 ----
    vpName = session.currentViewportName
    modelName = session.sessionState[vpName]['modelName']

    # ---- 打印信息 ----
    print("=" * 60)
    print("Abaqus 3-Point Alignment")
    print("=" * 60)
    print("Model: %s" % modelName)
    print("Moving instance: %s" % kw_moving_instance)
    for i, (s, tt) in enumerate(zip(src_points, tgt_points)):
        print("  Pt %d: src (%.4f, %.4f, %.4f) -> tgt (%.4f, %.4f, %.4f)"
              % (i + 1, s[0], s[1], s[2], tt[0], tt[1], tt[2]))

    # ---- 计算变换 ----
    R, t = compute_rigid_transform(src_points, tgt_points)
    axis, angle_deg = rotation_matrix_to_axis_angle(R)

    print("\n--- Result ---")
    print("Rotation matrix R:")
    for row in R:
        print("  [% .6f  % .6f  % .6f]" % row)
    print("Translation t: (%.6f, %.6f, %.6f)" % t)
    print("Axis:          (%.6f, %.6f, %.6f)" % axis)
    print("Angle:         %.4f deg" % angle_deg)

    # ---- 精度验证 ----
    print("\n--- Accuracy check ---")
    max_err = 0.0
    for i, s in enumerate(src_points):
        transformed = _v_add(_mat_vec_mul(R, s), t)
        err = _v_norm(_v_sub(transformed, tgt_points[i]))
        if err > max_err:
            max_err = err
        print("  Pt %d: transformed (%.6f, %.6f, %.6f), error = %.2e"
              % (i + 1, transformed[0], transformed[1], transformed[2], err))
    print("Max error: %.2e" % max_err)

    if kw_dry_run:
        print("\n[dry_run] No transform applied.")
        print("=" * 60)
        return

    # ---- 执行变换 ----
    ass = mdb.models[modelName].rootAssembly

    if kw_moving_instance not in ass.instances.keys():
        getWarningReply(
            message="Instance '%s' not found in assembly!" % kw_moving_instance,
            buttons=(CANCEL,))
        return

    print("\n--- Applying transform ---")
    if abs(angle_deg) > 1e-6:
        ass.rotate(
            instanceList=(kw_moving_instance,),
            axisPoint=(0.0, 0.0, 0.0),
            axisDirection=axis,
            angle=angle_deg,
        )
        print("  Rotated: axis=%s, angle=%.4f deg" % (axis, angle_deg))
    else:
        print("  Rotation near zero, skipped")

    if _v_norm(t) > 1e-9:
        ass.translate(
            instanceList=(kw_moving_instance,),
            vector=t,
        )
        print("  Translated: vector=%s" % (t,))
    else:
        print("  Translation near zero, skipped")

    try:
        session.viewports[session.currentViewportName].view.refresh()
    except Exception:
        pass

    print("\nDone! Instance '%s' aligned." % kw_moving_instance)
    print("=" * 60)
