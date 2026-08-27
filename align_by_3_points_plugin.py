# -*- coding: utf-8 -*-
"""
align_by_3_points_plugin.py
===========================
Abaqus/CAE 插件（GUI 版）：通过交互式拾取 3+3 个点，快速对齐两个零件实例。

## 安装方法
1. 将本文件复制到 Abaqus 插件目录：
   - Windows: C:\\Users\\<用户名>\\abaqus_plugins\\
   - Linux:   ~/abaqus_plugins/
2. 重启 Abaqus/CAE
3. 菜单：Plug-ins -> Align by 3 Points

## 使用流程
1. 选择 Model 和要移动的 Instance
2. 依次点击 "Pick Source 1/2/3"，在视口中拾取移动件上的三个点
3. 依次点击 "Pick Target 1/2/3"，在视口中拾取固定件上的三个对应点
4. （可选）勾选 Dry run 先验证
5. 点击 Apply / OK 执行对齐

## 选点说明
- 支持拾取：几何顶点 (Vertex)、基准点 (Datum Point)、网格节点 (Node)
- 拾取后按 Enter 或视口提示区的 Done 确认
- 三个点不能共线，源点与目标点按顺序一一对应
"""

from abaqus import mdb, session
from abaqusConstants import *
import math

# ============================================================
# GUI 导入（Abaqus GUI 进程）
# ============================================================
try:
    from abaqusGui import *
    from rsg.rsgDialog import RSGDialog
    _HAS_GUI = True
except ImportError:
    _HAS_GUI = False


# ============================================================
# 向量 / 矩阵工具函数（纯 Python，兼容 Abaqus Python 2.7）
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
    angle_deg = math.degrees(angle)
    return axis, angle_deg


# ============================================================
# 主对齐函数
# ============================================================
def align_instance(model_name, moving_instance, source_points,
                   target_points, dry_run=False):
    if len(source_points) != 3 or len(target_points) != 3:
        raise ValueError("Must provide exactly 3 source and 3 target points")

    if model_name not in mdb.models.keys():
        raise ValueError("Model '%s' not found" % model_name)

    model = mdb.models[model_name]
    assembly = model.rootAssembly

    if moving_instance not in assembly.instances.keys():
        raise ValueError("Instance '%s' not found" % moving_instance)

    src = [tuple(float(v) for v in p) for p in source_points]
    tgt = [tuple(float(v) for v in p) for p in target_points]

    print("=" * 60)
    print("Abaqus 3-Point Alignment")
    print("=" * 60)
    print("Moving instance: %s" % moving_instance)
    for i, (s, tt) in enumerate(zip(src, tgt)):
        print("  Pt %d: src (%.4f, %.4f, %.4f) -> tgt (%.4f, %.4f, %.4f)"
              % (i + 1, s[0], s[1], s[2], tt[0], tt[1], tt[2]))

    R, t = compute_rigid_transform(src, tgt)
    axis, angle_deg = rotation_matrix_to_axis_angle(R)

    print("\n--- Result ---")
    print("Rotation matrix R:")
    for row in R:
        print("  [% .6f  % .6f  % .6f]" % row)
    print("Translation t: (%.6f, %.6f, %.6f)" % t)
    print("Axis:          (%.6f, %.6f, %.6f)" % axis)
    print("Angle:         %.4f deg" % angle_deg)

    print("\n--- Accuracy check ---")
    max_err = 0.0
    for i, s in enumerate(src):
        transformed = _v_add(_mat_vec_mul(R, s), t)
        err = _v_norm(_v_sub(transformed, tgt[i]))
        if err > max_err:
            max_err = err
        print("  Pt %d: transformed (%.6f, %.6f, %.6f), error = %.2e"
              % (i + 1, transformed[0], transformed[1], transformed[2], err))
    print("Max error: %.2e" % max_err)

    if dry_run:
        print("\n[dry_run] No transform applied.")
        return {'R': R, 't': t, 'axis': axis, 'angle_deg': angle_deg}

    print("\n--- Applying transform ---")
    if abs(angle_deg) > 1e-6:
        assembly.rotate(
            instanceList=(moving_instance,),
            axisPoint=(0.0, 0.0, 0.0),
            axisDirection=axis,
            angle=angle_deg,
        )
        print("  Rotated: axis=%s, angle=%.4f deg" % (axis, angle_deg))
    else:
        print("  Rotation near zero, skipped")

    if _v_norm(t) > 1e-9:
        assembly.translate(
            instanceList=(moving_instance,),
            vector=t,
        )
        print("  Translated: vector=%s" % (t,))
    else:
        print("  Translation near zero, skipped")

    try:
        session.viewports[session.currentViewportName].view.refresh()
    except Exception:
        pass

    print("\nDone! Instance '%s' aligned." % moving_instance)
    print("=" * 60)
    return {'R': R, 't': t, 'axis': axis, 'angle_deg': angle_deg}


# ============================================================
# 交互式选点
# ============================================================
def extract_coordinates(obj):
    """从选中的对象中提取 (x, y, z) 坐标"""
    # DatumPoint: .pointOn
    if hasattr(obj, 'pointOn'):
        p = obj.pointOn
        return (float(p[0]), float(p[1]), float(p[2]))
    # Node / Vertex: .coordinates
    if hasattr(obj, 'coordinates'):
        p = obj.coordinates
        return (float(p[0]), float(p[1]), float(p[2]))
    # Vertex geometry: .getVertices()
    if hasattr(obj, 'getVertices'):
        verts = obj.getVertices()
        if verts and len(verts) > 0:
            p = verts[0]
            return (float(p[0]), float(p[1]), float(p[2]))
    # 直接可索引
    try:
        return (float(obj[0]), float(obj[1]), float(obj[2]))
    except Exception:
        pass
    raise ValueError("Cannot extract coordinates from %s" % type(obj))


def pick_point_interactive(prompt_text):
    """
    交互式在视口中拾取一个点。
    返回 (x, y, z) 或 None（用户取消）。
    """
    print(prompt_text)
    try:
        from abaqus import getNext
        objects = getNext()
        if objects is not None:
            # getNext 可能返回元组/列表
            if hasattr(objects, '__len__') and len(objects) > 0:
                obj = objects[0]
            else:
                obj = objects
            return extract_coordinates(obj)
    except Exception as e:
        print("Pick error: %s" % str(e))
    return None


# ============================================================
# GUI 对话框
# ============================================================
if _HAS_GUI:

    class AlignBy3PointsDialog(RSGDialog):
        """三点对齐插件主对话框"""

        def __init__(self, parent):
            RSGDialog.__init__(self, parent, 'Align by 3 Points')

            self.source_points = [None, None, None]
            self.target_points = [None, None, None]

            # ---- 模型 & 实例选择 ----
            self.model_combo = RSGComboBox(
                self, 'Model:', self._get_model_names(), 0)
            self.instance_combo = RSGComboBox(
                self, 'Moving Instance:', [], 0)
            RSGButton(self, 'Refresh Instance List',
                      self._on_refresh_instances)

            # ---- 分隔 ----
            RSGLabel(self, '---- Source Points (on moving instance) ----')

            self.src_texts = []
            for i in range(3):
                tf = RSGTextField(self, 'Source %d:' % (i + 1), 40)
                tf.setText('Not selected')
                self.src_texts.append(tf)
                RSGButton(self, 'Pick Source %d' % (i + 1),
                          self._make_pick_callback('source', i))

            # ---- 分隔 ----
            RSGLabel(self, '---- Target Points (on fixed instance) ----')

            self.tgt_texts = []
            for i in range(3):
                tf = RSGTextField(self, 'Target %d:' % (i + 1), 40)
                tf.setText('Not selected')
                self.tgt_texts.append(tf)
                RSGButton(self, 'Pick Target %d' % (i + 1),
                          self._make_pick_callback('target', i))

            # ---- 选项 ----
            self.dry_run_check = RSGCheckBox(
                self, 'Dry run (compute only, do not move)')

            # ---- 重置按钮 ----
            RSGButton(self, 'Reset All Points', self._on_reset)

            # 初始化实例列表
            self._on_refresh_instances()

        # ----------------------------------------------------------
        # 辅助方法
        # ----------------------------------------------------------
        def _get_model_names(self):
            try:
                return list(mdb.models.keys())
            except Exception:
                return ['Model-1']

        def _on_refresh_instances(self):
            model_name = self.model_combo.getCurrentText()
            try:
                assembly = mdb.models[model_name].rootAssembly
                names = list(assembly.instances.keys())
                self.instance_combo.setItems(names)
                if len(names) > 0:
                    self.instance_combo.setCurrentItem(0)
            except Exception as e:
                print("Refresh error: %s" % str(e))

        def _make_pick_callback(self, point_type, index):
            """生成拾取按钮的回调（闭包捕获 index）"""
            def callback():
                self._on_pick_point(point_type, index)
            return callback

        def _on_pick_point(self, point_type, index):
            """拾取一个点：隐藏对话框 -> 选点 -> 重新显示"""
            label = 'Source' if point_type == 'source' else 'Target'
            prompt = ('>>> Pick %s point %d in viewport, '
                      'then press Enter / Done' % (label, index + 1))

            # 隐藏对话框，让用户能操作视口
            self.hide()

            coord = pick_point_interactive(prompt)

            # 重新显示对话框
            self.show()

            if coord is not None:
                coord_str = '(%.4f, %.4f, %.4f)' % coord
                if point_type == 'source':
                    self.source_points[index] = coord
                    self.src_texts[index].setText(coord_str)
                else:
                    self.target_points[index] = coord
                    self.tgt_texts[index].setText(coord_str)
                print("%s %d = %s" % (label, index + 1, coord_str))
            else:
                print("%s %d pick cancelled" % (label, index + 1))

        def _on_reset(self):
            """清除所有已选点"""
            self.source_points = [None, None, None]
            self.target_points = [None, None, None]
            for tf in self.src_texts:
                tf.setText('Not selected')
            for tf in self.tgt_texts:
                tf.setText('Not selected')
            print("All points reset")

        # ----------------------------------------------------------
        # Apply / OK
        # ----------------------------------------------------------
        def onApply(self):
            model_name = self.model_combo.getCurrentText()
            instance_name = self.instance_combo.getCurrentText()
            dry_run = self.dry_run_check.getChecked()

            # 输入校验
            if None in self.source_points:
                AFXErrorMessage(
                    self, 'Please pick all 3 source points first.')
                return
            if None in self.target_points:
                AFXErrorMessage(
                    self, 'Please pick all 3 target points first.')
                return
            if not instance_name:
                AFXErrorMessage(
                    self, 'Please select a moving instance.')
                return

            try:
                align_instance(
                    model_name=model_name,
                    moving_instance=instance_name,
                    source_points=self.source_points,
                    target_points=self.target_points,
                    dry_run=dry_run,
                )
                if not dry_run:
                    AFXInformationMessage(
                        self, 'Alignment completed successfully!\n'
                              'Check the message area for details.')
            except Exception as e:
                AFXErrorMessage(self, 'Alignment failed:\n%s' % str(e))


# ============================================================
# 插件注册入口
# ============================================================
def register():
    """Abaqus 插件入口：在 Plug-ins 菜单中注册并显示对话框"""
    if not _HAS_GUI:
        print("Error: This plugin requires Abaqus/CAE GUI environment.")
        return

    dialog = AlignBy3PointsDialog(AFXGetMainWindow())
    dialog.create()
    dialog.showModal()
