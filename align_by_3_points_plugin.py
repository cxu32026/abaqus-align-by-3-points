# -*- coding: utf-8 -*-
"""
align_by_3_points_plugin.py
===========================
Abaqus 插件主入口文件。
Abaqus/CAE 启动时会扫描 abaqus_plugins 目录，识别本文件中的 register() 函数，
并在 Plug-ins 菜单中创建对应条目。

配套文件（必须与本文件放在同一目录）：
  - align_by_3_pointsDB.py      GUI 对话框定义
  - align_by_3_points_kernel.py 核心算法与选点逻辑
"""

from abaqusGui import *
from align_by_3_pointsDB import AlignBy3PointsDialog


def register():
    """插件注册入口：创建并显示三点对齐对话框"""
    dialog = AlignBy3PointsDialog(AFXGetMainWindow())
    dialog.create()
    dialog.showModal()
