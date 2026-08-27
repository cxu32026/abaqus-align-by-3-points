# -*- coding: utf-8 -*-
"""
align_by_3_pointsDB.py
======================
三点对齐插件的 GUI 对话框（Abaqus AFX 标准组件）。
"""

from abaqusGui import *
from align_by_3_points_kernel import (
    align_instance,
    pick_point_interactive,
    get_model_names,
    get_instance_names,
)


class AlignBy3PointsDialog(AFXDataDialog):
    """三点对齐主对话框"""

    # ---- 自定义消息 ID ----
    ID_PICK_SRC_1 = AFXDataDialog.ID_LAST + 1
    ID_PICK_SRC_2 = AFXDataDialog.ID_LAST + 2
    ID_PICK_SRC_3 = AFXDataDialog.ID_LAST + 3
    ID_PICK_TGT_1 = AFXDataDialog.ID_LAST + 4
    ID_PICK_TGT_2 = AFXDataDialog.ID_LAST + 5
    ID_PICK_TGT_3 = AFXDataDialog.ID_LAST + 6
    ID_REFRESH    = AFXDataDialog.ID_LAST + 7
    ID_RESET      = AFXDataDialog.ID_LAST + 8

    def __init__(self, parent):
        AFXDataDialog.__init__(
            self, parent,
            'Align by 3 Points',
            self.OK | self.CANCEL | self.APPLY,
            DIALOG_ACTIONS_SEPARATOR,
        )

        self.source_points = [None, None, None]
        self.target_points = [None, None, None]

        # ==== 主垂直布局 ====
        main = AFXVerticalFrame(
            self, FRAME_SUNKEN | FRAME_THICK | LAYOUT_FILL_X)

        # ---- 模型 & 实例选择 ----
        selFrame = AFXHorizontalFrame(main, LAYOUT_FILL_X)

        AFXLabel(selFrame, 0, 'Model:')
        self.model_combo = AFXComboBox(selFrame, 15, 10)
        for name in get_model_names():
            self.model_combo.appendItem(name)
        if self.model_combo.getNumItems() > 0:
            self.model_combo.setCurrentItem(0)

        AFXLabel(selFrame, 0, '   Moving Instance:')
        self.instance_combo = AFXComboBox(selFrame, 20, 10)

        refresh_btn = AFXButton(selFrame, 'Refresh')
        refresh_btn.setTarget(self)
        refresh_btn.setSelector(self.ID_REFRESH)

        # ---- Source Points 分组 ----
        src_group = AFXGroupBox(main, 'Source Points (on moving instance)')
        self.src_texts = []
        src_ids = [self.ID_PICK_SRC_1, self.ID_PICK_SRC_2, self.ID_PICK_SRC_3]
        for i in range(3):
            row = AFXHorizontalFrame(src_group, LAYOUT_FILL_X)
            AFXLabel(row, 0, 'Src %d:' % (i + 1))
            tf = AFXTextField(row, 30)
            tf.setText('Not selected')
            tf.setEditable(FALSE)
            self.src_texts.append(tf)
            btn = AFXButton(row, 'Pick Src %d' % (i + 1))
            btn.setTarget(self)
            btn.setSelector(src_ids[i])

        # ---- Target Points 分组 ----
        tgt_group = AFXGroupBox(main, 'Target Points (on fixed instance)')
        self.tgt_texts = []
        tgt_ids = [self.ID_PICK_TGT_1, self.ID_PICK_TGT_2, self.ID_PICK_TGT_3]
        for i in range(3):
            row = AFXHorizontalFrame(tgt_group, LAYOUT_FILL_X)
            AFXLabel(row, 0, 'Tgt %d:' % (i + 1))
            tf = AFXTextField(row, 30)
            tf.setText('Not selected')
            tf.setEditable(FALSE)
            self.tgt_texts.append(tf)
            btn = AFXButton(row, 'Pick Tgt %d' % (i + 1))
            btn.setTarget(self)
            btn.setSelector(tgt_ids[i])

        # ---- 选项 ----
        optFrame = AFXHorizontalFrame(main, LAYOUT_FILL_X)
        self.dry_run_check = AFXCheckButton(
            optFrame, 'Dry run (compute only, do not move)')
        self.dry_run_check.setCheck(FALSE)

        reset_btn = AFXButton(optFrame, 'Reset All')
        reset_btn.setTarget(self)
        reset_btn.setSelector(self.ID_RESET)

        # 初始化实例列表
        self._refresh_instances()

    # ================================================================
    # 辅助方法
    # ================================================================
    def _refresh_instances(self):
        """根据当前选中的模型刷新实例下拉框"""
        idx = self.model_combo.getCurrentItem()
        if idx < 0:
            return
        model_name = self.model_combo.getItemText(idx)
        self.instance_combo.clearItems()
        for name in get_instance_names(model_name):
            self.instance_combo.appendItem(name)
        if self.instance_combo.getNumItems() > 0:
            self.instance_combo.setCurrentItem(0)

    def _do_pick(self, point_type, index):
        """拾取一个点：隐藏对话框 -> 选点 -> 重新显示"""
        label = 'Source' if point_type == 'source' else 'Target'
        prompt = ('>>> Pick %s point %d in viewport, '
                  'then press Enter / Done' % (label, index + 1))

        self.hide()
        coord = pick_point_interactive(prompt)
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

    def _reset_all(self):
        """清除所有已选点"""
        self.source_points = [None, None, None]
        self.target_points = [None, None, None]
        for tf in self.src_texts:
            tf.setText('Not selected')
        for tf in self.tgt_texts:
            tf.setText('Not selected')
        print("All points reset")

    # ================================================================
    # 按钮回调
    # ================================================================
    def onPickSrc1(self, sender, sel, ptr):
        self._do_pick('source', 0)
        return 1

    def onPickSrc2(self, sender, sel, ptr):
        self._do_pick('source', 1)
        return 1

    def onPickSrc3(self, sender, sel, ptr):
        self._do_pick('source', 2)
        return 1

    def onPickTgt1(self, sender, sel, ptr):
        self._do_pick('target', 0)
        return 1

    def onPickTgt2(self, sender, sel, ptr):
        self._do_pick('target', 1)
        return 1

    def onPickTgt3(self, sender, sel, ptr):
        self._do_pick('target', 2)
        return 1

    def onRefresh(self, sender, sel, ptr):
        self._refresh_instances()
        return 1

    def onReset(self, sender, sel, ptr):
        self._reset_all()
        return 1

    # ================================================================
    # Apply / OK
    # ================================================================
    def onApply(self):
        """点击 Apply 或 OK 时执行对齐"""
        # 获取模型名
        midx = self.model_combo.getCurrentItem()
        if midx < 0:
            AFXErrorMessage(self, 'Please select a model.')
            return 0
        model_name = self.model_combo.getItemText(midx)

        # 获取实例名
        iidx = self.instance_combo.getCurrentItem()
        if iidx < 0:
            AFXErrorMessage(self, 'Please select a moving instance.')
            return 0
        instance_name = self.instance_combo.getItemText(iidx)

        dry_run = (self.dry_run_check.getCheck() == TRUE)

        # 校验点
        if None in self.source_points:
            AFXErrorMessage(self, 'Please pick all 3 source points first.')
            return 0
        if None in self.target_points:
            AFXErrorMessage(self, 'Please pick all 3 target points first.')
            return 0

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
                    self,
                    'Alignment completed!\nCheck message area for details.',
                )
        except Exception as e:
            AFXErrorMessage(self, 'Alignment failed:\n%s' % str(e))
            return 0

        return 1

    # ================================================================
    # 消息映射
    # ================================================================
    FXMAPFUNC(SEL_COMMAND, ID_PICK_SRC_1,
              'AlignBy3PointsDialog.onPickSrc1')
    FXMAPFUNC(SEL_COMMAND, ID_PICK_SRC_2,
              'AlignBy3PointsDialog.onPickSrc2')
    FXMAPFUNC(SEL_COMMAND, ID_PICK_SRC_3,
              'AlignBy3PointsDialog.onPickSrc3')
    FXMAPFUNC(SEL_COMMAND, ID_PICK_TGT_1,
              'AlignBy3PointsDialog.onPickTgt1')
    FXMAPFUNC(SEL_COMMAND, ID_PICK_TGT_2,
              'AlignBy3PointsDialog.onPickTgt2')
    FXMAPFUNC(SEL_COMMAND, ID_PICK_TGT_3,
              'AlignBy3PointsDialog.onPickTgt3')
    FXMAPFUNC(SEL_COMMAND, ID_REFRESH,
              'AlignBy3PointsDialog.onRefresh')
    FXMAPFUNC(SEL_COMMAND, ID_RESET,
              'AlignBy3PointsDialog.onReset')
