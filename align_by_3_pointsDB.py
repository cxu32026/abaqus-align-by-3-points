# -*- coding: utf-8 -*-
"""
align_by_3_pointsDB.py
======================
三点对齐插件的 GUI 对话框（标准 AFX + FX 组件）。
"""

from abaqusConstants import *
from abaqusGui import *
from kernelAccess import mdb, session
import os

thisPath = os.path.abspath(__file__)
thisDir = os.path.dirname(thisPath)


###########################################################################
# 对话框
###########################################################################
class AlignBy3PointsDB(AFXDataDialog):

    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def __init__(self, form):

        # Construct the base class.
        #
        AFXDataDialog.__init__(self, form, 'Align by 3 Points',
            self.OK | self.APPLY | self.CANCEL, DIALOG_ACTIONS_SEPARATOR)

        okBtn = self.getActionButton(self.ID_CLICKED_OK)
        okBtn.setText('OK')

        applyBtn = self.getActionButton(self.ID_CLICKED_APPLY)
        applyBtn.setText('Apply')

        # ---- 获取当前模型的实例列表 ----
        vpName = session.currentViewportName
        modelName = session.sessionState[vpName]['modelName']
        ass = mdb.models[modelName].rootAssembly
        instance_names = ass.instances.keys()

        # ---- 移动实例选择 ----
        self.instance_combo = AFXComboBox(
            p=self, ncols=20, nvis=1, text='Moving Instance: ',
            tgt=form.kw_moving_instanceKw, sel=0)
        self.instance_combo.setMaxVisible(10)
        for name in instance_names:
            self.instance_combo.appendItem(text=name)
        if len(instance_names) > 0:
            self.instance_combo.setCurrentItem(0)

        # ---- Source Points 分组 ----
        GroupBox_src = FXGroupBox(
            p=self, text='Source Points (on moving instance)',
            opts=FRAME_GROOVE | LAYOUT_FILL_X)

        src_kws = [form.kw_src1Kw, form.kw_src2Kw, form.kw_src3Kw]
        for i in range(3):
            pickHf = FXHorizontalFrame(
                p=GroupBox_src, opts=0, x=0, y=0, w=0, h=0,
                pl=0, pr=0, pt=0, pb=0,
                hs=DEFAULT_SPACING, vs=DEFAULT_SPACING)
            pickHf.setSelector(99)
            label = FXLabel(
                p=pickHf, text='Src %d: (None)' % (i + 1),
                ic=None, opts=LAYOUT_CENTER_Y | JUSTIFY_LEFT)
            pickHandler = AlignBy3PointsDBPickHandler(
                form, src_kws[i],
                'Pick source point %d: ' % (i + 1),
                VERTICES, ONE, label)
            icon = afxGetIcon('select', AFX_ICON_SMALL)
            FXButton(
                p=pickHf, text='\tPick Src %d' % (i + 1), ic=icon,
                tgt=pickHandler, sel=AFXMode.ID_ACTIVATE,
                opts=BUTTON_NORMAL | LAYOUT_CENTER_Y,
                x=0, y=0, w=0, h=0, pl=2, pr=2, pt=1, pb=1)

        # ---- Target Points 分组 ----
        GroupBox_tgt = FXGroupBox(
            p=self, text='Target Points (on fixed instance)',
            opts=FRAME_GROOVE | LAYOUT_FILL_X)

        tgt_kws = [form.kw_tgt1Kw, form.kw_tgt2Kw, form.kw_tgt3Kw]
        for i in range(3):
            pickHf = FXHorizontalFrame(
                p=GroupBox_tgt, opts=0, x=0, y=0, w=0, h=0,
                pl=0, pr=0, pt=0, pb=0,
                hs=DEFAULT_SPACING, vs=DEFAULT_SPACING)
            pickHf.setSelector(99)
            label = FXLabel(
                p=pickHf, text='Tgt %d: (None)' % (i + 1),
                ic=None, opts=LAYOUT_CENTER_Y | JUSTIFY_LEFT)
            pickHandler = AlignBy3PointsDBPickHandler(
                form, tgt_kws[i],
                'Pick target point %d: ' % (i + 1),
                VERTICES, ONE, label)
            icon = afxGetIcon('select', AFX_ICON_SMALL)
            FXButton(
                p=pickHf, text='\tPick Tgt %d' % (i + 1), ic=icon,
                tgt=pickHandler, sel=AFXMode.ID_ACTIVATE,
                opts=BUTTON_NORMAL | LAYOUT_CENTER_Y,
                x=0, y=0, w=0, h=0, pl=2, pr=2, pt=1, pb=1)

        # ---- Dry run ----
        FXCheckButton(
            p=self, text='Dry run (compute only, do not move)',
            tgt=form.kw_dry_runKw, sel=0)

        # ---- 提示 ----
        l = FXLabel(
            p=self,
            text='Note: Always confirm selection with DONE or middle mouse button.',
            opts=JUSTIFY_LEFT)


###########################################################################
# Pick Handler
###########################################################################
class AlignBy3PointsDBPickHandler(AFXProcedure):

    count = 0

    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def __init__(self, form, keyword, prompt, entitiesToPick,
                 numberToPick, label):
        self.form = form
        self.keyword = keyword
        self.prompt = prompt
        self.entitiesToPick = entitiesToPick
        self.numberToPick = numberToPick
        self.label = label
        self.labelText = label.getText()
        AFXProcedure.__init__(self, form.getOwner())
        AlignBy3PointsDBPickHandler.count += 1
        self.setModeName(
            'AlignBy3PointsDBPickHandler%d'
            % (AlignBy3PointsDBPickHandler.count))

    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def getFirstStep(self):
        return AFXPickStep(
            self, self.keyword, self.prompt,
            self.entitiesToPick, self.numberToPick,
            sequenceStyle=TUPLE)

    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def getNextStep(self, previousStep):
        self.label.setText(self.labelText.replace('None', 'Picked'))
        return None
