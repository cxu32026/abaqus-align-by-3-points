# -*- coding: utf-8 -*-
"""
align_by_3_points_plugin.py
===========================
Abaqus 插件主入口。
采用标准 AFXForm + registerGuiMenuButton 架构，兼容 Abaqus 2016+。

配套文件（必须放在同一目录）：
  - align_by_3_pointsDB.py      GUI 对话框
  - align_by_3_points_kernel.py kernel 端执行逻辑
"""

from abaqusGui import *
from abaqusConstants import ALL
import osutils, os


###########################################################################
# Class definition
###########################################################################
class AlignBy3Points_plugin(AFXForm):

    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def __init__(self, owner):

        # Construct the base class.
        #
        AFXForm.__init__(self, owner)
        self.radioButtonGroups = {}

        self.cmd = AFXGuiCommand(
            mode=self,
            method='align_by_3_points_function',
            objectName='align_by_3_points_kernel',
            registerQuery=False)

        pickedDefault = ''

        # 移动实例名
        self.kw_moving_instanceKw = AFXStringKeyword(
            self.cmd, 'kw_moving_instance', True, '')

        # 源点（移动件上的3个点）
        self.kw_src1Kw = AFXObjectKeyword(
            self.cmd, 'kw_src1', TRUE, pickedDefault)
        self.kw_src2Kw = AFXObjectKeyword(
            self.cmd, 'kw_src2', TRUE, pickedDefault)
        self.kw_src3Kw = AFXObjectKeyword(
            self.cmd, 'kw_src3', TRUE, pickedDefault)

        # 目标点（固定件上的3个点）
        self.kw_tgt1Kw = AFXObjectKeyword(
            self.cmd, 'kw_tgt1', TRUE, pickedDefault)
        self.kw_tgt2Kw = AFXObjectKeyword(
            self.cmd, 'kw_tgt2', TRUE, pickedDefault)
        self.kw_tgt3Kw = AFXObjectKeyword(
            self.cmd, 'kw_tgt3', TRUE, pickedDefault)

        # Dry run
        self.kw_dry_runKw = AFXBoolKeyword(
            self.cmd, 'kw_dry_run',
            AFXBoolKeyword.TRUE_FALSE, True, False)

    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def getFirstDialog(self):
        import align_by_3_pointsDB
        return align_by_3_pointsDB.AlignBy3PointsDB(self)

    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def doCustomChecks(self):
        # Try to set the appropriate radio button on.
        #
        for kw1, kw2, d in list(self.radioButtonGroups.values()):
            try:
                value = d[kw1.getValue()]
                kw2.setValue(value)
            except:
                pass
        return True

    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def okToCancel(self):
        # No need to close the dialog when a file operation
        # (such as New or Open) or model change is executed.
        #
        return False


#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Register the plug-in
#
thisPath = os.path.abspath(__file__)
thisDir = os.path.dirname(thisPath)

toolset = getAFXApp().getAFXMainWindow().getPluginToolset()
toolset.registerGuiMenuButton(
    buttonText='Align by 3 Points',
    object=AlignBy3Points_plugin(toolset),
    messageId=AFXMode.ID_ACTIVATE,
    icon=None,
    kernelInitString='import align_by_3_points_kernel',
    applicableModules=['Assembly'],
    version='1.0',
    author='AI Assistant',
    description='Align two part instances by matching 3 source points '
                'to 3 target points (rigid transform: rotation + translation). '
                'Pick vertices on each instance, confirm with DONE or '
                'middle mouse button, then press Apply/OK.',
    helpUrl='N/A'
)
