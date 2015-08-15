#!/usr/bin/env python

######################################################################
# Rice University Software Distribution License
#
# Copyright (c) 2010, Rice University
# All Rights Reserved.
#
# For a full description see the file named LICENSE.
#
######################################################################

# Author: Mark Moll

import sys
from os.path import abspath, dirname, join
from decimal import Decimal
import inspect
qt5 = False
try:
    # try PyQt4 first
    from PyQt4 import QtGui, QtCore, QtGui, QtOpenGL
    from PyQt4.QtCore import pyqtSignal as Signal
    QtWidgets = QtGui
except:
    try:
        from PyQt5 import QtWidgets, QtCore, QtGui, QtOpenGL
        from PyQt5.QtCore import pyqtSignal as Signal
        qt5 = True
    except:
        # if PyQt* wasn't found, try PySide
        from PySide import QtCore, QtGui, QtOpenGL
        from PySide.QtCore import Signal
        QtWidgets = QtGui
import OpenGL
OpenGL.ERROR_CHECKING = False
from OpenGL import GL, GLU
import webbrowser, re
from math import cos, sin, asin, acos, atan2, pi, pow, ceil, sqrt
# The ConfigParser module has been renamed to configparser in Python 3.0
try:
    import ConfigParser
except:
    import configparser as ConfigParser

sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'ompl/py-bindings' ) )
from ompl.util import OutputHandler, useOutputHandler, LogLevel, OMPL_DEBUG, OMPL_INFORM, OMPL_WARN, OMPL_ERROR
from ompl import base as ob
from ompl import geometric as og
from ompl import control as oc
from ompl import app as oa

import ompl
ompl.initializePlannerLists()

# wrapper for API change in PyQt
def getOpenFileNameAsAstring(qtwindow, title="", directory="", ffilter=""):
    fname = QtWidgets.QFileDialog.getOpenFileName(qtwindow, title, directory, ffilter)
    return str(fname[0]) if isinstance(fname, tuple) else str(fname)

# wrapper for API change in PyQt
def getSaveFileNameAsAstring(qtwindow, title="", directory=""):
    fname = QtWidgets.QFileDialog.getSaveFileName(qtwindow, title, directory)
    return str(fname[0]) if isinstance(fname, tuple) else str(fname)

class LogOutputHandler(OutputHandler):
    def __init__(self, textEdit):
        super(LogOutputHandler, self).__init__()
        self.textEdit = textEdit
        self.redColor = QtGui.QColor(255, 0, 0)
        self.orangeColor = QtGui.QColor(255, 128, 0)
        self.greenColor = QtGui.QColor(0, 255, 64)
        self.blackColor = QtGui.QColor(0, 0, 0)

    def log(self, text, level, filename, line):
        if level == LogLevel.LOG_DEBUG:
            self.debug(text)
        elif level == LogLevel.LOG_INFO:
            self.inform(text)
        elif level == LogLevel.LOG_WARN:
            self.warn(text)
        elif level == LogLevel.LOG_ERROR:
            self.error(text)
        else:
            print(text)

    def debug(self, text):
        self.textEdit.setTextColor(self.greenColor)
        self.textEdit.append(text)
        self.textEdit.repaint()

    def inform(self, text):
        self.textEdit.setTextColor(self.blackColor)
        self.textEdit.append(text)
        self.textEdit.repaint()

    def warn(self, text):
        self.textEdit.setTextColor(self.orangeColor)
        self.textEdit.append(text)
        self.textEdit.repaint()

    def error(self, text):
        self.textEdit.setTextColor(self.redColor)
        self.textEdit.append(text)
        self.textEdit.repaint()

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        QtWidgets.QMainWindow.__init__(self)
        self.createActions()
        self.createMenus()
        self.createRobotTypeList()
        self.mainWidget = MainWidget(self.robotTypes)
        self.setCentralWidget(self.mainWidget)
        self.setWindowTitle('OMPL.app')
        self.environmentFile = None
        self.robotFile = None
        self.path = None
        self.mainWidget.problemWidget.robotTypeSelect.currentIndexChanged[int].connect(self.setRobotType)
        self.mainWidget.solveWidget.solveButton.clicked.connect(self.solve)
        self.mainWidget.solveWidget.clearButton.clicked.connect(self.clear)
        self.mainWidget.boundsWidget.resetButton.clicked.connect(self.resetBounds)

        # connect timeLimit widgets in geometric and control planning with each other and with the
        # MainWindow.setTimeLimit method
        self.mainWidget.plannerWidget.geometricPlanning.timeLimit.valueChanged[float].connect(self.setTimeLimit)
        self.mainWidget.plannerWidget.geometricPlanning.timeLimit.valueChanged[float].connect(
            self.mainWidget.plannerWidget.controlPlanning.timeLimit.setValue)
        self.mainWidget.plannerWidget.controlPlanning.timeLimit.valueChanged[float].connect(self.setTimeLimit)
        self.mainWidget.plannerWidget.controlPlanning.timeLimit.valueChanged[float].connect(
            self.mainWidget.plannerWidget.geometricPlanning.timeLimit.setValue)
        self.timeLimit = self.mainWidget.plannerWidget.geometricPlanning.timeLimit.value()

        self.mainWidget.boundsWidget.bounds_low.valueChanged.connect(self.mainWidget.glViewer.setLowerBound)
        self.mainWidget.boundsWidget.bounds_high.valueChanged.connect(self.mainWidget.glViewer.setUpperBound)
        self.mainWidget.glViewer.boundLowChanged.connect(self.mainWidget.boundsWidget.bounds_low.setBounds)
        self.mainWidget.glViewer.boundHighChanged.connect(self.mainWidget.boundsWidget.bounds_high.setBounds)

        #self.commandWindow = CommandWindow(self) # not implemented yet
        robotType = [t[0] for t in self.robotTypes].index('GSE3RigidBodyPlanning')
        self.mainWidget.problemWidget.robotTypeSelect.setCurrentIndex(robotType)

        # connect to OMPL's console output (via OutputHandlers)
        self.logWindow = LogWindow(self)
        self.oh = LogOutputHandler(self.logWindow.logView)
        useOutputHandler(self.oh)

    def openEnvironment(self):
        fname = getOpenFileNameAsAstring(self, "Open Environment")
        if len(fname)>0 and fname!=self.environmentFile:
            self.environmentFile = fname
            self.omplSetup.setEnvironmentMesh(self.environmentFile)
            self.mainWidget.glViewer.setEnvironment(self.omplSetup.renderEnvironment())
            self.resetBounds()
            if self.isGeometric:
                if self.robotFile:
                    self.mainWidget.plannerWidget.geometricPlanning.resolution.setValue(
                        self.omplSetup.getSpaceInformation().getStateValidityCheckingResolution())

    def openRobot(self):
        fname = getOpenFileNameAsAstring(self, "Open Robot")
        if len(fname)>0 and fname!=self.robotFile:
            self.robotFile = fname
            self.omplSetup.setRobotMesh(self.robotFile)
            self.mainWidget.glViewer.setRobot(self.omplSetup.renderRobot())
            self.omplSetup.inferEnvironmentBounds()
            # full state
            start = self.omplSetup.getDefaultStartState()
            # just the first geometric component
            start = self.omplSetup.getGeometricComponentState(start,0)
            self.mainWidget.problemWidget.setStartPose(start, self.is3D)
            self.mainWidget.problemWidget.setGoalPose(start, self.is3D)
            if self.isGeometric:
                self.mainWidget.plannerWidget.geometricPlanning.resolution.setValue(
                    self.omplSetup.getSpaceInformation().getStateValidityCheckingResolution())
            self.mainWidget.glViewer.setBounds(self.omplSetup.getGeometricComponentStateSpace().getBounds())

    def openConfig(self):
        fname = getOpenFileNameAsAstring(self, "Open Problem Configuration", "", "*.cfg")
        if len(fname)>0:
            OMPL_INFORM("Loading " + fname)
            if (sys.version_info > (3, 0)):
                config = ConfigParser.ConfigParser(strict = False)
            else:
                config = ConfigParser.ConfigParser()
            config.readfp(open(fname, 'r'))
            if config.has_option("problem", "start.z"):
                if config.has_option("problem", "control"):
                    ctype = config.get("problem", "control")
                    if ctype == "blimp":
                        robotType = [t[0] for t in self.robotTypes].index('GBlimpPlanning')
                    elif ctype == "quadrotor":
                        robotType = [t[0] for t in self.robotTypes].index('GQuadrotorPlanning')
                    else:
                        robotType = [t[0] for t in self.robotTypes].index('GSE3RigidBodyPlanning')
                else:
                    robotType = [t[0] for t in self.robotTypes].index('GSE3RigidBodyPlanning')
                self.mainWidget.problemWidget.robotTypeSelect.setCurrentIndex(robotType)
            else:
                if config.has_option("problem", "control"):
                    ctype = config.get("problem", "control")
                    if ctype == "kinematic_car":
                        robotType = [t[0] for t in self.robotTypes].index('GKinematicCarPlanning')
                    elif ctype == "dynamic_car":
                        robotType = [t[0] for t in self.robotTypes].index('GDynamicCarPlanning')
                    else:
                        robotType = [t[0] for t in self.robotTypes].index('GSE2RigidBodyPlanning')
                else:
                    robotType = [t[0] for t in self.robotTypes].index('GSE2RigidBodyPlanning')
                self.mainWidget.problemWidget.robotTypeSelect.setCurrentIndex(robotType)
            cfg_dir = dirname(fname)
            self.setRobotType(robotType)
            self.environmentFile = join(cfg_dir, config.get("problem", "world"))
            self.robotFile = join(cfg_dir, config.get("problem", "robot"))
            self.omplSetup.setEnvironmentMesh(self.environmentFile)
            self.omplSetup.setRobotMesh(self.robotFile)
            self.mainWidget.glViewer.setEnvironment(self.omplSetup.renderEnvironment())
            self.mainWidget.glViewer.setRobot(self.omplSetup.renderRobot())
            self.resetBounds()
            if self.isGeometric:
                self.mainWidget.plannerWidget.geometricPlanning.resolution.setValue(
                    self.omplSetup.getSpaceInformation().getStateValidityCheckingResolution())
            start = ob.State(self.omplSetup.getGeometricComponentStateSpace())
            goal = ob.State(self.omplSetup.getGeometricComponentStateSpace())
            if self.is3D:
                start().setX(config.getfloat("problem", "start.x"))
                start().setY(config.getfloat("problem", "start.y"))
                start().setZ(config.getfloat("problem", "start.z"))
                start().rotation().setAxisAngle(config.getfloat("problem", "start.axis.x"),
                                                config.getfloat("problem", "start.axis.y"),
                                                config.getfloat("problem", "start.axis.z"),
                                                config.getfloat("problem", "start.theta"))
                goal().setX(config.getfloat("problem", "goal.x"))
                goal().setY(config.getfloat("problem", "goal.y"))
                goal().setZ(config.getfloat("problem", "goal.z"))
                goal().rotation().setAxisAngle(config.getfloat("problem", "goal.axis.x"),
                                               config.getfloat("problem", "goal.axis.y"),
                                               config.getfloat("problem", "goal.axis.z"),
                                               config.getfloat("problem", "goal.theta"))
            else:
                start().setX(config.getfloat("problem", "start.x"))
                start().setY(config.getfloat("problem", "start.y"))
                start().setYaw(config.getfloat("problem", "start.theta"))
                goal().setX(config.getfloat("problem", "goal.x"))
                goal().setY(config.getfloat("problem", "goal.y"))
                goal().setYaw(config.getfloat("problem", "goal.theta"))
            self.mainWidget.problemWidget.setStartPose(start, self.is3D)
            self.mainWidget.problemWidget.setGoalPose(goal, self.is3D)
            if self.is3D:
                if (config.has_option("problem", "volume.min.x") and config.has_option("problem", "volume.min.y") and config.has_option("problem", "volume.min.z") and
                    config.has_option("problem", "volume.max.x") and config.has_option("problem", "volume.max.y") and config.has_option("problem", "volume.max.z")):
                    bounds = ob.RealVectorBounds(3)
                    bounds.low[0] = config.getfloat("problem", "volume.min.x")
                    bounds.low[1] = config.getfloat("problem", "volume.min.y")
                    bounds.low[2] = config.getfloat("problem", "volume.min.z")
                    bounds.high[0] = config.getfloat("problem", "volume.max.x")
                    bounds.high[1] = config.getfloat("problem", "volume.max.y")
                    bounds.high[2] = config.getfloat("problem", "volume.max.z")
                    self.omplSetup.getGeometricComponentStateSpace().setBounds(bounds)
                    self.mainWidget.glViewer.setBounds(bounds)
            else:
                if (config.has_option("problem", "volume.min.x") and config.has_option("problem", "volume.min.y") and
                    config.has_option("problem", "volume.max.x") and config.has_option("problem", "volume.max.y")):
                    bounds = ob.RealVectorBounds(2)
                    bounds.low[0] = config.getfloat("problem", "volume.min.x")
                    bounds.low[1] = config.getfloat("problem", "volume.min.y")
                    bounds.high[0] = config.getfloat("problem", "volume.max.x")
                    bounds.high[1] = config.getfloat("problem", "volume.max.y")
                    self.omplSetup.getGeometricComponentStateSpace().setBounds(bounds)
                    self.mainWidget.glViewer.setBounds(bounds)
            if config.has_option("problem", "objective"):
                ind = self.mainWidget.problemWidget.objectiveSelect.findText(
                    config.get("problem", "objective").replace("_", " "))
                if ind != -1:
                    self.mainWidget.problemWidget.objectiveSelect.setCurrentIndex(ind)
                    if config.has_option("problem", "objective.threshold"):
                        self.mainWidget.problemWidget.objectiveThreshold.setValue(
                            config.getfloat("problem", "objective.threshold"))

    def saveConfig(self):
        fname = getSaveFileNameAsAstring(self, 'Save Problem Configuration', 'config.cfg')
        if len(fname)>0:
            config = ConfigParser.ConfigParser()
            config.add_section("problem")
            config.set("problem", "robot", self.robotFile)
            config.set("problem", "world", self.environmentFile)
            startPose = self.mainWidget.problemWidget.getStartPose()
            goalPose = self.mainWidget.problemWidget.getGoalPose()
            config.set("problem", "objective",
                str(self.mainWidget.problemWidget.objectiveSelect.currentText()).replace(" ", "_"))
            config.set("problem", "objective.threshold", str(self.mainWidget.problemWidget.objectiveThreshold.value()))
            ctype = str(self.mainWidget.problemWidget.robotTypeSelect.currentText())
            if not ctype.startswith('Rigid body planning'):
                if ctype == "Blimp":
                    config.set("problem", "control", "blimp")
                elif ctype == "Quadrotor":
                    config.set("problem", "control", "quadrotor")
                elif ctype == "Dynamic car":
                    config.set("problem", "control", "dynamic_car")
                elif ctype == "Kinematic car":
                    config.set("problem", "control", "kinematic_car")
            b = self.omplSetup.getGeometricComponentStateSpace().getBounds()
            if self.is3D:
                config.set("problem", "start.x", str(startPose().getX()))
                config.set("problem", "start.y", str(startPose().getY()))
                config.set("problem", "start.z", str(startPose().getZ()))
                rs = startPose().rotation()
                if rs.w==1:
                    config.set("problem", "start.theta",  "0")
                    config.set("problem", "start.axis.x", "1")
                    config.set("problem", "start.axis.y", "0")
                    config.set("problem", "start.axis.z", "0")
                else:
                    config.set("problem", "start.theta", str(2 * acos(rs.w)))
                    ds = sqrt(1.0 - rs.w * rs.w)
                    config.set("problem", "start.axis.x", str(rs.x / ds))
                    config.set("problem", "start.axis.y", str(rs.y / ds))
                    config.set("problem", "start.axis.z", str(rs.z / ds))
                config.set("problem", "goal.x", str(goalPose().getX()))
                config.set("problem", "goal.y", str(goalPose().getY()))
                config.set("problem", "goal.z", str(goalPose().getZ()))
                rg = goalPose().rotation()
                if rg.w==1:
                    config.set("problem", "goal.theta",  "0")
                    config.set("problem", "goal.axis.x", "1")
                    config.set("problem", "goal.axis.y", "0")
                    config.set("problem", "goal.axis.z", "0")
                else:
                    config.set("problem", "goal.theta", str(2 * acos(rg.w)))
                    dg = sqrt(1.0 - rg.w * rg.w)
                    config.set("problem", "goal.axis.x", str(rg.x / dg))
                    config.set("problem", "goal.axis.y", str(rg.y / dg))
                    config.set("problem", "goal.axis.z", str(rg.z / dg))
                config.set("problem", "volume.min.x", str(b.low[0]))
                config.set("problem", "volume.min.y", str(b.low[1]))
                config.set("problem", "volume.min.z", str(b.low[2]))
                config.set("problem", "volume.max.x", str(b.high[0]))
                config.set("problem", "volume.max.y", str(b.high[1]))
                config.set("problem", "volume.max.z", str(b.high[2]))
            else:
                config.set("problem", "start.x", str(startPose().getX()))
                config.set("problem", "start.y", str(startPose().getY()))
                config.set("problem", "start.theta", str(startPose().getYaw()))
                config.set("problem", "goal.x", str(goalPose().getX()))
                config.set("problem", "goal.y", str(goalPose().getY()))
                config.set("problem", "goal.theta", str(goalPose().getYaw()))
                config.set("problem", "volume.min.x", str(b.low[0]))
                config.set("problem", "volume.min.y", str(b.low[1]))
                config.set("problem", "volume.max.x", str(b.high[0]))
                config.set("problem", "volume.max.y", str(b.high[1]))
            with open(fname, 'w') as configfile:
                config.write(configfile)
            OMPL_INFORM("Saved " + fname)

    def _arrayToSE2State(self, a):
        st = ob.State(self.omplSetup.getGeometricComponentStateSpace())
        st().setXY(a[0],a[1])
        st().setYaw(a[2])
        return st
    def _arrayToSE3State(self, a):
        st = ob.State(self.omplSetup.getGeometricComponentStateSpace())
        st().setXYZ(a[0], a[1], a[2])
        R = st().rotation()
        nrm = 1./sqrt(a[3]*a[3] + a[4]*a[4] + a[5]*a[5] + a[6]*a[6])
        (R.x, R.y, R.z, R.w) = (a[3]*nrm, a[4]*nrm, a[5]*nrm, a[6]*nrm)
        return st
    def openPath(self):
        fname = getOpenFileNameAsAstring(self, "Open Path")
        if len(fname)>0:
            path = []
            for line in open(fname,'r').readlines():
                l = line.strip()
                if len(l) == 0:
                    continue
                path.append([float(x) for x in l.split(' ')])

            self.mainWidget.glViewer.solutionPath = []
            # assume that first 3 components map to SE(2) if 3<len<7
            if len(path[0]) > 2 and len(path[0]) < 7:
                self.path = [self._arrayToSE2State(s) for s in path]
            # assume that first 7 components map to SE(3) if len>=7
            elif len(path[0]) >= 7:
                self.path = [self._arrayToSE3State(s) for s in path]
            else:
                # unknown state type
                OMPL_Error("Wrong state format")
                raise ValueError
            self.mainWidget.glViewer.setSolutionPath(self.path)
            # setStart/GoalPose can change bounds, so save and restore them
            bounds = self.mainWidget.glViewer.getBounds()
            self.mainWidget.problemWidget.setStartPose(self.path[0], self.is3D)
            self.mainWidget.problemWidget.setGoalPose(self.path[-1], self.is3D)
            self.mainWidget.glViewer.setBounds(bounds)

    def savePath(self):
        if self.path:
            fname = getSaveFileNameAsAstring(self, 'Save Path', 'path.txt')
            if len(fname)>0:
                ind = range(7 if self.is3D else 3)
                pathstr = '\n'.join([ ' '.join([str(s[i]) for i in ind]) for s in self.path])
                open(fname,'w').write(pathstr)

    def savePlannerData(self):
        fname = getSaveFileNameAsAstring(self, 'Save Roadmap/Tree', 'plannerData.graphml')
        if len(fname)>0:
            pd = ob.PlannerData(self.omplSetup.getSpaceInformation())
            self.omplSetup.getPlannerData(pd)
            pd.computeEdgeWeights()
            with open(fname, 'w') as outfile:
                outfile.write(pd.printGraphML())
                outfile.close()

    def setSolutionPath(self, path):
            ns = len(path.getStates())
            self.path = [ self.omplSetup.getGeometricComponentState(
                ob.State(self.omplSetup.getGeometricComponentStateSpace(),path.getState(i)), 0) for i in range(ns) ]
            self.mainWidget.glViewer.setSolutionPath(self.path)

    def randMotion(self):
        self.configureApp()

        if self.isGeometric:
            pg = og.PathGeometric(self.omplSetup.getSpaceInformation())
            if pg.randomValid(100):
                pg.interpolate()
                self.setSolutionPath(pg)
            else:
                OMPL_Error("Unable to generate random valid path")
        else:
            pc = oc.PathControl(self.omplSetup.getSpaceInformation())
            if pc.randomValid(100):
                self.setSolutionPath(pc.asGeometric())
            else:
                OMPL_Error("Unable to generate random valid path")

    def showLogWindow(self):
        if self.logWindow.isHidden():
            self.logWindow.show()
            self.logWindow.raise_()
            self.logWindow.activateWindow()
        else:
            self.logWindow.hide()
            self.mainWidget.raise_()
            self.mainWidget.activateWindow()

    def showCommandWindow(self):
        self.commandWindow.show()
        self.commandWindow.raise_()
        self.commandWindow.activateWindow()

    def omplWebSite(self):
        webbrowser.open('http://ompl.kavrakilab.org')
    def contactDevs(self):
        webbrowser.open('mailto:ompl-devel@lists.sourceforge.net')
    def emailList(self):
        webbrowser.open('mailto:ompl-users@lists.sourceforge.net')

    def createPlanner(self):
        si = self.omplSetup.getSpaceInformation()
        if self.isGeometric:
            plannerid = self.mainWidget.plannerWidget.geometricPlanning.plannerSelect.currentIndex()
            plannerParams = self.mainWidget.plannerWidget.geometricPlanning.plannerList[plannerid]
        else:
            plannerid = self.mainWidget.plannerWidget.controlPlanning.plannerSelect.currentIndex()
            plannerParams = self.mainWidget.plannerWidget.controlPlanning.plannerList[plannerid]
        if plannerParams[0].startswith('ompl.control.Syclop'):
            decomposition = self.omplSetup.allocDecomposition()
            planner = eval('%s(si,decomposition)' % plannerParams[0])
        else:
            planner = eval('%s(si)' % plannerParams[0])
        params = planner.params()
        for (param,widget) in plannerParams[1].items():
            if isinstance(widget, QtWidgets.QCheckBox):
                params[param].setValue('1' if widget.isChecked() else '0')
            elif isinstance(widget, QtWidgets.QComboBox):
                if self.isGeometric:
                    val = og.planners.getPlanners()[plannerParams[0]][param][2][widget.currentIndex()]
                else:
                    val = oc.planners.getPlanners()[plannerParams[0]][param][2][widget.currentIndex()]
                params[param].setValue(val)
            else:
                params[param].setValue(str(widget.value()))
        return planner

    def setRobotType(self, value):
        self.environmentFile = None
        self.robotFile = None
        self.path = None
        self.omplSetup = eval('oa.%s()' % self.robotTypes[value][0])
        self.clear(True)
        self.isGeometric = self.robotTypes[value][2]==oa.GEOMETRIC
        self.is3D = isinstance(self.omplSetup.getGeometricComponentStateSpace(), ob.SE3StateSpace)
        self.mainWidget.plannerWidget.setCurrentIndex(0 if self.isGeometric else 1)
        self.mainWidget.problemWidget.poses.setCurrentIndex(0 if self.is3D else 1)

    def setTimeLimit(self, value):
        OMPL_DEBUG('Changing time limit from %g to %g' % (self.timeLimit, value))
        self.timeLimit = value

    def configureApp(self):
        self.omplSetup.clear()
        startPose = self.omplSetup.getFullStateFromGeometricComponent(self.mainWidget.problemWidget.getStartPose())
        goalPose = self.omplSetup.getFullStateFromGeometricComponent(self.mainWidget.problemWidget.getGoalPose())
        self.omplSetup.setPlanner(self.createPlanner())
        if self.is3D:
            bounds = ob.RealVectorBounds(3)
            (bounds.low[0],bounds.low[1],bounds.low[2]) = self.mainWidget.glViewer.bounds_low
            (bounds.high[0],bounds.high[1],bounds.high[2]) = self.mainWidget.glViewer.bounds_high
        else:
            bounds = ob.RealVectorBounds(2)
            (bounds.low[0],bounds.low[1]) = self.mainWidget.glViewer.bounds_low[:2]
            (bounds.high[0],bounds.high[1]) = self.mainWidget.glViewer.bounds_high[:2]
        self.omplSetup.getGeometricComponentStateSpace().setBounds(bounds)
        if self.isGeometric:
            self.omplSetup.setStartAndGoalStates(startPose, goalPose, 1e-6)
            self.omplSetup.getSpaceInformation().setStateValidityCheckingResolution(
                self.mainWidget.plannerWidget.geometricPlanning.resolution.value())
        else:
            self.omplSetup.setStartAndGoalStates(startPose, goalPose, .1)
            self.omplSetup.getSpaceInformation().setPropagationStepSize(
                self.mainWidget.plannerWidget.controlPlanning.propagation.value())
            self.omplSetup.getSpaceInformation().setMinMaxControlDuration(
                self.mainWidget.plannerWidget.controlPlanning.minControlDuration.value(),
                self.mainWidget.plannerWidget.controlPlanning.maxControlDuration.value())
        self.omplSetup.setOptimizationObjectiveAndThreshold(
            str(self.mainWidget.problemWidget.objectiveSelect.currentText()),
            self.mainWidget.problemWidget.objectiveThreshold.value())
        self.omplSetup.setup()

    def solve(self):
        self.configureApp()
        OMPL_DEBUG(str(self.omplSetup))

        solved = self.omplSetup.solve(self.timeLimit)

        # update the planner data to render, if needed
        pd = ob.PlannerData(self.omplSetup.getSpaceInformation())
        self.omplSetup.getPlannerData(pd)
        self.mainWidget.glViewer.plannerDataList = self.omplSetup.renderPlannerData(pd)

        # update the displayed bounds, in case planning did so
        self.mainWidget.glViewer.setBounds(self.omplSetup.getGeometricComponentStateSpace().getBounds())
        if solved:
            if self.isGeometric:
                path = self.omplSetup.getSolutionPath()
                initialValid = path.check()
                if initialValid == False:
                    OMPL_Error("Path reported by planner seems to be invalid!")
                self.omplSetup.simplifySolution()
                path = self.omplSetup.getSolutionPath()
                if initialValid == True and path.check() == False:
                    OMPL_Error("Simplified path seems to be invalid!")
            else:
                path = self.omplSetup.getSolutionPath().asGeometric()
                if path.check() == False:
                    OMPL_Error("Path reported by planner seems to be invalid!")

            ns = int(100.0 * float(path.length()) / float(self.omplSetup.getStateSpace().getMaximumExtent()))
            if self.isGeometric and len(path.getStates()) < ns:
                OMPL_DEBUG("Interpolating solution path to " + str(ns) + " states")
                path.interpolate(ns)
                if len(path.getStates()) != ns:
                    OMPL_Error("Interpolation produced " + str(len(path.getStates())) + " states instead of " + str(ns) + " states!")
#            if path.check() == False:
#                OMPL_Error("Something wicked happened to the path during interpolation")
            self.setSolutionPath(path)

    def clear(self, deepClean=False):
        self.omplSetup.clear()
        self.mainWidget.glViewer.clear(deepClean)

    def resetBounds(self):
        if self.is3D:
            b = ob.RealVectorBounds(3)
            self.omplSetup.getGeometricComponentStateSpace().setBounds(b)
        else:
            b = ob.RealVectorBounds(2)
            self.omplSetup.getGeometricComponentStateSpace().setBounds(b)
        self.omplSetup.inferEnvironmentBounds()
        self.mainWidget.glViewer.setBounds(self.omplSetup.getGeometricComponentStateSpace().getBounds())

    def createActions(self):
        self.openEnvironmentAct = QtWidgets.QAction('Open &Environment', self,
            shortcut='Ctrl+E', statusTip='Open an environment model',
            triggered=self.openEnvironment)
        self.openRobotAct = QtWidgets.QAction('Open &Robot', self,
            shortcut='Ctrl+R', statusTip='Open a robot model',
            triggered=self.openRobot)
        self.openConfigAct = QtWidgets.QAction('Open Problem &Configuration', self,
            shortcut='Ctrl+O', statusTip='Open a problem configuration (.cfg file)',
            triggered=self.openConfig)
        self.saveConfigAct = QtWidgets.QAction('Save Problem Con&figuration', self,
            shortcut='Ctrl+S', statusTip='Save a problem configuration (.cfg file)',
            triggered=self.saveConfig)
        self.openPathAct = QtWidgets.QAction('&Open Path', self,
            shortcut='Ctrl+Alt+O', statusTip='Open a path',
            triggered=self.openPath)
        self.savePathAct = QtWidgets.QAction('Save &Path', self,
            shortcut='Ctrl+Alt+S', statusTip='Save a path',
            triggered=self.savePath)
        self.savePlannerDataAct = QtWidgets.QAction('Save Roadmap/Tree', self,
            statusTip='Save the roadmap/tree that was created by "Solve"',
            triggered=self.savePlannerData)
        self.exitAct = QtWidgets.QAction('E&xit', self, shortcut='Ctrl+Q',
            statusTip='Exit the application', triggered=self.close)

        self.logWindowAct = QtWidgets.QAction('Log Window', self,
            shortcut='Ctrl+1', triggered=self.showLogWindow)
        self.randMotionAct = QtWidgets.QAction('Random &Motion', self, shortcut='Ctrl+M', triggered=self.randMotion)
        self.commandWindowAct = QtWidgets.QAction('Command Window', self,
            shortcut='Ctrl+2', triggered=self.showCommandWindow)

        self.omplWebAct = QtWidgets.QAction('OMPL Web Site', self,
            triggered=self.omplWebSite)
        self.contactDevsAct = QtWidgets.QAction('Contact Developers', self,
            triggered=self.contactDevs)
        self.emailListAct = QtWidgets.QAction('Email OMPL Mailing List', self,
            triggered=self.emailList)

    def createMenus(self):
        self.fileMenu = self.menuBar().addMenu('&File')
        self.fileMenu.addAction(self.openEnvironmentAct)
        self.fileMenu.addAction(self.openRobotAct)
        self.fileMenu.addAction(self.openConfigAct)
        self.fileMenu.addAction(self.saveConfigAct)
        self.fileMenu.addAction(self.openPathAct)
        self.fileMenu.addAction(self.savePathAct)
        self.fileMenu.addAction(self.savePlannerDataAct)
        self.fileMenu.addSeparator()
        self.fileMenu.addAction(self.exitAct)

        self.toolsMenu = self.menuBar().addMenu('&Tools')
        self.toolsMenu.addAction(self.logWindowAct)
        self.toolsMenu.addAction(self.randMotionAct)
        # self.windowMenu.addAction(self.commandWindowAct)

        self.helpMenu = self.menuBar().addMenu('Help')
        self.helpMenu.addAction(self.omplWebAct)
        self.helpMenu.addAction(self.contactDevsAct)
        self.helpMenu.addAction(self.emailListAct)

    def createRobotTypeList(self):
        from inspect import isclass
        self.robotTypes = []
        for c in dir(oa):
            if eval('isclass(oa.%s) and issubclass(oa.%s, (oa.AppBaseGeometric,oa.AppBaseControl)) and issubclass(oa.%s, oa.RenderGeometry)' % (c,c,c)):
                name = eval('oa.%s().getName()' % c)
                apptype = eval('oa.%s().getAppType()' % c)
                self.robotTypes.append((c, name, apptype))


class MainWidget(QtWidgets.QWidget):
    def __init__(self, robotTypes, parent=None, flags=QtCore.Qt.WindowFlags(0)):
        super(MainWidget, self).__init__(parent, flags)
        self.glViewer = GLViewer()
        self.problemWidget = ProblemWidget(robotTypes)
        self.plannerWidget = PlannerWidget()
        self.boundsWidget = BoundsWidget()
        self.solveWidget = SolveWidget()
        self.solveWidget.setSizePolicy(QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed))
        tabWidget = QtWidgets.QTabWidget()
        tabWidget.addTab(self.problemWidget, "Problem")
        tabWidget.addTab(self.plannerWidget, "Planner")
        tabWidget.addTab(self.boundsWidget, "Bounding box")
        tabWidget.setSizePolicy(QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed))
        layout = QtWidgets.QGridLayout()
        layout.addWidget(self.glViewer, 0, 0, 2, 1)
        layout.addWidget(tabWidget, 0, 1)
        layout.addWidget(self.solveWidget, 2, 0, 1, 2)
        self.setLayout(layout)
        self.problemWidget.startChanged.connect(self.glViewer.setStartPose)
        self.problemWidget.goalChanged.connect(self.glViewer.setGoalPose)
        self.solveWidget.explorationVizSelect.currentIndexChanged[int].connect(self.glViewer.showPlannerData)
        self.solveWidget.animateCheck.toggled.connect(self.glViewer.toggleAnimation)
        self.solveWidget.speedSlider.valueChanged.connect(self.glViewer.setSpeed)

class LogWindow(QtWidgets.QWidget):
    def __init__(self, parent=None, flags=QtCore.Qt.Tool):
        super(LogWindow, self).__init__(parent, flags)
        self.setWindowTitle('OMPL Log')
        self.resize(640, 320)
        self.logView = QtWidgets.QTextEdit(self)
        self.logView.setReadOnly(True)
        layout = QtWidgets.QGridLayout()
        layout.addWidget(self.logView, 0, 0)
        self.setLayout(layout)

class CommandWindow(QtWidgets.QWidget):
    def __init__(self, parent=None, flags=QtCore.Qt.Tool):
        super(CommandWindow, self).__init__(parent, flags)

class GLViewer(QtOpenGL.QGLWidget):
    boundLowChanged = Signal(list)
    boundHighChanged = Signal(list)

    def __init__(self, parent=None):
        super(GLViewer, self).__init__(parent)
        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.lastPos = QtCore.QPoint()
        self.environment = None
        self.robot = None
        self.center = [0,0,0]
        self.scale = 1
        self.viewheight = 1
        self.cameraPose = [0,0,0,0,0,0]
        self.startPose = [0,0,0,0,0,0]
        self.goalPose = [0,0,0,0,0,0]
        self.solutionPath = None
        self.pathIndex = 0
        self.timer = QtCore.QTimer()
        self.timer.start(100.)
        self.timer.timeout.connect(self.updatePathPose)
        self.animate = True
        self.drawPlannerData = False
        self.plannerDataList = None
        self.bounds_low = None
        self.bounds_high = None
        #self.elevation = 0

    def minimumSizeHint(self):
        return QtCore.QSize(500, 300)

    def setRotationAngle(self, axisIndex, angle):
        if angle != self.cameraPose[axisIndex]:
            self.cameraPose[axisIndex] = angle
            self.updateGL()
    def getBounds(self):
        bounds = ob.RealVectorBounds(3)
        for i in range(3):
            bounds.low[i] = self.bounds_low[i]
            bounds.high[i] = self.bounds_high[i]
        return bounds
    def setBounds(self, bounds):
        self.bounds_low = [x for x in bounds.low ]
        self.bounds_high = [x for x in bounds.high ]
        if len(self.bounds_low)==2:
            self.bounds_low.append(0)
            self.bounds_high.append(0)
        bbox = list(zip(self.bounds_low, self.bounds_high))
        self.center = [ .5*(p0+p1) for (p0,p1) in bbox ]
        m = max([p1-p0 for (p0,p1) in bbox ])
        self.scale = 1. if m==0 else 1. / m
        self.viewheight = (self.bounds_high[2]-self.bounds_low[2])*self.scale*3
        self.boundLowChanged.emit(self.bounds_low)
        self.boundHighChanged.emit(self.bounds_high)
    def updateBounds(self, pos):
        lo=False
        hi=False
        if self.bounds_low == None:
            self.bounds_low = pos
            self.bounds_high = pos
            self.boundLowChanged.emit(self.bounds_low)
            self.boundHighChanged.emit(self.bounds_high)
        else:
            for i in range(3):
                if pos[i]<self.bounds_low[i]:
                    self.bounds_low[i] = pos[i]
                    lo = True
                elif pos[i]>self.bounds_high[i]:
                    self.bounds_high[i] = pos[i]
                    hi = True
            if lo: self.boundLowChanged.emit(self.bounds_low)
            if hi: self.boundHighChanged.emit(self.bounds_high)
    def setLowerBound(self, bound):
        self.bounds_low = bound
        self.updateGL()
    def setUpperBound(self, bound):
        self.bounds_high = bound
        self.updateGL()
    def setStartPose(self, value):
        self.startPose = value
        self.updateBounds(value[3:])
        self.updateGL()
    def setGoalPose(self, value):
        self.goalPose = value
        self.updateBounds(value[3:])
        self.updateGL()
    def showPlannerData(self, value):
        self.drawPlannerData = value
        self.updateGL()
    def toggleAnimation(self, value):
        self.animate = value
        if self.animate:
            self.timer.start()
        else:
            self.timer.stop()
        self.updateGL()
    def setSpeed(self, value):
        if value==0:
            self.timer.stop()
        else:
            self.timer.start(100.0/float(value))
            self.updatePathPose()
    def updatePathPose(self):
        if self.solutionPath != None:
            self.pathIndex = (self.pathIndex + 1) % len(self.solutionPath)
            self.updateGL()
    def setSolutionPath(self, path):
        self.solutionPath = [ self.getTransform(state()) for state in path ]
        self.pathIndex = 0
        self.updateGL()
    def setRobot(self, robot):
        if self.robot: GL.glDeleteLists(self.robot, 1)
        self.robot = robot
    def setEnvironment(self, environment):
        if self.environment: GL.glDeleteLists(self.environment, 1)
        self.environment = environment
    def clear(self, deepClean=False):
        self.solutionPath = None
        self.plannerDataList = None
        self.pathIndex = 0
        if deepClean:
            self.setRobot(None)
            self.setEnvironment(None)
            self.bounds_low = None
            self.bounds_high = None
            self.cameraPose = [0,0,0,0,0,0]
        self.updateGL()
    def initializeGL(self):
        GL.glClearColor(0.5,0.5,0.5,1.)
        GL.glEnable(GL.GL_LIGHTING)
        GL.glEnable(GL.GL_LIGHT0)
        GL.glEnable(GL.GL_LIGHT1)
        GL.glEnable(GL.GL_DEPTH_TEST)
        GL.glLightModeli(GL.GL_LIGHT_MODEL_TWO_SIDE, GL.GL_TRUE)
        GL.glEnable(GL.GL_NORMALIZE)
        GL.glColorMaterial(GL.GL_FRONT_AND_BACK, GL.GL_DIFFUSE)
        GL.glEnable(GL.GL_LINE_SMOOTH)
        GL.glShadeModel(GL.GL_FLAT)
        GL.glEnable(GL.GL_BLEND)
        GL.glBlendFunc(GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA)
        #GL.glEnable(GL.GL_CULL_FACE)

    def transform(self, pose):
        GL.glPushMatrix()
        GL.glTranslatef(pose[3], pose[4], pose[5])
        GL.glRotated(pose[0], 1.0, 0.0, 0.0)
        GL.glRotated(pose[1], 0.0, 1.0, 0.0)
        GL.glRotated(pose[2], 0.0, 0.0, 1.0)
    def getTransform(self, xform):
        if hasattr(xform,'rotation'):
            R = xform.rotation()
            (w,x,y,z) = (R.w, -R.x, -R.y, -R.z)
            return [ w*w+x*x-y*y-z*z, 2*(x*y-w*z), 2*(x*z+w*y), 0,
                2*(x*y+w*z), w*w-x*x+y*y-z*z, 2*(y*z-w*x), 0,
                2*(x*z-w*y), 2*(y*z+w*x), w*w-x*x-y*y+z*z, 0,
                xform.getX(), xform.getY(), xform.getZ(), 1 ]
        else:
            th = -xform.getYaw()
            return [ cos(th), -sin(th), 0, 0,
                sin(th), cos(th), 0, 0,
                0, 0, 1, 0,
                xform.getX(), xform.getY(), 0, 1 ]

    def drawBounds(self):
        lo = self.bounds_low
        hi = self.bounds_high
        p = [lo, [lo[0],lo[1],hi[2]], [lo[0],hi[1],lo[2]], [lo[0],hi[1],hi[2]],
            [hi[0],lo[1],lo[2]], [hi[0],lo[1],hi[2]], [hi[0],hi[1],lo[2]], hi]
        ind = [(0,1),(1,3),(3,2),(2,0),(4,5),(5,7),(7,6),(6,4),(0,4),(1,5),(2,6),(3,7)]
        GL.glDisable(GL.GL_LIGHTING)
        GL.glDisable(GL.GL_COLOR_MATERIAL)
        GL.glColor3f(1,1,1)
        GL.glBegin(GL.GL_LINES)
        for edge in ind:
            GL.glVertex3fv(p[edge[0]])
            GL.glVertex3fv(p[edge[1]])
        GL.glEnd()

    def paintGL(self):
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
        GL.glMatrixMode(GL.GL_MODELVIEW)
        GL.glLoadIdentity()
        GLU.gluLookAt(0,0,3, 0,0,-5, 0,1,0)
        #GLU.gluLookAt(1,1,2, -1,-1,-3, 0,1,0)
        self.transform(self.cameraPose)
        GL.glScalef(self.scale, self.scale, self.scale)
        GL.glTranslatef(-self.center[0],-self.center[1],-self.center[2])
        # draw bounding box
        if self.bounds_low:
            self.drawBounds()
        if (self.robot):
            # draw start pose
            self.transform(self.startPose)
            GL.glCallList(self.robot)
            GL.glPopMatrix()

            # draw path pose(s)
            if self.solutionPath != None:
                if self.animate:
                    GL.glPushMatrix()
                    GL.glMultMatrixf(self.solutionPath[self.pathIndex])
                    GL.glCallList(self.robot)
                    GL.glPopMatrix()
                else:
                    n = len(self.solutionPath)
                    nmax = 100
                    if n < nmax:
                        ind = range(0,n)
                    else:
                        step = float(n - 1.)/float(nmax - 1)
                        ind = [int(step*i) for i in range(nmax)]
                    for i in ind:
                        GL.glPushMatrix()
                        GL.glMultMatrixf(self.solutionPath[i])
                        GL.glCallList(self.robot)
                        GL.glPopMatrix()

            # draw goal pose
            self.transform(self.goalPose)
            GL.glCallList(self.robot)
            GL.glPopMatrix()

        # draw environment
        if self.environment: GL.glCallList(self.environment)

        # draw the planner data
        if self.drawPlannerData and self.plannerDataList:
            GL.glCallList(self.drawPlannerData + self.plannerDataList - 1)

        GL.glPopMatrix()

    def resizeGL(self, width, height):
        side = min(width, height)
        if side <= 0:
            return
        GL.glMatrixMode(GL.GL_PROJECTION)
        GL.glLoadIdentity()
        GLU.gluPerspective(45., float(width) / float(height), 1., 1000.)
        GL.glViewport(0, 0, width, height)

    def mousePressEvent(self, event):
        self.lastPos = event.pos()

    def mouseMoveEvent(self, event):
        dx = event.x() - self.lastPos.x()
        dy = event.y() - self.lastPos.y()
        buttons = event.buttons()
        modifiers = event.modifiers()
        if buttons & QtCore.Qt.LeftButton and not (modifiers & QtCore.Qt.META):
            if modifiers & QtCore.Qt.SHIFT:
                self.center[0] = self.center[0] + dx/self.scale
                self.center[1] = self.center[1] + dy/self.scale
                self.updateGL()
            else:
                self.setRotationAngle(0, self.cameraPose[0] + dy)
                self.setRotationAngle(1, self.cameraPose[1] + dx)
        elif buttons & QtCore.Qt.RightButton or \
            (buttons & QtCore.Qt.LeftButton and modifiers & QtCore.Qt.META):
            self.setRotationAngle(0, self.cameraPose[0] + dy)
            self.setRotationAngle(2, self.cameraPose[2] + dx)
        elif buttons & QtCore.Qt.MidButton:
            if dy>0:
                self.scale = self.scale*(1. + .01*dy)
            else:
                self.scale = self.scale*(1. - .01*dy)
        self.lastPos = event.pos()

    def wheelEvent(self, event):
        if qt5:
            self.scale = self.scale * pow(2.0, -event.angleDelta().y() / 240.0)
        else:
            self.scale = self.scale * pow(2.0, -event.delta() / 240.0)
        self.lastPos = event.pos()
        self.updateGL()

class ProblemWidget(QtWidgets.QWidget):
    startChanged = Signal(list)
    goalChanged = Signal(list)

    def __init__(self, robotTypes):
        super(ProblemWidget, self).__init__()
        robotTypeLabel =  QtWidgets.QLabel('Robot type')
        self.robotTypeSelect = QtWidgets.QComboBox()
        for robotType in robotTypes:
            self.robotTypeSelect.addItem(robotType[1])
        self.robotTypeSelect.setMaximumSize(200, 2000)

        self.startPose3D = Pose3DBox('Start pose')
        self.goalPose3D = Pose3DBox('Goal pose')
        self.startPose2D = Pose2DBox('Start pose')
        self.goalPose2D = Pose2DBox('Goal pose')

        elevation2Dlabel = QtWidgets.QLabel('Elevation')
        self.elevation2D = QtWidgets.QDoubleSpinBox()
        self.elevation2D.setRange(-1000, 1000)
        self.elevation2D.setSingleStep(1)

        startGoal3D = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.startPose3D)
        layout.addWidget(self.goalPose3D)
        startGoal3D.setLayout(layout)

        startGoal2D = QtWidgets.QWidget()
        layout = QtWidgets.QGridLayout()
        layout.addWidget(self.startPose2D, 0, 0, 1, 2)
        layout.addWidget(self.goalPose2D, 1, 0, 1 ,2)
        #layout.addWidget(elevation2Dlabel, 2, 0, QtCore.Qt.AlignRight)
        #layout.addWidget(self.elevation2D, 2, 1)
        startGoal2D.setLayout(layout)

        self.poses = QtWidgets.QStackedWidget()
        self.poses.addWidget(startGoal3D)
        self.poses.addWidget(startGoal2D)

        self.objectives = {'length': 'PathLengthOptimizationObjective',
            'max min clearance': 'MaximizeMinClearanceObjective',
            'mechanical work': 'MechanicalWorkOptimizationObjective'
        }
        self.objectiveSelect = QtWidgets.QComboBox()
        self.objectiveSelect.addItems(sorted(self.objectives.keys()))
        self.objectiveThreshold = QtWidgets.QDoubleSpinBox()
        self.objectiveThreshold.setRange(0, 10000)
        self.objectiveThreshold.setSingleStep(1)
        self.objectiveThreshold.setValue(10000)

        layout = QtWidgets.QGridLayout()
        layout.addWidget(robotTypeLabel, 0, 0)
        layout.addWidget(self.robotTypeSelect, 0, 1)
        layout.addWidget(self.poses, 1, 0, 1, 2)
        layout.addWidget(QtWidgets.QLabel('Opt. objective'), 2, 0)
        layout.addWidget(self.objectiveSelect, 2, 1)
        layout.addWidget(QtWidgets.QLabel('Cost threshold'), 3, 0)
        layout.addWidget(self.objectiveThreshold, 3, 1)
        self.setLayout(layout)

        self.startPose3D.valueChanged.connect(self.startPoseChange)
        self.goalPose3D.valueChanged.connect(self.goalPoseChange)
        self.startPose2D.valueChanged.connect(self.startPoseChange)
        self.goalPose2D.valueChanged.connect(self.goalPoseChange)
        self.elevation2D.valueChanged.connect(self.elevationChange)

    def setStartPose(self, value, is3D):
        self.startPose2D.setPose(value, is3D)
        # if is3D:
        #     self.elevation2D.setValue(value().getZ())
        self.startPose3D.setPose(value, self.elevation2D.value(), is3D)
    def getStartPose(self):
        return self.startPose3D.getPose() if self.poses.currentIndex()==0 else self.startPose2D.getPose()
    def getGoalPose(self):
        return self.goalPose3D.getPose() if self.poses.currentIndex()==0 else self.goalPose2D.getPose()
    def startPoseChange(self, value):
        if self.poses.currentIndex()==1: value[5] = self.elevation2D.value()
        self.startChanged.emit(value)
    def setGoalPose(self, value, is3D):
        self.goalPose2D.setPose(value, is3D)
        # if is3D:
        #     self.elevation2D.setValue(value().getZ())
        self.goalPose3D.setPose(value, self.elevation2D.value(), is3D)
    def goalPoseChange(self, value):
        if self.poses.currentIndex()==1: value[5] = self.elevation2D.value()
        self.goalChanged.emit(value)
    def elevationChange(self, value):
        state = [ 0, 0, self.startPose2D.rot.value(), self.startPose2D.posx.value(), self.startPose2D.posy.value(), value ]
        self.startChanged.emit(state)
        state = [ 0, 0, self.goalPose2D.rot.value(), self.goalPose2D.posx.value(), self.goalPose2D.posy.value(), value ]
        self.goalChanged.emit(state)
    def getObjective(self, si):
        obj = eval('ob.%s(si)' % self.objectives[self.objectiveSelect.currentText()])
        obj.setCostThreshold(self.objectiveThreshold.value())
        return obj

class Pose3DBox(QtWidgets.QGroupBox):
    valueChanged = Signal(list)

    def __init__(self, title):
        super(Pose3DBox, self).__init__(title)
        xlabel = QtWidgets.QLabel('X')
        ylabel = QtWidgets.QLabel('Y')
        zlabel = QtWidgets.QLabel('Z')
        poslabel = QtWidgets.QLabel('Position')
        rotlabel = QtWidgets.QLabel('Rotation')

        self.posx = QtWidgets.QDoubleSpinBox()
        self.posx.setRange(-1000, 1000)
        self.posx.setSingleStep(1)
        self.posy = QtWidgets.QDoubleSpinBox()
        self.posy.setRange(-1000, 1000)
        self.posy.setSingleStep(1)
        self.posz = QtWidgets.QDoubleSpinBox()
        self.posz.setRange(-1000, 1000)
        self.posz.setSingleStep(1)
        self.rotx = QtWidgets.QDoubleSpinBox()
        self.rotx.setRange(-360,360)
        self.rotx.setSingleStep(1)
        self.roty = QtWidgets.QDoubleSpinBox()
        self.roty.setRange(-360,360)
        self.roty.setSingleStep(1)
        self.rotz = QtWidgets.QDoubleSpinBox()
        self.rotz.setRange(-360,360)
        self.rotz.setSingleStep(1)

        layout = QtWidgets.QGridLayout()
        layout.addWidget(poslabel, 0, 1, QtCore.Qt.AlignHCenter | QtCore.Qt.AlignBottom)
        layout.addWidget(rotlabel, 0, 2, QtCore.Qt.AlignHCenter | QtCore.Qt.AlignBottom)
        layout.addWidget(xlabel, 1, 0, QtCore.Qt.AlignRight)
        layout.addWidget(ylabel, 2, 0, QtCore.Qt.AlignRight)
        layout.addWidget(zlabel, 3, 0, QtCore.Qt.AlignRight)
        layout.addWidget(self.posx, 1, 1)
        layout.addWidget(self.posy, 2, 1)
        layout.addWidget(self.posz, 3, 1)
        layout.addWidget(self.rotx, 1, 2)
        layout.addWidget(self.roty, 2, 2)
        layout.addWidget(self.rotz, 3, 2)
        self.setLayout(layout)

        self.posx.valueChanged.connect(self.poseChange)
        self.posy.valueChanged.connect(self.poseChange)
        self.posz.valueChanged.connect(self.poseChange)
        self.rotx.valueChanged.connect(self.poseChange)
        self.roty.valueChanged.connect(self.poseChange)
        self.rotz.valueChanged.connect(self.poseChange)

    def setPose(self, value, elevation, is3D):
        state = value()
        if is3D:
            self.posx.setValue(state.getX())
            self.posy.setValue(state.getY())
            self.posz.setValue(state.getZ())
            q = state.rotation()
            rad2deg = 180/pi
            self.rotx.setValue(rad2deg * atan2(2.*(q.w*q.x+q.y*q.z), 1.-2.*(q.x*q.x+q.y*q.y)))
            self.roty.setValue(rad2deg * asin(max(min(2.*(q.w*q.y-q.z*q.x),1.),-1.)))
            self.rotz.setValue(rad2deg * atan2(2.*(q.w*q.z+q.x*q.y), 1.-2.*(q.y*q.y+q.z*q.z)))
        else:
            self.posx.setValue(state.getX())
            self.posy.setValue(state.getY())
            self.posz.setValue(elevation)
            self.rotx.setValue(0)
            self.roty.setValue(0)
            self.rotz.setValue(180 * state.getYaw() / pi)

    def getPose(self):
        state = ob.State(ob.SE3StateSpace())
        state().setX(self.posx.value())
        state().setY(self.posy.value())
        state().setZ(self.posz.value())
        angles = [self.rotx.value(), self.roty.value(), self.rotz.value()]
        c = [ cos(angle*pi/360.) for angle in angles ]
        s = [ sin(angle*pi/360.) for angle in angles ]
        rot = state().rotation()
        rot.w = c[0]*c[1]*c[2] - s[0]*s[1]*s[2]
        rot.x = s[0]*c[1]*c[2] + c[0]*s[1]*s[2]
        rot.y = c[0]*s[1]*c[2] - s[0]*c[1]*s[2]
        rot.z = c[0]*c[1]*s[2] + s[0]*s[1]*c[2]
        return state

    def poseChange(self, value):
        self.valueChanged.emit([self.rotx.value(), self.roty.value(), self.rotz.value(),
            self.posx.value(), self.posy.value(), self.posz.value() ])

class Pose2DBox(QtWidgets.QGroupBox):
    valueChanged = Signal(list)

    def __init__(self, title):
        super(Pose2DBox, self).__init__(title)
        xlabel = QtWidgets.QLabel('X')
        ylabel = QtWidgets.QLabel('Y')
        rotlabel = QtWidgets.QLabel('Rotation')

        self.posx = QtWidgets.QDoubleSpinBox()
        self.posx.setRange(-1000, 1000)
        self.posx.setSingleStep(1)
        self.posy = QtWidgets.QDoubleSpinBox()
        self.posy.setRange(-1000, 1000)
        self.posy.setSingleStep(1)
        self.rot = QtWidgets.QDoubleSpinBox()
        self.rot.setRange(-360,360)
        self.rot.setSingleStep(1)

        layout = QtWidgets.QGridLayout()
        layout.addWidget(xlabel, 0, 0, QtCore.Qt.AlignRight)
        layout.addWidget(ylabel, 1, 0, QtCore.Qt.AlignRight)
        layout.addWidget(rotlabel, 2, 0, QtCore.Qt.AlignRight)
        layout.addWidget(self.posx, 0, 1)
        layout.addWidget(self.posy, 1, 1)
        layout.addWidget(self.rot, 2, 1)
        self.setLayout(layout)

        self.posx.valueChanged.connect(self.poseChange)
        self.posy.valueChanged.connect(self.poseChange)
        self.rot.valueChanged.connect(self.poseChange)

    def setPose(self, value, is3D):
        state = value()
        if is3D:
            self.posx.setValue(state.getX())
            self.posy.setValue(state.getY())
            q = state.rotation()
            self.rot.setValue(atan2(2.*(q.w*q.z+q.x*q.y), 1.-2.*(q.y*q.y+q.z*q.z)) * 180 / pi )
        else:
            self.posx.setValue(state.getX())
            self.posy.setValue(state.getY())
            self.rot.setValue(state.getYaw() * 180 / pi)

    def getPose(self):
        state = ob.State(ob.SE2StateSpace())
        state().setX(self.posx.value())
        state().setY(self.posy.value())
        state().setYaw(self.rot.value() * pi / 180)
        return state

    def poseChange(self, value):
        self.valueChanged.emit([0, 0, self.rot.value(), self.posx.value(), self.posy.value(), 0 ])

class PlannerHelperWidget(QtWidgets.QGroupBox):
    def __init__(self, name, planners):
        super(PlannerHelperWidget, self).__init__(name)
        self.setFlat(True)
        self.plannerSelect = QtWidgets.QComboBox()
        self.stackedWidget = QtWidgets.QStackedWidget()

        self.plannerList = []
        for planner, params in sorted(planners.items()):
            displayName = planner.split('.')[-1]
            self.plannerSelect.addItem(displayName)
            options = QtWidgets.QGroupBox('%s options' % displayName)
            layout = QtWidgets.QGridLayout()
            i = 0
            paramDict = {}
            for (key,val) in sorted(params.items()):
                label = QtWidgets.QLabel(val[0])
                if val[1] == ompl.PlanningAlgorithms.BOOL:
                    widget = QtWidgets.QCheckBox()
                    widget.setChecked(val[3])
                elif val[1] == ompl.PlanningAlgorithms.ENUM:
                    widget = QtWidgets.QComboBox()
                    widget.addItems(val[2])
                    widget.setCurrentIndex(val[3])
                elif val[1] == ompl.PlanningAlgorithms.INT:
                    widget = QtWidgets.QSpinBox()
                    widget.setRange(val[2][0], val[2][2])
                    widget.setSingleStep(val[2][1])
                    widget.setValue(val[3])
                elif val[1] == ompl.PlanningAlgorithms.DOUBLE:
                    widget = QtWidgets.QDoubleSpinBox()
                    numDecimals = max([-Decimal(str(v)).as_tuple().exponent for v in val[2]])
                    if numDecimals < 2:
                        numDecimals = 2
                    elif numDecimals > 5:
                        numDecimals = 5
                    widget.setDecimals(numDecimals)
                    widget.setRange(val[2][0], val[2][2])
                    widget.setSingleStep(val[2][1])
                    widget.setValue(val[3])
                else:
                    print("Warning: parameter of unknown type ignored!")
                    continue
                layout.addWidget(label, i, 0, QtCore.Qt.AlignRight)
                layout.addWidget(widget, i, 1)
                i = i + 1
                paramDict[key] = widget
            options.setLayout(layout)
            self.stackedWidget.addWidget(options)
            self.plannerList.append((planner,paramDict))

        self.plannerSelect.setSizeAdjustPolicy(QtWidgets.QComboBox.AdjustToMinimumContentsLength)
        self.plannerSelect.currentIndexChanged[int].connect(self.stackedWidget.setCurrentIndex)

class GeometricPlannerWidget(PlannerHelperWidget):
    def __init__(self):
        super(GeometricPlannerWidget, self).__init__('Geometric planning', og.planners.getPlanners())

        # make KPIECE1 the default planner
        self.plannerSelect.setCurrentIndex([p[0] for p in self.plannerList].index('ompl.geometric.KPIECE1'))

        timeLimitLabel = QtWidgets.QLabel('Time (sec.)')
        self.timeLimit = QtWidgets.QDoubleSpinBox()
        self.timeLimit.setRange(0, 10000)
        self.timeLimit.setSingleStep(1)
        self.timeLimit.setValue(10.0)

        resolutionLabel = QtWidgets.QLabel('Collision checking\nresolution')
        resolutionLabel.setAlignment(QtCore.Qt.AlignRight)
        self.resolution = QtWidgets.QDoubleSpinBox()
        self.resolution.setRange(0.001, 1.0)
        self.resolution.setSingleStep(.002)
        self.resolution.setValue(0.010)
        self.resolution.setDecimals(3)

        layout = QtWidgets.QGridLayout()
        layout.addWidget(QtWidgets.QLabel('Planner'), 0, 0, QtCore.Qt.AlignRight)
        layout.addWidget(self.plannerSelect, 0, 1)
        layout.addWidget(timeLimitLabel, 1, 0, QtCore.Qt.AlignRight)
        layout.addWidget(self.timeLimit, 1, 1)
        layout.addWidget(resolutionLabel, 2, 0, QtCore.Qt.AlignRight)
        layout.addWidget(self.resolution, 2, 1)
        layout.addWidget(self.stackedWidget, 3, 0, 1, 2)
        self.setLayout(layout)

class ControlPlannerWidget(PlannerHelperWidget):
    def __init__(self):
        super(ControlPlannerWidget, self).__init__('Planning with controls', oc.planners.getPlanners())

        # make KPIECE1 the default planner
        self.plannerSelect.setCurrentIndex([p[0] for p in self.plannerList].index('ompl.control.KPIECE1'))

        timeLimitLabel = QtWidgets.QLabel('Time (sec.)')
        self.timeLimit = QtWidgets.QDoubleSpinBox()
        self.timeLimit.setRange(0, 1000)
        self.timeLimit.setSingleStep(1)
        self.timeLimit.setValue(10.0)

        propagationLabel = QtWidgets.QLabel('Propagation\nstep size')
        propagationLabel.setAlignment(QtCore.Qt.AlignRight)
        self.propagation = QtWidgets.QDoubleSpinBox()
        self.propagation.setRange(0.01, 1000.00)
        self.propagation.setSingleStep(.01)
        self.propagation.setValue(0.2)
        self.propagation.setDecimals(2)

        durationLabel = QtWidgets.QLabel('Control duration\n(min/max #steps)')
        durationLabel.setAlignment(QtCore.Qt.AlignRight)
        self.minControlDuration = QtWidgets.QSpinBox()
        self.minControlDuration.setRange(1, 1000)
        self.minControlDuration.setSingleStep(1)
        self.minControlDuration.setValue(1)
        self.maxControlDuration = QtWidgets.QSpinBox()
        self.maxControlDuration.setRange(1, 1000)
        self.maxControlDuration.setSingleStep(1)
        self.maxControlDuration.setValue(20)

        layout = QtWidgets.QGridLayout()
        layout.addWidget(QtWidgets.QLabel('Planner'), 0, 0, QtCore.Qt.AlignRight)
        layout.addWidget(self.plannerSelect, 0, 1, 1, 2)
        layout.addWidget(timeLimitLabel, 1, 0, QtCore.Qt.AlignRight)
        layout.addWidget(self.timeLimit, 1, 1, 1, 2)
        layout.addWidget(propagationLabel, 2, 0, QtCore.Qt.AlignRight)
        layout.addWidget(self.propagation, 2, 1, 1, 2)
        layout.addWidget(durationLabel, 3, 0)
        layout.addWidget(self.minControlDuration, 3, 1)
        layout.addWidget(self.maxControlDuration, 3, 2)
        layout.addWidget(self.stackedWidget, 4, 0, 1, 3)
        self.setLayout(layout)

class PlannerWidget(QtWidgets.QStackedWidget):
    def __init__(self):
        super(PlannerWidget, self).__init__()
        self.geometricPlanning = GeometricPlannerWidget()
        self.controlPlanning = ControlPlannerWidget()
        self.addWidget(self.geometricPlanning)
        self.addWidget(self.controlPlanning)


class BoundsWidget(QtWidgets.QWidget):
    def __init__(self):
        super(BoundsWidget, self).__init__()
        self.bounds_high = BoundsBox('Upper bounds')
        self.bounds_low = BoundsBox('Lower bounds')
        self.resetButton = QtWidgets.QPushButton('Reset')
        layout = QtWidgets.QGridLayout()
        layout.addWidget(self.bounds_high, 0,0)
        layout.addWidget(self.bounds_low, 1,0)
        layout.addWidget(self.resetButton, 2,0, QtCore.Qt.AlignRight)
        self.setLayout(layout)

class BoundsBox(QtWidgets.QGroupBox):
    valueChanged = Signal(list)

    def __init__(self, title):
        super(BoundsBox, self).__init__(title)
        xlabel = QtWidgets.QLabel('X')
        ylabel = QtWidgets.QLabel('Y')
        zlabel = QtWidgets.QLabel('Z')

        self.posx = QtWidgets.QDoubleSpinBox()
        self.posx.setRange(-1000, 1000)
        self.posx.setSingleStep(1)
        self.posy = QtWidgets.QDoubleSpinBox()
        self.posy.setRange(-1000, 1000)
        self.posy.setSingleStep(1)
        self.posz = QtWidgets.QDoubleSpinBox()
        self.posz.setRange(-1000, 1000)
        self.posz.setSingleStep(1)

        layout = QtWidgets.QGridLayout()
        layout.addWidget(xlabel, 1, 0, QtCore.Qt.AlignRight)
        layout.addWidget(ylabel, 2, 0, QtCore.Qt.AlignRight)
        layout.addWidget(zlabel, 3, 0, QtCore.Qt.AlignRight)
        layout.addWidget(self.posx, 1, 1)
        layout.addWidget(self.posy, 2, 1)
        layout.addWidget(self.posz, 3, 1)
        self.setLayout(layout)

        self.posx.valueChanged.connect(self.boundsChange)
        self.posy.valueChanged.connect(self.boundsChange)
        self.posz.valueChanged.connect(self.boundsChange)

    def setBounds(self, value):
        self.posx.setValue(value[0])
        self.posy.setValue(value[1])
        self.posz.setValue(value[2])

    def boundsChange(self, value):
        self.valueChanged.emit([ self.posx.value(), self.posy.value(), self.posz.value() ])

class SolveWidget(QtWidgets.QWidget):
    def __init__(self):
        super(SolveWidget, self).__init__()
        self.solveButton = QtWidgets.QPushButton('Solve')
        self.clearButton = QtWidgets.QPushButton('Clear')
        explorationVizLabel = QtWidgets.QLabel('Show:')
        self.explorationVizSelect = QtWidgets.QComboBox()
        self.explorationVizSelect.addItem('none')
        self.explorationVizSelect.addItem('states')
        self.explorationVizSelect.addItem('states and edges')
        self.animateCheck = QtWidgets.QCheckBox('Animate')
        self.animateCheck.setChecked(True)
        speedlabel = QtWidgets.QLabel('Speed:')
        self.speedSlider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.speedSlider.setTickPosition(QtWidgets.QSlider.TicksBothSides)
        self.speedSlider.setTickInterval(1)
        self.speedSlider.setSingleStep(1)
        self.speedSlider.setValue(1)
        self.speedSlider.setMaximum(11)
        self.speedSlider.setMaximumSize(200, 30)

        layout = QtWidgets.QGridLayout()
        layout.addWidget(self.solveButton, 0, 0)
        layout.addWidget(self.clearButton, 0, 1)
        layout.addWidget(explorationVizLabel, 0, 2, QtCore.Qt.AlignRight)
        layout.addWidget(self.explorationVizSelect, 0, 3)
        layout.addWidget(self.animateCheck, 0, 4)
        layout.addWidget(speedlabel, 0, 5)
        layout.addWidget(self.speedSlider, 0, 6)
        self.setLayout(layout)

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)

    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
