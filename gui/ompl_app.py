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
try:
    # try PyQt4 first
    from PyQt4 import QtCore, QtGui, QtOpenGL
    from PyQt4.QtCore import pyqtSignal as Signal
except:
    # if PyQt4 wasn't found, try PySide
    from PySide import QtCore, QtGui, QtOpenGL
    from PySide.QtCore import Signal
from OpenGL import GL, GLU
import webbrowser, re
from math import cos, sin, asin, atan2, pi, pow, ceil

try:
    from ompl.util import OutputHandler, useOutputHandler
    from ompl import base as ob
    from ompl import geometric as og
    from ompl import control as oc
    from ompl import app as oa
except:
    sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'ompl/py-bindings' ) )
    from ompl.util import OutputHandler, useOutputHandler
    from ompl import base as ob
    from ompl import geometric as og
    from ompl import control as oc
    from ompl import app as oa

class LogOutputHandler(OutputHandler):
    def __init__(self, textEdit):
        super(LogOutputHandler, self).__init__()
        self.textEdit = textEdit
        self.redColor = QtGui.QColor(255, 0, 0)
        self.orangeColor = QtGui.QColor(255, 128, 0)
        self.greenColor = QtGui.QColor(0, 255, 64)
        self.blackColor = QtGui.QColor(0, 0, 0)

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

class MainWindow(QtGui.QMainWindow):
    def __init__(self):
        QtGui.QMainWindow.__init__(self)
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
        self.mainWidget.plannerWidget.geometricPlanning.timeLimit.valueChanged.connect(self.setTimeLimit)
        self.mainWidget.plannerWidget.geometricPlanning.timeLimit.valueChanged.connect(
            self.mainWidget.plannerWidget.controlPlanning.timeLimit.setValue)
        self.mainWidget.plannerWidget.controlPlanning.timeLimit.valueChanged.connect(self.setTimeLimit)
        self.mainWidget.plannerWidget.controlPlanning.timeLimit.valueChanged.connect(
            self.mainWidget.plannerWidget.geometricPlanning.timeLimit.setValue)
        self.timeLimit = self.mainWidget.plannerWidget.geometricPlanning.timeLimit.value()

        self.mainWidget.plannerWidget.geometricPlanning.plannerSelect.currentIndexChanged[int].connect(self.setPlanner)
        self.mainWidget.plannerWidget.controlPlanning.plannerSelect.currentIndexChanged[int].connect(self.setPlanner)

        self.mainWidget.boundsWidget.bounds_low.valueChanged.connect(self.mainWidget.glViewer.setLowerBound)
        self.mainWidget.boundsWidget.bounds_high.valueChanged.connect(self.mainWidget.glViewer.setUpperBound)
        self.mainWidget.glViewer.boundLowChanged.connect(self.mainWidget.boundsWidget.bounds_low.setBounds)
        self.mainWidget.glViewer.boundHighChanged.connect(self.mainWidget.boundsWidget.bounds_high.setBounds)

        #self.commandWindow = CommandWindow(self) # not implemented yet
        robotType = [t[0] for t in self.robotTypes].index('GSE3RigidBodyPlanning')
        self.mainWidget.problemWidget.robotTypeSelect.setCurrentIndex(robotType)
        self.setPlanner(0)

        # connect to OMPL's console output (via OutputHandlers)
        self.logWindow = LogWindow(self)
        self.oh = LogOutputHandler(self.logWindow.logView)
        useOutputHandler(self.oh)

    # methods for sending messages to the console output window
    def msgDebug(self, text):
        self.oh.debug(text)
    def msgInform(self, text):
        self.oh.inform(text)
    def msgWarn(self, text):
        self.oh.warn(text)
    def msgError(self, text):
        self.oh.error(text)

    def openEnvironment(self):
        fname = QtGui.QFileDialog.getOpenFileName(self, "Open Environment")
        fname = str(fname[0]) if isinstance(fname, tuple) else str(fname)
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
        fname = QtGui.QFileDialog.getOpenFileName(self, "Open Robot")
        fname = str(fname[0]) if isinstance(fname, tuple) else str(fname)
        if len(fname)>0 and fname!=self.robotFile:
            self.robotFile = fname
            self.omplSetup.setRobotMesh(self.robotFile)
            self.mainWidget.glViewer.setRobot(self.omplSetup.renderRobot())
            self.omplSetup.inferEnvironmentBounds()
            # full state
            start = self.omplSetup.getDefaultStartState()
            # just the first geometric component
            start = ob.State(self.omplSetup.getGeometricComponentStateSpace(),
                self.omplSetup.getGeometricComponentState(start(),0))
            self.mainWidget.problemWidget.setStartPose(start, self.is3D)
            self.mainWidget.problemWidget.setGoalPose(start, self.is3D)
            if self.isGeometric:
                self.mainWidget.plannerWidget.geometricPlanning.resolution.setValue(
                    self.omplSetup.getSpaceInformation().getStateValidityCheckingResolution())
            self.mainWidget.glViewer.setBounds(self.omplSetup.getGeometricComponentStateSpace().getBounds())

    def openPath(self):
        fname = str(QtGui.QFileDialog.getOpenFileName(self, "Open Path"))
        if len(fname)>0:
            pathstr = open(fname,'r').read()
            # Match whitespace-separated sequences of 2 to 4 numbers
            regex = re.compile('\[([0-9\.e-]+\s){0,3}[0-9\.e-]+\]')
            states = regex.finditer(pathstr)
            self.path = []
            self.mainWidget.glViewer.solutionPath = []
            for state in states:
                pos = [float(x) for x in state.group()[1:-1].split()]
                state = next(states)
                rot = [float(x) for x in state.group()[1:-1].split()]
                s = ob.State(self.omplSetup.getGeometricComponentStateSpace())
                if len(pos)==3 and len(rot)==4:
                    # SE(3) state
                    s().setX(pos[0])
                    s().setY(pos[1])
                    s().setZ(pos[2])
                    R = s().rotation()
                    (R.x, R.y, R.z, R.w) = rot
                elif len(pos)==2 and len(rot)==1:
                    # SE(2) state
                    s().setX(pos[0])
                    s().setY(pos[1])
                    s().setYaw(rot[0])
                else:
                    # unknown state type
                    self.msgError("Wrong state format %s, %s", (pos, rot))
                    raise ValueError
                self.path.append(s)
                self.mainWidget.glViewer.solutionPath.append(
                    self.mainWidget.glViewer.getTransform(s()))
            self.mainWidget.problemWidget.setStartPose(self.path[0], self.is3D)
            self.mainWidget.problemWidget.setGoalPose(self.path[-1], self.is3D)

    def savePath(self):
        if self.path:
            fname = str(QtGui.QFileDialog.getSaveFileName(self, 'Save Path', 'path.txt'))
            if len(fname)>0:
                if isinstance(self.path, list):
                    pathstr = ''.join([str(s) for s in self.path])
                else:
                    pathstr = str(self.path)
                open(fname,'w').write(pathstr)

    def setSolutionPath(self, path):
            ns = len(path.states)
            self.path = [ self.omplSetup.getGeometricComponentState(path.states[i], 0) for i in range(ns) ]
            self.mainWidget.glViewer.setSolutionPath(self.path)

    def randMotion(self):
        self.configureApp()

        if self.isGeometric:
            pg = og.PathGeometric(self.omplSetup.getSpaceInformation())
            if pg.randomValid(100):
                pg.interpolate()
                self.setSolutionPath(pg)
            else:
                self.msgError("Unable to generate random valid path")
        else:
            pc = oc.PathControl(self.omplSetup.getSpaceInformation())
            if pc.randomValid(100):
                self.setSolutionPath(pc.asGeometric())
            else:
                self.msgError("Unable to generate random valid path")
                
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

    def setPlanner(self, value):
        self.planner = value
    def createPlanner(self):
        si = self.omplSetup.getSpaceInformation()
        if self.isGeometric:
            if self.planner==0:
                planner = og.KPIECE1(si)
                planner.setRange(self.mainWidget.plannerWidget.geometricPlanning.KPIECERange.value())
                planner.setGoalBias(self.mainWidget.plannerWidget.geometricPlanning.KPIECEGoalBias.value())
                planner.setBorderFraction(self.mainWidget.plannerWidget.geometricPlanning.KPIECEBorderFraction.value())
            elif self.planner==1:
                planner = og.BKPIECE1(si)
                planner.setRange(self.mainWidget.plannerWidget.geometricPlanning.BKPIECERange.value())
                planner.setBorderFraction(self.mainWidget.plannerWidget.geometricPlanning.BKPIECEBorderFraction.value())
            elif self.planner==2:
                planner = og.LBKPIECE1(si)
                planner.setRange(self.mainWidget.plannerWidget.geometricPlanning.LBKPIECERange.value())
                planner.setBorderFraction(self.mainWidget.plannerWidget.geometricPlanning.LBKPIECEBorderFraction.value())
            elif self.planner==3:
                planner = og.BasicPRM(si)
                planner.setMaxNearestNeighbors(self.mainWidget.plannerWidget.geometricPlanning.PRMMaxNearestNeighbors.value())
            elif self.planner==4:
                planner = og.SBL(si)
                planner.setRange(self.mainWidget.plannerWidget.geometricPlanning.SBLRange.value())
            elif self.planner==5:
                planner = og.RRTConnect(si)
                planner.setRange(self.mainWidget.plannerWidget.geometricPlanning.RRTConnectRange.value())
            elif self.planner==6:
                planner = og.RRT(si)
                planner.setRange(self.mainWidget.plannerWidget.geometricPlanning.RRTRange.value())
                planner.setGoalBias(self.mainWidget.plannerWidget.geometricPlanning.RRTGoalBias.value())
            elif self.planner==7:
                planner = og.LazyRRT(si)
                planner.setRange(self.mainWidget.plannerWidget.geometricPlanning.LazyRRTRange.value())
                planner.setGoalBias(self.mainWidget.plannerWidget.geometricPlanning.LazyRRTGoalBias.value())
            elif self.planner==8:
                planner = og.EST(si)
                planner.setRange(self.mainWidget.plannerWidget.geometricPlanning.ESTRange.value())
                planner.setGoalBias(self.mainWidget.plannerWidget.geometricPlanning.ESTGoalBias.value())
        else:
            if self.planner==0:
                planner = oc.KPIECE1(si)
                planner.setGoalBias(self.mainWidget.plannerWidget.controlPlanning.KPIECEGoalBias.value())
                planner.setBorderFraction(self.mainWidget.plannerWidget.controlPlanning.KPIECEBorderFraction.value())
            elif self.planner==1:
                planner = oc.RRT(si)
                planner.setGoalBias(self.mainWidget.plannerWidget.controlPlanning.RRTGoalBias.value())
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
        self.msgDebug('Changing time limit from %g to %g' % (self.timeLimit, value))
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
        self.omplSetup.setup()

    def solve(self):
        self.configureApp()
        self.msgDebug(str(self.omplSetup))

        solved = self.omplSetup.solve(self.timeLimit)

        # update the planner data to render, if needed
        self.mainWidget.glViewer.plannerDataList = self.omplSetup.renderPlannerData(self.omplSetup.getPlannerData())

        # update the displayed bounds, in case planning did so
        self.mainWidget.glViewer.setBounds(self.omplSetup.getGeometricComponentStateSpace().getBounds())
        if solved:
            if self.isGeometric:
                path = self.omplSetup.getSolutionPath()
                initialValid = path.check()
                if initialValid == False:
                    self.msgError("Path reported by planner seems to be invalid!")
                self.omplSetup.simplifySolution()
                path = self.omplSetup.getSolutionPath()
                if initialValid == True and path.check() == False:
                    self.msgError("Simplified path seems to be invalid!")
            else:
                path = self.omplSetup.getSolutionPath().asGeometric()
                if path.check() == False:
                    self.msgError("Path reported by planner seems to be invalid!")

            ns = int(100.0 * float(path.length()) / float(self.omplSetup.getStateSpace().getMaximumExtent()))
            if self.isGeometric and len(path.states) < ns:
                self.msgDebug("Interpolating solution path to " + str(ns) + " states")
                path.interpolate(ns)
                if len(path.states) != ns:
                    self.msgError("Interpolation produced " + str(len(path.states)) + " states instead of " + str(ns) + " states!")
#            if path.check() == False:
#                self.msgError("Something wicked happened to the path during interpolation")
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
        self.openEnvironmentAct = QtGui.QAction('Open &Environment', self,
            shortcut='Ctrl+E', statusTip='Open an environment model',
            triggered=self.openEnvironment)
        self.openRobotAct = QtGui.QAction('Open &Robot', self,
            shortcut='Ctrl+R', statusTip='Open a robot model',
            triggered=self.openRobot)
        self.openPathAct = QtGui.QAction('&Open Path', self,
            shortcut='Ctrl+O', statusTip='Open a path',
            triggered=self.openPath)
        self.savePathAct = QtGui.QAction('Save &Path', self,
            shortcut='Ctrl+S', statusTip='Save a path',
            triggered=self.savePath)
        self.exitAct = QtGui.QAction('E&xit', self, shortcut='Ctrl+Q',
            statusTip='Exit the application', triggered=self.close)

        self.logWindowAct = QtGui.QAction('Log Window', self,
            shortcut='Ctrl+1', triggered=self.showLogWindow)
        self.randMotionAct = QtGui.QAction('Random &Motion', self, shortcut='Ctrl+M', triggered=self.randMotion)
        self.commandWindowAct = QtGui.QAction('Command Window', self,
            shortcut='Ctrl+2', triggered=self.showCommandWindow)

        self.omplWebAct = QtGui.QAction('OMPL Web Site', self,
            triggered=self.omplWebSite)
        self.contactDevsAct = QtGui.QAction('Contact Developers', self,
            triggered=self.contactDevs)
        self.emailListAct = QtGui.QAction('Email OMPL Mailing List', self,
            triggered=self.emailList)

    def createMenus(self):
        self.fileMenu = self.menuBar().addMenu('&File')
        self.fileMenu.addAction(self.openEnvironmentAct)
        self.fileMenu.addAction(self.openRobotAct)
        self.fileMenu.addAction(self.openPathAct)
        self.fileMenu.addAction(self.savePathAct)
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
            #if eval('isclass(oa.%s) and issubclass(oa.%s, oa.AppBaseGeometric) and issubclass(oa.%s, oa.RenderGeometry)' % (c,c,c)):
                name = eval('oa.%s().getName()' % c)
                apptype = eval('oa.%s().getAppType()' % c)
                self.robotTypes.append((c, name, apptype))


class MainWidget(QtGui.QWidget):
    def __init__(self, robotTypes, parent=None, flags=QtCore.Qt.WindowFlags(0)):
        super(MainWidget, self).__init__(parent, flags)
        self.glViewer = GLViewer()
        self.problemWidget = ProblemWidget(robotTypes)
        self.plannerWidget = PlannerWidget()
        self.boundsWidget = BoundsWidget()
        self.solveWidget = SolveWidget()
        self.solveWidget.setSizePolicy(QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Fixed))
        tabWidget = QtGui.QTabWidget()
        tabWidget.addTab(self.problemWidget, "Problem")
        tabWidget.addTab(self.plannerWidget, "Planner")
        tabWidget.addTab(self.boundsWidget, "Bounding box")
        tabWidget.setSizePolicy(QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Fixed))
        layout = QtGui.QGridLayout()
        layout.addWidget(self.glViewer, 0, 0, 2, 1)
        layout.addWidget(tabWidget, 0, 1)
        layout.addWidget(self.solveWidget, 2, 0, 1, 2)
        self.setLayout(layout)
        self.problemWidget.startChanged.connect(self.glViewer.setStartPose)
        self.problemWidget.goalChanged.connect(self.glViewer.setGoalPose)
        self.solveWidget.showData.toggled.connect(self.glViewer.toggleShowData)
        self.solveWidget.animateCheck.toggled.connect(self.glViewer.toggleAnimation)
        self.solveWidget.speedSlider.valueChanged.connect(self.glViewer.setSpeed)

class LogWindow(QtGui.QWidget):
    def __init__(self, parent=None, flags=QtCore.Qt.Tool):
        super(LogWindow, self).__init__(parent, flags)
        self.setWindowTitle('OMPL Log')
        self.resize(640, 320)
        self.logView = QtGui.QTextEdit(self)
        self.logView.setReadOnly(True)
        layout = QtGui.QGridLayout()
        layout.addWidget(self.logView, 0, 0)
        self.setLayout(layout)

class CommandWindow(QtGui.QWidget):
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
    def setBounds(self, bounds):
        self.bounds_low = [x for x in bounds.low ]
        self.bounds_high = [x for x in bounds.high ]
        if len(self.bounds_low)==2:
            self.bounds_low.append(0)
            self.bounds_high.append(0)
        bbox = zip(self.bounds_low, self.bounds_high)
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
    def toggleShowData(self, value):
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
        self.solutionPath = [ self.getTransform(state) for state in path ]
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
            GL.glCallList(self.plannerDataList)

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
        self.scale = self.scale * pow(2.0, -event.delta() / 240.0)
        self.lastPos = event.pos()
        self.updateGL()

class ProblemWidget(QtGui.QWidget):
    startChanged = Signal(list)
    goalChanged = Signal(list)

    def __init__(self, robotTypes):
        super(ProblemWidget, self).__init__()
        robotTypeLabel =  QtGui.QLabel('Robot type')
        self.robotTypeSelect = QtGui.QComboBox()
        for robotType in robotTypes:
            self.robotTypeSelect.addItem(robotType[1])
        self.robotTypeSelect.setMaximumSize(200, 2000)

        self.startPose3D = Pose3DBox('Start pose')
        self.goalPose3D = Pose3DBox('Goal pose')
        self.startPose2D = Pose2DBox('Start pose')
        self.goalPose2D = Pose2DBox('Goal pose')

        elevation2Dlabel = QtGui.QLabel('Elevation')
        self.elevation2D = QtGui.QDoubleSpinBox()
        self.elevation2D.setRange(-1000, 1000)
        self.elevation2D.setSingleStep(1)

        startGoal3D = QtGui.QWidget()
        layout = QtGui.QVBoxLayout()
        layout.addWidget(self.startPose3D)
        layout.addWidget(self.goalPose3D)
        startGoal3D.setLayout(layout)

        startGoal2D = QtGui.QWidget()
        layout = QtGui.QGridLayout()
        layout.addWidget(self.startPose2D, 0, 0, 1, 2)
        layout.addWidget(self.goalPose2D, 1, 0, 1 ,2)
        #layout.addWidget(elevation2Dlabel, 2, 0, QtCore.Qt.AlignRight)
        #layout.addWidget(self.elevation2D, 2, 1)
        startGoal2D.setLayout(layout)

        self.poses = QtGui.QStackedWidget()
        self.poses.addWidget(startGoal3D)
        self.poses.addWidget(startGoal2D)

        layout = QtGui.QGridLayout()
        layout.addWidget(robotTypeLabel, 0, 0)
        layout.addWidget(self.robotTypeSelect, 0, 1)
        layout.addWidget(self.poses, 1, 0, 1, 2)
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

class Pose3DBox(QtGui.QGroupBox):
    valueChanged = Signal(list)

    def __init__(self, title):
        super(Pose3DBox, self).__init__(title)
        xlabel = QtGui.QLabel('X')
        ylabel = QtGui.QLabel('Y')
        zlabel = QtGui.QLabel('Z')
        poslabel = QtGui.QLabel('Position')
        rotlabel = QtGui.QLabel('Rotation')

        self.posx = QtGui.QDoubleSpinBox()
        self.posx.setRange(-1000, 1000)
        self.posx.setSingleStep(1)
        self.posy = QtGui.QDoubleSpinBox()
        self.posy.setRange(-1000, 1000)
        self.posy.setSingleStep(1)
        self.posz = QtGui.QDoubleSpinBox()
        self.posz.setRange(-1000, 1000)
        self.posz.setSingleStep(1)
        self.rotx = QtGui.QDoubleSpinBox()
        self.rotx.setRange(-360,360)
        self.rotx.setSingleStep(1)
        self.roty = QtGui.QDoubleSpinBox()
        self.roty.setRange(-360,360)
        self.roty.setSingleStep(1)
        self.rotz = QtGui.QDoubleSpinBox()
        self.rotz.setRange(-360,360)
        self.rotz.setSingleStep(1)

        layout = QtGui.QGridLayout()
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
            self.roty.setValue(rad2deg * asin(2.*(q.w*q.y-q.z*q.x)))
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
        rot.w = c[0]*c[1]*c[2] + s[0]*s[1]*s[2]
        rot.x = s[0]*c[1]*c[2] - c[0]*s[1]*s[2]
        rot.y = c[0]*s[1]*c[2] + s[0]*c[1]*s[2]
        rot.z = c[0]*c[1]*s[2] - s[0]*s[1]*c[2]
        return state

    def poseChange(self, value):
        self.valueChanged.emit([self.rotx.value(), self.roty.value(), self.rotz.value(),
            self.posx.value(), self.posy.value(), self.posz.value() ])

class Pose2DBox(QtGui.QGroupBox):
    valueChanged = Signal(list)

    def __init__(self, title):
        super(Pose2DBox, self).__init__(title)
        xlabel = QtGui.QLabel('X')
        ylabel = QtGui.QLabel('Y')
        rotlabel = QtGui.QLabel('Rotation')

        self.posx = QtGui.QDoubleSpinBox()
        self.posx.setRange(-1000, 1000)
        self.posx.setSingleStep(1)
        self.posy = QtGui.QDoubleSpinBox()
        self.posy.setRange(-1000, 1000)
        self.posy.setSingleStep(1)
        self.rot = QtGui.QDoubleSpinBox()
        self.rot.setRange(-360,360)
        self.rot.setSingleStep(1)

        layout = QtGui.QGridLayout()
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

class GeometricPlannerWidget(QtGui.QGroupBox):
    def __init__(self):
        super(GeometricPlannerWidget, self).__init__('Geometric planning')
        self.setFlat(True)

        # list of planners
        plannerLabel = QtGui.QLabel('Planner')
        self.plannerSelect = QtGui.QComboBox()
        self.plannerSelect.addItem('KPIECE')
        self.plannerSelect.addItem('Bi-directional KPIECE')
        self.plannerSelect.addItem('Lazy Bi-directional KPIECE')
        self.plannerSelect.addItem('BasicPRM')
        self.plannerSelect.addItem('SBL')
        self.plannerSelect.addItem('RRT Connect')
        self.plannerSelect.addItem('RRT')
        self.plannerSelect.addItem('Lazy RRT')
        self.plannerSelect.addItem('EST')
        self.plannerSelect.setMinimumContentsLength(10)
        self.plannerSelect.setSizeAdjustPolicy(QtGui.QComboBox.AdjustToMinimumContentsLength)

        # KPIECE options
        self.KPIECEOptions = QtGui.QGroupBox('KPIECE options')
        KPIECErangeLabel = QtGui.QLabel('Range')
        self.KPIECERange = QtGui.QDoubleSpinBox()
        self.KPIECERange.setRange(0, 10000)
        self.KPIECERange.setSingleStep(1)
        self.KPIECERange.setValue(0)
        KPIECEgoalBiasLabel = QtGui.QLabel('Goal bias')
        self.KPIECEGoalBias = QtGui.QDoubleSpinBox()
        self.KPIECEGoalBias.setRange(0, 1)
        self.KPIECEGoalBias.setSingleStep(.05)
        self.KPIECEGoalBias.setValue(0.05)
        KPIECEborderFractionLabel = QtGui.QLabel('Border fraction')
        self.KPIECEBorderFraction = QtGui.QDoubleSpinBox()
        self.KPIECEBorderFraction.setRange(0, 1)
        self.KPIECEBorderFraction.setSingleStep(.05)
        self.KPIECEBorderFraction.setValue(.9)
        layout = QtGui.QGridLayout()
        layout.addWidget(KPIECErangeLabel, 0, 0, QtCore.Qt.AlignRight)
        layout.addWidget(self.KPIECERange, 0, 1)
        layout.addWidget(KPIECEgoalBiasLabel, 1, 0, QtCore.Qt.AlignRight)
        layout.addWidget(self.KPIECEGoalBias, 1, 1)
        layout.addWidget(KPIECEborderFractionLabel, 2, 0, QtCore.Qt.AlignRight)
        layout.addWidget(self.KPIECEBorderFraction, 2, 1)
        self.KPIECEOptions.setLayout(layout)

        # BKPIECE options
        self.BKPIECEOptions = QtGui.QGroupBox('BKPIECE options')
        BKPIECErangeLabel = QtGui.QLabel('Range')
        self.BKPIECERange = QtGui.QDoubleSpinBox()
        self.BKPIECERange.setRange(0, 10000)
        self.BKPIECERange.setSingleStep(1)
        self.BKPIECERange.setValue(0)
        BKPIECEborderFractionLabel = QtGui.QLabel('Border fraction')
        self.BKPIECEBorderFraction = QtGui.QDoubleSpinBox()
        self.BKPIECEBorderFraction.setRange(0, 1)
        self.BKPIECEBorderFraction.setSingleStep(.05)
        self.BKPIECEBorderFraction.setValue(.9)
        layout = QtGui.QGridLayout()
        layout.addWidget(BKPIECErangeLabel, 0, 0, QtCore.Qt.AlignRight)
        layout.addWidget(self.BKPIECERange, 0, 1)
        layout.addWidget(BKPIECEborderFractionLabel, 1, 0, QtCore.Qt.AlignRight)
        layout.addWidget(self.BKPIECEBorderFraction, 1, 1)
        self.BKPIECEOptions.setLayout(layout)

        # LBKPIECE options
        self.LBKPIECEOptions = QtGui.QGroupBox('LBKPIECE options')
        LBKPIECErangeLabel = QtGui.QLabel('Range')
        self.LBKPIECERange = QtGui.QDoubleSpinBox()
        self.LBKPIECERange.setRange(0, 10000)
        self.LBKPIECERange.setSingleStep(1)
        self.LBKPIECERange.setValue(0)
        LBKPIECEborderFractionLabel = QtGui.QLabel('Border fraction')
        self.LBKPIECEBorderFraction = QtGui.QDoubleSpinBox()
        self.LBKPIECEBorderFraction.setRange(0, 1)
        self.LBKPIECEBorderFraction.setSingleStep(.05)
        self.LBKPIECEBorderFraction.setValue(.9)
        layout = QtGui.QGridLayout()
        layout.addWidget(LBKPIECErangeLabel, 0, 0, QtCore.Qt.AlignRight)
        layout.addWidget(self.LBKPIECERange, 0, 1)
        layout.addWidget(LBKPIECEborderFractionLabel, 1, 0, QtCore.Qt.AlignRight)
        layout.addWidget(self.LBKPIECEBorderFraction, 1, 1)
        self.LBKPIECEOptions.setLayout(layout)

        # PRM options
        self.PRMOptions = QtGui.QGroupBox('BasicPRM options')
        PRMmaxNearestNeighborsLabel = QtGui.QLabel('Max. nearest neighbors')
        self.PRMMaxNearestNeighbors = QtGui.QSpinBox()
        self.PRMMaxNearestNeighbors.setRange(0, 1000)
        self.PRMMaxNearestNeighbors.setSingleStep(1)
        self.PRMMaxNearestNeighbors.setValue(10)
        layout = QtGui.QGridLayout()
        layout.addWidget(PRMmaxNearestNeighborsLabel, 0, 0, QtCore.Qt.AlignRight)
        layout.addWidget(self.PRMMaxNearestNeighbors, 0, 1)
        self.PRMOptions.setLayout(layout)

        # SBL options
        self.SBLOptions = QtGui.QGroupBox('SBL options')
        SBLrangeLabel = QtGui.QLabel('Range')
        self.SBLRange = QtGui.QDoubleSpinBox()
        self.SBLRange.setRange(0, 10000)
        self.SBLRange.setSingleStep(1)
        self.SBLRange.setValue(0)
        layout = QtGui.QGridLayout()
        layout.addWidget(SBLrangeLabel, 0, 0, QtCore.Qt.AlignRight)
        layout.addWidget(self.SBLRange, 0, 1)
        self.SBLOptions.setLayout(layout)

        # RRT Connect options
        self.RRTConnectOptions = QtGui.QGroupBox('RRT Connect options')
        RRTConnectrangeLabel = QtGui.QLabel('Range')
        self.RRTConnectRange = QtGui.QDoubleSpinBox()
        self.RRTConnectRange.setRange(0, 10000)
        self.RRTConnectRange.setSingleStep(1)
        self.RRTConnectRange.setValue(0)
        layout = QtGui.QGridLayout()
        layout.addWidget(RRTConnectrangeLabel, 0, 0, QtCore.Qt.AlignRight)
        layout.addWidget(self.RRTConnectRange, 0, 1)
        self.RRTConnectOptions.setLayout(layout)

        # RRT options
        self.RRTOptions = QtGui.QGroupBox('RRT options')
        RRTrangeLabel = QtGui.QLabel('Range')
        self.RRTRange = QtGui.QDoubleSpinBox()
        self.RRTRange.setRange(0, 10000)
        self.RRTRange.setSingleStep(1)
        self.RRTRange.setValue(0)
        RRTgoalBiasLabel = QtGui.QLabel('Goal bias')
        self.RRTGoalBias = QtGui.QDoubleSpinBox()
        self.RRTGoalBias.setRange(0, 1)
        self.RRTGoalBias.setSingleStep(.05)
        self.RRTGoalBias.setValue(0.05)
        layout = QtGui.QGridLayout()
        layout.addWidget(RRTrangeLabel, 0, 0, QtCore.Qt.AlignRight)
        layout.addWidget(self.RRTRange, 0, 1)
        layout.addWidget(RRTgoalBiasLabel, 1, 0, QtCore.Qt.AlignRight)
        layout.addWidget(self.RRTGoalBias, 1, 1)
        self.RRTOptions.setLayout(layout)

        # Lazy RRT options
        self.LazyRRTOptions = QtGui.QGroupBox('Lazy RRT options')
        LazyRRTrangeLabel = QtGui.QLabel('Range')
        self.LazyRRTRange = QtGui.QDoubleSpinBox()
        self.LazyRRTRange.setRange(0, 10000)
        self.LazyRRTRange.setSingleStep(1)
        self.LazyRRTRange.setValue(0)
        LazyRRTgoalBiasLabel = QtGui.QLabel('Goal bias')
        self.LazyRRTGoalBias = QtGui.QDoubleSpinBox()
        self.LazyRRTGoalBias.setRange(0, 1)
        self.LazyRRTGoalBias.setSingleStep(.05)
        self.LazyRRTGoalBias.setValue(0.05)
        layout = QtGui.QGridLayout()
        layout.addWidget(LazyRRTrangeLabel, 0, 0, QtCore.Qt.AlignRight)
        layout.addWidget(self.LazyRRTRange, 0, 1)
        layout.addWidget(LazyRRTgoalBiasLabel, 1, 0, QtCore.Qt.AlignRight)
        layout.addWidget(self.LazyRRTGoalBias, 1, 1)
        self.LazyRRTOptions.setLayout(layout)

        # EST options
        self.ESTOptions = QtGui.QGroupBox('EST options')
        ESTgoalBiasLabel = QtGui.QLabel('Goal bias')
        ESTrangeLabel = QtGui.QLabel('Range')
        self.ESTRange = QtGui.QDoubleSpinBox()
        self.ESTRange.setRange(0, 10000)
        self.ESTRange.setSingleStep(1)
        self.ESTRange.setValue(0)
        self.ESTGoalBias = QtGui.QDoubleSpinBox()
        self.ESTGoalBias.setRange(0, 1)
        self.ESTGoalBias.setSingleStep(.05)
        self.ESTGoalBias.setValue(0.05)
        layout = QtGui.QGridLayout()
        layout.addWidget(ESTrangeLabel, 0, 0, QtCore.Qt.AlignRight)
        layout.addWidget(self.ESTRange, 0, 1)
        layout.addWidget(ESTgoalBiasLabel, 1, 0, QtCore.Qt.AlignRight)
        layout.addWidget(self.ESTGoalBias, 1, 1)
        self.ESTOptions.setLayout(layout)

        self.stackedWidget = QtGui.QStackedWidget()
        self.stackedWidget.addWidget(self.KPIECEOptions)
        self.stackedWidget.addWidget(self.BKPIECEOptions)
        self.stackedWidget.addWidget(self.LBKPIECEOptions)
        self.stackedWidget.addWidget(self.PRMOptions)
        self.stackedWidget.addWidget(self.SBLOptions)
        self.stackedWidget.addWidget(self.RRTConnectOptions)
        self.stackedWidget.addWidget(self.RRTOptions)
        self.stackedWidget.addWidget(self.LazyRRTOptions)
        self.stackedWidget.addWidget(self.ESTOptions)
        self.plannerSelect.activated[int].connect(self.stackedWidget.setCurrentIndex)

        timeLimitLabel = QtGui.QLabel('Time (sec.)')
        self.timeLimit = QtGui.QDoubleSpinBox()
        self.timeLimit.setRange(0, 1000)
        self.timeLimit.setSingleStep(1)
        self.timeLimit.setValue(10.0)

        resolutionLabel = QtGui.QLabel('Collision checking\nresolution')
        resolutionLabel.setAlignment(QtCore.Qt.AlignRight)
        self.resolution = QtGui.QDoubleSpinBox()
        self.resolution.setRange(0.001, 1.0)
        self.resolution.setSingleStep(.002)
        self.resolution.setValue(0.010)
        self.resolution.setDecimals(3)

        layout = QtGui.QGridLayout()
        layout.addWidget(plannerLabel, 0, 0, QtCore.Qt.AlignRight)
        layout.addWidget(self.plannerSelect, 0, 1)
        layout.addWidget(timeLimitLabel, 1, 0, QtCore.Qt.AlignRight)
        layout.addWidget(self.timeLimit, 1, 1)
        layout.addWidget(resolutionLabel, 2, 0, QtCore.Qt.AlignRight)
        layout.addWidget(self.resolution, 2, 1)
        layout.addWidget(self.stackedWidget, 3, 0, 1, 2)
        self.setLayout(layout)

class ControlPlannerWidget(QtGui.QGroupBox):
    def __init__(self):
        super(ControlPlannerWidget, self).__init__('Planning with controls')
        self.setFlat(True)

        # list of planners
        plannerLabel = QtGui.QLabel('Planner')
        self.plannerSelect = QtGui.QComboBox()
        self.plannerSelect.addItem('KPIECE')
        self.plannerSelect.addItem('RRT')

        # control KPIECE options
        self.KPIECEOptions = QtGui.QGroupBox('KPIECE options')
        KPIECEgoalBiasLabel = QtGui.QLabel('Goal bias')
        self.KPIECEGoalBias = QtGui.QDoubleSpinBox()
        self.KPIECEGoalBias.setRange(0, 1)
        self.KPIECEGoalBias.setSingleStep(.05)
        self.KPIECEGoalBias.setValue(0.05)
        KPIECEborderFractionLabel = QtGui.QLabel('Border fraction')
        self.KPIECEBorderFraction = QtGui.QDoubleSpinBox()
        self.KPIECEBorderFraction.setRange(0, 1)
        self.KPIECEBorderFraction.setSingleStep(.05)
        self.KPIECEBorderFraction.setValue(.9)
        layout = QtGui.QGridLayout()
        layout.addWidget(KPIECEgoalBiasLabel, 0, 0, QtCore.Qt.AlignRight)
        layout.addWidget(self.KPIECEGoalBias, 0, 1)
        layout.addWidget(KPIECEborderFractionLabel, 1, 0, QtCore.Qt.AlignRight)
        layout.addWidget(self.KPIECEBorderFraction, 1, 1)
        self.KPIECEOptions.setLayout(layout)

        # RRT options
        self.RRTOptions = QtGui.QGroupBox('RRT options')
        RRTgoalBiasLabel = QtGui.QLabel('Goal bias')
        self.RRTGoalBias = QtGui.QDoubleSpinBox()
        self.RRTGoalBias.setRange(0, 1)
        self.RRTGoalBias.setSingleStep(.05)
        self.RRTGoalBias.setValue(0.05)
        layout = QtGui.QGridLayout()
        layout.addWidget(RRTgoalBiasLabel, 0, 0, QtCore.Qt.AlignRight)
        layout.addWidget(self.RRTGoalBias, 0, 1)
        self.RRTOptions.setLayout(layout)

        self.stackedWidget = QtGui.QStackedWidget()
        self.stackedWidget.addWidget(self.KPIECEOptions)
        self.stackedWidget.addWidget(self.RRTOptions)
        self.plannerSelect.activated.connect(self.stackedWidget.setCurrentIndex)

        timeLimitLabel = QtGui.QLabel('Time (sec.)')
        self.timeLimit = QtGui.QDoubleSpinBox()
        self.timeLimit.setRange(0, 1000)
        self.timeLimit.setSingleStep(1)
        self.timeLimit.setValue(10.0)

        propagationLabel = QtGui.QLabel('Propagation\nstep size')
        propagationLabel.setAlignment(QtCore.Qt.AlignRight)
        self.propagation = QtGui.QDoubleSpinBox()
        self.propagation.setRange(0.01, 1000.00)
        self.propagation.setSingleStep(.01)
        self.propagation.setValue(0.2)
        self.propagation.setDecimals(2)

        durationLabel = QtGui.QLabel('Control duration\n(min/max #steps)')
        durationLabel.setAlignment(QtCore.Qt.AlignRight)
        self.minControlDuration = QtGui.QSpinBox()
        self.minControlDuration.setRange(1, 1000)
        self.minControlDuration.setSingleStep(1)
        self.minControlDuration.setValue(1)
        self.maxControlDuration = QtGui.QSpinBox()
        self.maxControlDuration.setRange(1, 1000)
        self.maxControlDuration.setSingleStep(1)
        self.maxControlDuration.setValue(20)

        layout = QtGui.QGridLayout()
        layout.addWidget(plannerLabel, 0, 0, QtCore.Qt.AlignRight)
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

class PlannerWidget(QtGui.QStackedWidget):
    def __init__(self):
        super(PlannerWidget, self).__init__()
        self.geometricPlanning = GeometricPlannerWidget()
        self.controlPlanning = ControlPlannerWidget()
        self.addWidget(self.geometricPlanning)
        self.addWidget(self.controlPlanning)


class BoundsWidget(QtGui.QWidget):
    def __init__(self):
        super(BoundsWidget, self).__init__()
        self.bounds_high = BoundsBox('Upper bounds')
        self.bounds_low = BoundsBox('Lower bounds')
        self.resetButton = QtGui.QPushButton('Reset')
        layout = QtGui.QGridLayout()
        layout.addWidget(self.bounds_high, 0,0)
        layout.addWidget(self.bounds_low, 1,0)
        layout.addWidget(self.resetButton, 2,0, QtCore.Qt.AlignRight)
        self.setLayout(layout)

class BoundsBox(QtGui.QGroupBox):
    valueChanged = Signal(list)

    def __init__(self, title):
        super(BoundsBox, self).__init__(title)
        xlabel = QtGui.QLabel('X')
        ylabel = QtGui.QLabel('Y')
        zlabel = QtGui.QLabel('Z')

        self.posx = QtGui.QDoubleSpinBox()
        self.posx.setRange(-1000, 1000)
        self.posx.setSingleStep(1)
        self.posy = QtGui.QDoubleSpinBox()
        self.posy.setRange(-1000, 1000)
        self.posy.setSingleStep(1)
        self.posz = QtGui.QDoubleSpinBox()
        self.posz.setRange(-1000, 1000)
        self.posz.setSingleStep(1)

        layout = QtGui.QGridLayout()
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

class SolveWidget(QtGui.QWidget):
    def __init__(self):
        super(SolveWidget, self).__init__()
        self.solveButton = QtGui.QPushButton('Solve')
        self.clearButton = QtGui.QPushButton('Clear')
        self.showData = QtGui.QCheckBox('Show Exploration')
        self.showData.setChecked(False)
        self.animateCheck = QtGui.QCheckBox('Animate')
        self.animateCheck.setChecked(True)
        speedlabel = QtGui.QLabel('Speed:')
        self.speedSlider = QtGui.QSlider(QtCore.Qt.Horizontal)
        self.speedSlider.setTickPosition(QtGui.QSlider.TicksBothSides)
        self.speedSlider.setTickInterval(1)
        self.speedSlider.setSingleStep(1)
        self.speedSlider.setValue(1)
        self.speedSlider.setMaximum(11)
        self.speedSlider.setMaximumSize(200, 30)

        layout = QtGui.QGridLayout()
        layout.addWidget(self.solveButton, 0, 0)
        layout.addWidget(self.clearButton, 0, 1)
        layout.addWidget(self.showData, 0, 2)
        layout.addWidget(self.animateCheck, 0, 3)
        layout.addWidget(speedlabel, 0, 5)
        layout.addWidget(self.speedSlider, 0, 6)
        self.setLayout(layout)

if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)

    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
