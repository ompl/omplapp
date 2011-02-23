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
from PyQt4 import QtCore, QtGui, QtOpenGL
from OpenGL import GL, GLU
import webbrowser, re, tempfile
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

class OMPLConfiguration:
    def __init__(self):
        self.geometricPlanning = False
        if self.geometricPlanning:
            self.omplSetup = oa.SE3RigidBodyPlanning()
        else:
            self.omplSetup = oa.SE3ControlPlanning()

# global variable
gConfig = OMPLConfiguration()

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
        self.mainWidget = MainWidget()
        self.setCentralWidget(self.mainWidget)
        if gConfig.geometricPlanning:
            self.setWindowTitle('OMPL: Planning With Geometric Constraints')
        else:
            self.setWindowTitle('OMPL: Planning With Differential Constraints')
        self.environmentFile = None
        self.robotFile = None
        self.path = None
        self.omplSetup = gConfig.omplSetup
        self.mainWidget.solveWidget.solveButton.clicked.connect(self.solve)
        self.mainWidget.solveWidget.clearButton.clicked.connect(self.clear)
        self.mainWidget.plannerWidget.plannerSelect.activated.connect(self.setPlanner)
        if gConfig.geometricPlanning:
            self.mainWidget.plannerWidget.KPIECERange.valueChanged.connect(self.setRange)
            self.mainWidget.plannerWidget.KPIECEGoalBias.valueChanged.connect(self.setGoalBias)
            self.mainWidget.plannerWidget.KPIECEBorderFraction.valueChanged.connect(self.setBorderFraction)
            self.mainWidget.plannerWidget.LBKPIECERange.valueChanged.connect(self.setRange)
            self.mainWidget.plannerWidget.LBKPIECEBorderFraction.valueChanged.connect(self.setBorderFraction)
            self.mainWidget.plannerWidget.PRMMaxNearestNeighbors.valueChanged.connect(self.setMaxNearestNeighbors)
            self.mainWidget.plannerWidget.SBLRange.valueChanged.connect(self.setRange)
            self.mainWidget.plannerWidget.RRTConnectRange.valueChanged.connect(self.setRange)
            self.mainWidget.plannerWidget.RRTRange.valueChanged.connect(self.setRange)
            self.mainWidget.plannerWidget.RRTGoalBias.valueChanged.connect(self.setGoalBias)
            self.mainWidget.plannerWidget.LazyRRTRange.valueChanged.connect(self.setRange)
            self.mainWidget.plannerWidget.LazyRRTGoalBias.valueChanged.connect(self.setGoalBias)
            self.mainWidget.plannerWidget.ESTRange.valueChanged.connect(self.setRange)
            self.mainWidget.plannerWidget.ESTGoalBias.valueChanged.connect(self.setGoalBias)
        else:
            self.mainWidget.plannerWidget.KPIECEGoalBias.valueChanged.connect(self.setGoalBias)
            self.mainWidget.plannerWidget.KPIECEBorderFraction.valueChanged.connect(self.setBorderFraction)
            self.mainWidget.plannerWidget.RRTGoalBias.valueChanged.connect(self.setGoalBias)
            self.mainWidget.plannerWidget.timeLimit.valueChanged.connect(self.setTimeLimit)

        self.timeLimit = self.mainWidget.plannerWidget.timeLimit.value()
        self.mainWidget.plannerWidget.timeLimit.valueChanged.connect(self.setTimeLimit)
        self.mainWidget.boundsWidget.bounds_low.valueChanged.connect(self.mainWidget.glViewer.setLowerBound)
        self.mainWidget.boundsWidget.bounds_high.valueChanged.connect(self.mainWidget.glViewer.setUpperBound)
        self.mainWidget.glViewer.boundLowChanged.connect(self.mainWidget.boundsWidget.bounds_low.setBounds)
        self.mainWidget.glViewer.boundHighChanged.connect(self.mainWidget.boundsWidget.bounds_high.setBounds)

        # connect to OMPL's console output (via OutputHandlers)
        self.logWindow = LogWindow(self)
        self.oh = LogOutputHandler(self.logWindow.logView)
        useOutputHandler(self.oh)

        # not implemented yet
        #self.commandWindow = CommandWindow(self)
        self.setPlanner(0)
        self.zerobounds = ob.RealVectorBounds(3)
        self.zerobounds.setLow(0)
        self.zerobounds.setHigh(0)

    def msgDebug(self, text):
        self.oh.debug(text)

    def msgInform(self, text):
        self.oh.inform(text)

    def msgWarn(self, text):
        self.oh.warn(text)

    def msgError(self, text):
        self.oh.error(text)

    def openEnvironment(self):
        fname = str(QtGui.QFileDialog.getOpenFileName(self, "Open Environment"))
        if len(fname)>0 and fname!=self.environmentFile:
            self.environmentFile = fname
            self.omplSetup.getSE3StateManifold().setBounds(self.zerobounds)
            self.mainWidget.glViewer.setEnvironment(
                self.omplSetup.setEnvironmentMesh(self.environmentFile))
            self.omplSetup.inferEnvironmentBounds()
            if gConfig.geometricPlanning:
                if self.robotFile:
                    self.mainWidget.plannerWidget.resolution.setValue(
                        self.omplSetup.getSpaceInformation().getStateValidityCheckingResolution())
            self.mainWidget.glViewer.setBounds(self.omplSetup.getSE3StateManifold().getBounds())
    def openRobot(self):
        fname = str(QtGui.QFileDialog.getOpenFileName(self, "Open Robot"))
        if len(fname)>0 and fname!=self.robotFile:
            self.robotFile = fname
            self.omplSetup.getSE3StateManifold().setBounds(self.zerobounds)
            if not self.environmentFile: self.environmentFile = self.robotFile
            self.mainWidget.glViewer.setEnvironment(
                self.omplSetup.setEnvironmentMesh(self.environmentFile))
            self.mainWidget.glViewer.setRobot(
                self.omplSetup.setRobotMesh(self.robotFile))
            self.omplSetup.inferEnvironmentBounds()
            start = ob.State(self.omplSetup.getSE3StateManifold())
            self.omplSetup.getEnvStartState(start)
            self.mainWidget.problemWidget.startPose.setPose(start)
            self.mainWidget.problemWidget.goalPose.setPose(start)
            if gConfig.geometricPlanning:
                self.mainWidget.plannerWidget.resolution.setValue(
                    self.omplSetup.getSpaceInformation().getStateValidityCheckingResolution())
            self.mainWidget.glViewer.setBounds(self.omplSetup.getSE3StateManifold().getBounds())

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
                s = ob.State(self.omplSetup.getSE3StateManifold())
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
                    s().setZ(0.)
                    s().rotation().setAxisAngle(0,0,1,rot[0])
                else:
                    # unknown state type
                    print pos, rot
                    raise ValueError
                self.path.append(s)
                self.mainWidget.glViewer.solutionPath.append(
                    self.mainWidget.glViewer.getTransform(s()))
            self.mainWidget.problemWidget.startPose.setPose(self.path[0])
            self.mainWidget.problemWidget.goalPose.setPose(self.path[-1])

    def savePath(self):
        if self.path:
            fname = str(QtGui.QFileDialog.getSaveFileName(self, 'Save Path', 'path.txt'))
            if len(fname)>0:
                if isinstance(self.path, list):
                    pathstr = ''.join([str(s) for s in self.path])
                else:
                    pathstr = str(self.path)
                open(fname,'w').write(pathstr)

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
        si = self.omplSetup.getSpaceInformation()
        if gConfig.geometricPlanning:
            if value==0:
                self.planner = og.KPIECE1(si)
                self.planner.setRange(self.mainWidget.plannerWidget.KPIECERange.value())
                self.planner.setGoalBias(self.mainWidget.plannerWidget.KPIECEGoalBias.value())
                self.planner.setBorderFraction(self.mainWidget.plannerWidget.KPIECEBorderFraction.value())
            elif value==1:
                self.planner = og.LBKPIECE1(si)
                self.planner.setRange(self.mainWidget.plannerWidget.LBKPIECERange.value())
                self.planner.setBorderFraction(self.mainWidget.plannerWidget.LBKPIECEBorderFraction.value())
            elif value==2:
                self.planner = og.BasicPRM(si)
                self.planner.setMaxNearestNeighbors(self.mainWidget.plannerWidget.PRMMaxNearestNeighbors.value())
            elif value==3:
                self.planner = og.SBL(si)
                self.planner.setRange(self.mainWidget.plannerWidget.SBLRange.value())
            elif value==4:
                self.planner = og.RRTConnect(si)
                self.planner.setRange(self.mainWidget.plannerWidget.RRTConnectRange.value())
            elif value==5:
                self.planner = og.RRT(si)
                self.planner.setRange(self.mainWidget.plannerWidget.RRTRange.value())
                self.planner.setGoalBias(self.mainWidget.plannerWidget.RRTGoalBias.value())
            elif value==6:
                self.planner = og.LazyRRT(si)
                self.planner.setRange(self.mainWidget.plannerWidget.LazyRRTRange.value())
                self.planner.setGoalBias(self.mainWidget.plannerWidget.LazyRRTGoalBias.value())
            elif value==7:
                self.planner = og.EST(si)
                self.planner.setRange(self.mainWidget.plannerWidget.ESTRange.value())
                self.planner.setGoalBias(self.mainWidget.plannerWidget.ESTGoalBias.value())
        else:
            if value==0:
                self.planner = oc.KPIECE1(si)
                self.planner.setGoalBias(self.mainWidget.plannerWidget.KPIECEGoalBias.value())
                self.planner.setBorderFraction(self.mainWidget.plannerWidget.KPIECEBorderFraction.value())
            elif value==1:
                self.planner = oc.RRT(si)
                self.planner.setGoalBias(self.mainWidget.plannerWidget.RRTGoalBias.value())
            
    def setRange(self, value):
        self.msgDebug('Changing range from %g to %g' %  (self.planner.getRange(), value))
        self.planner.setRange(value)
    def setGoalBias(self, value):
        self.msgDebug('Changing goal bias from %g to %g' %  (self.planner.getGoalBias(), value))
        self.planner.setGoalBias(value)
    def setBorderFraction(self, value):
        self.msgDebug('Changing border fraction from %g to %g' %  (self.planner.getBorderFraction(), value))
        self.planner.setBorderFraction(value)
    def setMaxNearestNeighbors(self, value):
        self.msgDebug('Changing max. nearest neighbors from %g to %g' %  (self.planner.getMaxNearestNeighbors(), value))
        self.planner.setMaxNearestNeighbors(value)
    def setTimeLimit(self, value):
        self.msgDebug('Changing time limit from %g to %g' % (self.timeLimit, value))
        self.timeLimit = value
    def solve(self):
        self.omplSetup.clear()
        startPose = self.convertToOmplPose(self.mainWidget.glViewer.startPose)
        goalPose = self.convertToOmplPose(self.mainWidget.glViewer.goalPose)
        self.omplSetup.setPlanner(self.planner)
        bounds = ob.RealVectorBounds(3)
        (bounds.low[0],bounds.low[1],bounds.low[2]) = self.mainWidget.glViewer.bounds_low
        (bounds.high[0],bounds.high[1],bounds.high[2]) = self.mainWidget.glViewer.bounds_high
        self.omplSetup.getSE3StateManifold().setBounds(bounds)
#        print [x for x in bounds.low ]
#        print [x for x in bounds.high ]
        self.omplSetup.setStartAndGoalStates(startPose, goalPose)
        if gConfig.geometricPlanning:
            self.omplSetup.getSpaceInformation().setStateValidityCheckingResolution(
                self.mainWidget.plannerWidget.resolution.value())
        else:
            self.omplSetup.getSpaceInformation().setPropagationStepSize(
                self.mainWidget.plannerWidget.propagation.value())
            self.omplSetup.getSpaceInformation().setMinMaxControlDuration(
                self.mainWidget.plannerWidget.minControlDuration.value(),
                self.mainWidget.plannerWidget.maxControlDuration.value())

        self.msgDebug(str(self.omplSetup))

        solved = self.omplSetup.solve(self.timeLimit)
        
        # update the planner data to render, if needed
        self.mainWidget.glViewer.plannerDataList = self.omplSetup.renderPlannerData()

        # update the displayed bounds, in case planning did so
        self.mainWidget.glViewer.setBounds(self.omplSetup.getSE3StateManifold().getBounds())
        if solved:
            self.omplSetup.simplifySolution()
            self.omplSetup.simplifySolution()
            self.omplSetup.simplifySolution()
            self.omplSetup.simplifySolution()
            self.path = self.omplSetup.getSolutionPath()
            ns = 100
            if len(self.path.states) < ns:
                self.path.interpolate(ns)
                if len(self.path.states) != ns:
                    self.msgError("Interpolation produced " + str(len(self.path.states)) + " states instead of " + str(ns) + " states!")
            if self.path.check() == False:
                self.msgError("Path reported by planner seems to be invalid!")
            self.mainWidget.glViewer.setSolutionPath(self.path)

    def clear(self):
        self.omplSetup.clear()
        self.mainWidget.glViewer.clear()

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

        self.windowMenu = self.menuBar().addMenu('&Window')
        self.windowMenu.addAction(self.logWindowAct)
        # self.windowMenu.addAction(self.commandWindowAct)

        self.helpMenu = self.menuBar().addMenu('Help')
        self.helpMenu.addAction(self.omplWebAct)
        self.helpMenu.addAction(self.contactDevsAct)
        self.helpMenu.addAction(self.emailListAct)

    def convertToOmplPose(self, pose):
        c = [ cos(angle*pi/360.) for angle in pose[:3] ]
        s = [ sin(angle*pi/360.) for angle in pose[:3] ]
        state = ob.State(self.omplSetup.getSE3StateManifold())
        rot = state().rotation()
        rot.w = c[0]*c[1]*c[2] + s[0]*s[1]*s[2]
        rot.x = s[0]*c[1]*c[2] - c[0]*s[1]*s[2]
        rot.y = c[0]*s[1]*c[2] + s[0]*c[1]*s[2]
        rot.z = c[0]*c[1]*s[2] - s[0]*s[1]*c[2]
        state().setX(pose[3])
        state().setY(pose[4])
        state().setZ(pose[5])
        return state

class MainWidget(QtGui.QWidget):
    def __init__(self, parent=None, flags=QtCore.Qt.WindowFlags(0)):
        super(MainWidget, self).__init__(parent, flags)
        self.glViewer = GLViewer()
        self.problemWidget = ProblemWidget()
        self.plannerWidget = PlannerWidget()
        self.boundsWidget = BoundsWidget()
        self.solveWidget = SolveWidget()
        tabWidget = QtGui.QTabWidget()
        tabWidget.addTab(self.problemWidget, "Problem")
        tabWidget.addTab(self.plannerWidget, "Planner")
        tabWidget.addTab(self.boundsWidget, "Bounding box")
        layout = QtGui.QGridLayout()
        layout.addWidget(self.glViewer, 0, 0)
        layout.addWidget(tabWidget, 0, 1)
        layout.addWidget(self.solveWidget, 1, 0, 1, 2)
        self.setLayout(layout)
        self.problemWidget.startPose.valueChanged.connect(self.glViewer.setStartPose)
        self.problemWidget.goalPose.valueChanged.connect(self.glViewer.setGoalPose)
        self.solveWidget.showData.toggled.connect(self.glViewer.toggleShowData)
        self.solveWidget.animateCheck.toggled.connect(self.glViewer.toggleAnimation)
        self.solveWidget.speedSlider.valueChanged.connect(self.glViewer.setSpeed)

class LogWindow(QtGui.QWidget):
    def __init__(self, parent=None, flags=QtCore.Qt.Tool):
        super(LogWindow, self).__init__(parent, flags)
#        self.logFile = tempfile.NamedTemporaryFile()
#        sys.stdout = self.logFile
#        sys.stderr = self.logFile
#        self.logWatch = QtCore.QFileSystemWatcher()
#        self.logWatch.addPath(self.logFile.name)
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
    boundLowChanged = QtCore.pyqtSignal(list)
    boundHighChanged = QtCore.pyqtSignal(list)

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

    def minimumSizeHint(self):
        return QtCore.QSize(500, 300)

    def sizeHint(self):
        return QtCore.QSize(500, 300)
    def maximumSize(self):
        return QtCore.QSize(1000, 1000)

    def setRotationAngle(self, axisIndex, angle):
        if angle != self.cameraPose[axisIndex]:
            self.cameraPose[axisIndex] = angle
            self.updateGL()
    def setBounds(self, bounds):
        self.bounds_low = [x for x in bounds.low ]
        self.bounds_high = [x for x in bounds.high ]
        bbox = zip(self.bounds_low, self.bounds_high)
        self.center = [ .5*(p0+p1) for (p0,p1) in bbox ]
        self.scale = 1. / max([p1-p0 for (p0,p1) in bbox ])
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
        self.updateBounds(self.startPose[3:])
        self.updateGL()
    def setGoalPose(self, value):
        self.goalPose = value
        self.updateBounds(self.startPose[3:])
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
        n = len(path.states)
        self.solutionPath = [ self.getTransform(path.states[i]) for i in range(n) ]
        self.pathIndex = 0
        self.updateGL()
    def setRobot(self, robot):
        if self.robot: GL.glDeleteLists(self.robot, 1)
        self.robot = robot
    def setEnvironment(self, environment):
        if self.environment: GL.glDeleteLists(self.environment, 1)
        self.environment = environment
    def clear(self):
        self.solutionPath = None
        self.plannerDataList = None
        self.pathIndex = 0
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
        R = xform.rotation()
        (w,x,y,z) = (R.w, -R.x, -R.y, -R.z)
        return [ w*w+x*x-y*y-z*z, 2*(x*y-w*z), 2*(x*z+w*y), 0,
            2*(x*y+w*z), w*w-x*x+y*y-z*z, 2*(y*z-w*x), 0,
            2*(x*z-w*y), 2*(y*z+w*x), w*w-x*x-y*y+z*z, 0,
            xform.getX(), xform.getY(), xform.getZ(), 1 ]

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
                    if n<100:
                        ind = range(0,n)
                    else:
                        step = float(n - 1.)/99.
                        ind = [int(step*i) for i in range(100)]
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
        if modifiers & QtCore.Qt.CTRL:
            print 'foo!'
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
    def __init__(self):
        super(ProblemWidget, self).__init__()
        self.startPose = PoseBox('Start Pose')
        self.goalPose = PoseBox('Goal Pose')
        layout = QtGui.QGridLayout()
        layout.addWidget(self.startPose, 0,0)
        layout.addWidget(self.goalPose, 1,0)
        self.setLayout(layout)

class PoseBox(QtGui.QGroupBox):
    valueChanged = QtCore.pyqtSignal(list)

    def __init__(self, title):
        super(PoseBox, self).__init__(title)
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
        layout.addWidget(poslabel, 0, 1, QtCore.Qt.AlignHCenter)
        layout.addWidget(rotlabel, 0, 2, QtCore.Qt.AlignHCenter)
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

    def setPose(self, value):
        self.posx.setValue(value().getX())
        self.posy.setValue(value().getY())
        self.posz.setValue(value().getZ())
        q = value().rotation()
        rad2deg = 180/pi
        self.rotx.setValue(rad2deg * atan2(2.*(q.w*q.x+q.y*q.z), 1.-2.*(q.x*q.x+q.y*q.y)))
        self.roty.setValue(rad2deg * asin(2.*(q.w*q.y-q.z*q.x)))
        self.rotz.setValue(rad2deg * atan2(2.*(q.w*q.z+q.x*q.y), 1.-2.*(q.y*q.y+q.z*q.z)))

    def poseChange(self, value):
        self.valueChanged.emit([self.rotx.value(), self.roty.value(), self.rotz.value(),
            self.posx.value(), self.posy.value(), self.posz.value() ])

class PlannerWidget(QtGui.QWidget):
    def __init__(self):
        super(PlannerWidget, self).__init__()

        # list of planners
        plannerLabel = QtGui.QLabel('Planner')
        self.plannerSelect = QtGui.QComboBox()
        if gConfig.geometricPlanning:
            self.plannerSelect.addItem('KPIECE')
            self.plannerSelect.addItem('Lazy Bi-directional KPIECE')
            self.plannerSelect.addItem('BasicPRM')
            self.plannerSelect.addItem('SBL')
            self.plannerSelect.addItem('RRT Connect')
            self.plannerSelect.addItem('RRT')
            self.plannerSelect.addItem('Lazy RRT')
            self.plannerSelect.addItem('EST')
        else:
            self.plannerSelect.addItem('KPIECE')
            self.plannerSelect.addItem('RRT')

        self.plannerSelect.setMinimumContentsLength(10)
        self.plannerSelect.setSizeAdjustPolicy(QtGui.QComboBox.AdjustToMinimumContentsLength)

        if gConfig.geometricPlanning:
            # KPIECE options
            self.KPIECEOptions = QtGui.QGroupBox('KPIECE Options')
            KPIECErangeLabel = QtGui.QLabel('Range')
            self.KPIECERange = QtGui.QDoubleSpinBox()
            self.KPIECERange.setRange(0, 10000)
            self.KPIECERange.setSingleStep(1)
            self.KPIECERange.setValue(0)
            KPIECEgoalBiasLabel = QtGui.QLabel('Goal Bias')
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
            
            # LBKPIECE options
            self.LBKPIECEOptions = QtGui.QGroupBox('LBKPIECE Options')
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
            self.PRMOptions = QtGui.QGroupBox('BasicPRM Options')
            PRMmaxNearestNeighborsLabel = QtGui.QLabel('Max. Nearest Neighbors')
            self.PRMMaxNearestNeighbors = QtGui.QSpinBox()
            self.PRMMaxNearestNeighbors.setRange(0, 1000)
            self.PRMMaxNearestNeighbors.setSingleStep(1)
            self.PRMMaxNearestNeighbors.setValue(10)
            layout = QtGui.QGridLayout()
            layout.addWidget(PRMmaxNearestNeighborsLabel, 0, 0, QtCore.Qt.AlignRight)
            layout.addWidget(self.PRMMaxNearestNeighbors, 0, 1)
            self.PRMOptions.setLayout(layout)
            
            # SBL options
            self.SBLOptions = QtGui.QGroupBox('SBL Options')
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
            self.RRTConnectOptions = QtGui.QGroupBox('RRT Connect Options')
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
            self.RRTOptions = QtGui.QGroupBox('RRT Options')
            RRTrangeLabel = QtGui.QLabel('Range')
            self.RRTRange = QtGui.QDoubleSpinBox()
            self.RRTRange.setRange(0, 10000)
            self.RRTRange.setSingleStep(1)
            self.RRTRange.setValue(0)
            RRTgoalBiasLabel = QtGui.QLabel('Goal Bias')
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
            self.LazyRRTOptions = QtGui.QGroupBox('Lazy RRT Options')
            LazyRRTrangeLabel = QtGui.QLabel('Range')
            self.LazyRRTRange = QtGui.QDoubleSpinBox()
            self.LazyRRTRange.setRange(0, 10000)
            self.LazyRRTRange.setSingleStep(1)
            self.LazyRRTRange.setValue(0)
            LazyRRTgoalBiasLabel = QtGui.QLabel('Goal Bias')
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
            self.ESTOptions = QtGui.QGroupBox('EST Options')
            ESTgoalBiasLabel = QtGui.QLabel('Goal Bias')
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
        else:
            # KPIECE options
            self.KPIECEOptions = QtGui.QGroupBox('KPIECE Options')
            KPIECEgoalBiasLabel = QtGui.QLabel('Goal Bias')
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
            self.RRTOptions = QtGui.QGroupBox('RRT Options')
            RRTgoalBiasLabel = QtGui.QLabel('Goal Bias')
            self.RRTGoalBias = QtGui.QDoubleSpinBox()
            self.RRTGoalBias.setRange(0, 1)
            self.RRTGoalBias.setSingleStep(.05)
            self.RRTGoalBias.setValue(0.05)
            layout = QtGui.QGridLayout()
            layout.addWidget(RRTgoalBiasLabel, 0, 0, QtCore.Qt.AlignRight)
            layout.addWidget(self.RRTGoalBias, 0, 1)
            self.RRTOptions.setLayout(layout)
            
        self.stackedWidget = QtGui.QStackedWidget()
        
        if gConfig.geometricPlanning:
            self.stackedWidget.addWidget(self.KPIECEOptions)
            self.stackedWidget.addWidget(self.LBKPIECEOptions)
            self.stackedWidget.addWidget(self.PRMOptions)
            self.stackedWidget.addWidget(self.SBLOptions)
            self.stackedWidget.addWidget(self.RRTConnectOptions)
            self.stackedWidget.addWidget(self.RRTOptions)
            self.stackedWidget.addWidget(self.LazyRRTOptions)
            self.stackedWidget.addWidget(self.ESTOptions)
        else:
            self.stackedWidget.addWidget(self.KPIECEOptions)
            self.stackedWidget.addWidget(self.RRTOptions)

        self.plannerSelect.activated.connect(self.stackedWidget.setCurrentIndex)

        timeLimitLabel = QtGui.QLabel('Time (sec.)')
        self.timeLimit = QtGui.QDoubleSpinBox()
        self.timeLimit.setRange(0, 1000)
        self.timeLimit.setSingleStep(1)
        self.timeLimit.setValue(10.0)

        layout = QtGui.QGridLayout()
        layout.addWidget(plannerLabel, 0, 0, QtCore.Qt.AlignRight)
        layout.addWidget(self.plannerSelect, 0, 1)
        layout.addWidget(timeLimitLabel, 1, 0, QtCore.Qt.AlignRight)
        layout.addWidget(self.timeLimit, 1, 1)

        if gConfig.geometricPlanning:
            resolutionLabel = QtGui.QLabel('Collision checking\nresolution')
            resolutionLabel.setAlignment(QtCore.Qt.AlignRight)
            self.resolution = QtGui.QDoubleSpinBox()
            self.resolution.setRange(0.001, 1.0)
            self.resolution.setSingleStep(.002)
            self.resolution.setValue(0.010)
            self.resolution.setDecimals(3)
            layout.addWidget(resolutionLabel, 2, 0, QtCore.Qt.AlignRight)
            layout.addWidget(self.resolution, 2, 1)
        else:
            propagationLabel = QtGui.QLabel('Propagation step\nsize')
            propagationLabel.setAlignment(QtCore.Qt.AlignRight)
            self.propagation = QtGui.QDoubleSpinBox()
            self.propagation.setRange(0.01, 1000.00)
            self.propagation.setSingleStep(.01)
            self.propagation.setValue(0.01)
            self.propagation.setDecimals(2)
            layout.addWidget(propagationLabel, 2, 0, QtCore.Qt.AlignRight)
            layout.addWidget(self.propagation, 2, 1)
            durationLabel = QtGui.QLabel('Control duration')
            durationLabel.setAlignment(QtCore.Qt.AlignRight)
            layout.addWidget(durationLabel, 3, 0)
            self.minControlDuration = QtGui.QSpinBox()
            self.minControlDuration.setRange(1, 1000)
            self.minControlDuration.setSingleStep(1)
            self.minControlDuration.setValue(2)
            self.maxControlDuration = QtGui.QSpinBox()
            self.maxControlDuration.setRange(1, 1000)
            self.maxControlDuration.setSingleStep(1)
            self.maxControlDuration.setValue(20)
            ctrlLayout = QtGui.QGridLayout()
            ctrlLayout.addWidget(self.minControlDuration, 0, 0)
            ctrlLayout.addWidget(self.maxControlDuration, 0, 1)
            layout.addLayout(ctrlLayout, 3, 1)

        layout.addWidget(self.stackedWidget, 4, 0, 1, 2)
        self.setLayout(layout)


class BoundsWidget(QtGui.QWidget):
    def __init__(self):
        super(BoundsWidget, self).__init__()
        self.bounds_high = BoundsBox('Upper bounds')
        self.bounds_low = BoundsBox('Lower bounds')
        layout = QtGui.QGridLayout()
        layout.addWidget(self.bounds_high, 0,0)
        layout.addWidget(self.bounds_low, 1,0)
        self.setLayout(layout)

class BoundsBox(QtGui.QGroupBox):
    valueChanged = QtCore.pyqtSignal(list)

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

    # command line configuration for gConfig ?

    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
