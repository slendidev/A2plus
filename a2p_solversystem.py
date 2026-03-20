# -*- coding: utf-8 -*-
#***************************************************************************
#*                                                                         *
#*   Copyright (c) 2018 kbwbe                                              *
#*                                                                         *
#*   This program is free software; you can redistribute it and/or modify  *
#*   it under the terms of the GNU Lesser General Public License (LGPL)    *
#*   as published by the Free Software Foundation; either version 2 of     *
#*   the License, or (at your option) any later version.                   *
#*   for detail see the LICENCE text file.                                 *
#*                                                                         *
#*   This program is distributed in the hope that it will be useful,       *
#*   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
#*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
#*   GNU Library General Public License for more details.                  *
#*                                                                         *
#*   You should have received a copy of the GNU Library General Public     *
#*   License along with this program; if not, write to the Free Software   *
#*   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  *
#*   USA                                                                   *
#*                                                                         *
#***************************************************************************

import os
import time
import FreeCAD, FreeCADGui
from FreeCAD import Base
from PySide import QtGui, QtCore
#from a2p_translateUtils import *
import threading
import a2plib
from a2plib import (
    path_a2p,
    Msg,
    DebugMsg,
    A2P_DEBUG_LEVEL,
    A2P_DEBUG_1,
    PARTIAL_SOLVE_STAGE1,
    )
from a2p_dependencies import Dependency
from a2p_rigid import Rigid

SOLVER_MAXSTEPS = 50000
translate = FreeCAD.Qt.translate

# SOLVER_CONTROLDATA has been replaced by SolverSystem.getSolverControlData()
#SOLVER_CONTROLDATA = {
#    #Index:(posAccuracy,spinAccuracy,completeSolvingRequired)
#    1:(0.1,0.1,True),
#    2:(0.01,0.01,True),
#    3:(0.001,0.001,True),
#    4:(0.0001,0.0001,False),
#    5:(0.00001,0.00001,False)
#    }

SOLVER_POS_ACCURACY = 1.0e-1  # gets to smaller values during solving
SOLVER_SPIN_ACCURACY = 1.0e-1 # gets to smaller values during solving

SOLVER_STEPS_CONVERGENCY_CHECK = 2000 #200
SOLVER_CONVERGENCY_FACTOR = 0.99
SOLVER_CONVERGENCY_ERROR_INIT_VALUE = 1.0e+20
FAST_SOLVER_MAXSTEPS = 12000
FAST_SOLVER_STEPS_CONVERGENCY_CHECK = 500
FAST_SOLVER_CONVERGENCY_FACTOR = 0.999

#------------------------------------------------------------------------------
# Threaded Solver Infrastructure
#------------------------------------------------------------------------------

class _SolverSignalHelper(QtCore.QObject):
    """
    Helper QObject that lives on main thread to receive cross-thread signals.
    Must be created on main thread.
    """
    solveFinished = QtCore.Signal(bool, object, object, object)  # success, solver, doc, callback
    
    def __init__(self, parent=None):
        super(_SolverSignalHelper, self).__init__(parent)
        self.solveFinished.connect(self._handleFinished, QtCore.Qt.QueuedConnection)
        # Ensure we're on the main thread
        if QtGui.QApplication.instance() is not None:
            self.moveToThread(QtGui.QApplication.instance().thread())
    
    def _handleFinished(self, success, solver, doc, callback):
        """Called on main thread when solver completes."""
        if doc is not None and success:
            # Apply placements on main thread
            solver.solutionToParts(doc)
            solver.status = "solved"
            touchedObjects = [doc.getObject(name) for name in solver.touchedObjectNames]
            a2plib.unTouchA2pObjects(touchedObjects)
        
        if callback:
            try:
                callback(success, solver)
            except Exception as e:
                Msg("Async solver callback error: {}\n".format(str(e)))


# Global signal helper - created lazily on first use (must be on main thread)
_SOLVER_SIGNAL_HELPER = None

def _getSolverSignalHelper():
    global _SOLVER_SIGNAL_HELPER
    if _SOLVER_SIGNAL_HELPER is None:
        _SOLVER_SIGNAL_HELPER = _SolverSignalHelper()
    return _SOLVER_SIGNAL_HELPER


class AsyncSolver(object):
    """
    Manages async solving using Python threading.
    Uses Qt signals with QueuedConnection to safely call back to main thread.
    
    Usage:
        asyncSolver = AsyncSolver()
        asyncSolver.solveAsync(doc, callback)
    """
    def __init__(self):
        self._thread = None
        self._cancelled = False
        self._lock = threading.Lock()
        self._callback = None
        self._doc = None
        self._solver = None
    
    def isBusy(self):
        with self._lock:
            return self._thread is not None and self._thread.is_alive()
    
    def cancel(self):
        with self._lock:
            self._cancelled = True
    
    def _isCancelled(self):
        with self._lock:
            return self._cancelled
    
    def solveAsync(self, doc, callback=None, matelist=None, fastMode=True, 
                   checkFaultyConstraints=False, cache=None):
        """
        Start an async solve. Returns immediately.
        callback(success, solver) is called on main thread when done.
        """
        # If busy, cancel and wait briefly
        if self.isBusy():
            self.cancel()
            if self._thread is not None:
                self._thread.join(timeout=0.5)
        
        with self._lock:
            self._cancelled = False
        
        self._doc = doc
        self._callback = callback
        
        # Get or create session and solver (must happen on main thread - accesses doc)
        changedConstraints, skipConstraintRefresh = _sessionOptionsFromCache(cache)
        session = _getSolverSession(doc)
        
        # Use the session's getSolver which handles caching and refresh
        solver, isSynced = session.getSolver(
            matelist,
            fastMode,
            checkFaultyConstraints,
            changedConstraints=changedConstraints,
            skipConstraintRefresh=skipConstraintRefresh
        )
        
        if solver.status == "loadingDependencyError":
            if callback:
                callback(False, solver)
            return
        
        self._solver = solver
        solverControlData = solver._prepareAccuracyLoop()
        
        # Get signal helper NOW on main thread (not in worker thread)
        signalHelper = _getSolverSignalHelper()
        
        # Start worker thread, passing the helper reference
        self._thread = threading.Thread(
            target=self._workerRun,
            args=(solver, doc, solverControlData, callback, signalHelper),
            daemon=True
        )
        self._thread.start()
    
    def _workerRun(self, solver, doc, solverControlData, callback, signalHelper):
        """Run in background thread - only does computation, no Qt calls."""
        success = False
        try:
            # Inject cancellation check into solver
            solver._asyncCancelCheck = self._isCancelled
            solver._asyncProgressCallback = None  # No progress for now (would need queue)
            
            success = solver._runAccuracyLoop(doc, solverControlData)
            
            if self._isCancelled():
                success = False
        except Exception as e:
            Msg("Solver thread error: {}\n".format(str(e)))
            success = False
        finally:
            solver._asyncCancelCheck = None
            solver._asyncProgressCallback = None
        
        # Clear thread reference
        with self._lock:
            self._thread = None
        
        # Marshal callback to main thread using Qt signal with QueuedConnection
        # signalHelper was created on main thread, so emit is thread-safe
        signalHelper.solveFinished.emit(success, solver, doc, callback)


# Global async solver instance for simple usage
_ASYNC_SOLVER = None

def getAsyncSolver():
    global _ASYNC_SOLVER
    if _ASYNC_SOLVER is None:
        _ASYNC_SOLVER = AsyncSolver()
    return _ASYNC_SOLVER

def solveConstraintsAsync(doc, callback=None, cache=None, matelist=None, fastMode=True):
    """
    Non-blocking solve. Returns immediately.
    callback(success, solver) called on main thread when done.
    """
    asyncSolver = getAsyncSolver()
    asyncSolver.solveAsync(doc, callback, matelist, fastMode, 
                           checkFaultyConstraints=False, cache=cache)
    return asyncSolver

#------------------------------------------------------------------------------

class SolverSystem():
    """
    class Solversystem():
    A new iterative solver, inspired by physics.
    Using "attraction" of parts by constraints
    """

    def __init__(self):
        self.doc = None
        self.stepCount = 0
        self.rigids = []        # list of rigid bodies
        self.constraints = []
        self.objectNames = []
        self.rigidByName = {}
        self.mySOLVER_SPIN_ACCURACY = SOLVER_SPIN_ACCURACY
        self.mySOLVER_POS_ACCURACY = SOLVER_POS_ACCURACY
        self.lastPositionError = SOLVER_CONVERGENCY_ERROR_INIT_VALUE
        self.lastAxisError = SOLVER_CONVERGENCY_ERROR_INIT_VALUE
        self.convergencyCounter = 0
        self.status = "created"
        self.partialSolverCurrentStage = 0
        self.currentstage = 0
        self.solvedCounter = 0
        self.maxPosError = 0.0
        self.maxAxisError = 0.0
        self.maxSingleAxisError = 0.0
        self.unmovedParts = []
        # Initialize cache dictionary to store positions of rigids and their solutions
        self.rigid_positions_cache = {}
        self.fastMode = False
        self.applyPlacementPosFactor = 1.0e-2
        self.applyPlacementSpinFactor = 1.0e-2
        self.maxSteps = SOLVER_MAXSTEPS
        self.convergencyCheckSteps = SOLVER_STEPS_CONVERGENCY_CHECK
        self.convergencyFactor = SOLVER_CONVERGENCY_FACTOR
        self.touchedObjectNames = set()
        self.timingLoadMs = 0.0
        self.timingChainMs = 0.0
        # Async solver hooks (set by SolverWorker when running in thread)
        self._asyncCancelCheck = None      # callable() -> bool, returns True to cancel
        self._asyncProgressCallback = None # callable(stepCount, maxPosError)

    def clear(self):
        for r in self.rigids:
            r.clear()
        self.stepCount = 0
        self.rigids = []
        self.constraints = []
        self.objectNames = []
        self.rigidByName = {}
        self.partialSolverCurrentStage = PARTIAL_SOLVE_STAGE1
        self.touchedObjectNames = set()
        self.timingLoadMs = 0.0
        self.timingChainMs = 0.0

    def configureMode(self, fastMode):
        self.fastMode = fastMode
        if fastMode:
            self.applyPlacementPosFactor = 3.0e-2
            self.applyPlacementSpinFactor = 3.0e-2
            self.maxSteps = FAST_SOLVER_MAXSTEPS
            self.convergencyCheckSteps = FAST_SOLVER_STEPS_CONVERGENCY_CHECK
            self.convergencyFactor = FAST_SOLVER_CONVERGENCY_FACTOR
        else:
            self.applyPlacementPosFactor = 1.0e-2
            self.applyPlacementSpinFactor = 1.0e-2
            self.maxSteps = SOLVER_MAXSTEPS
            self.convergencyCheckSteps = SOLVER_STEPS_CONVERGENCY_CHECK
            self.convergencyFactor = SOLVER_CONVERGENCY_FACTOR

    def getSolverControlData(self):
        if a2plib.SIMULATION_STATE:
            # do less accurate solving for simulations...
            solverControlData = {
                #Index:(posAccuracy,spinAccuracy,completeSolvingRequired)
                1:(0.1,0.1,True)
                }
        else:
            solverControlData = {
                #Index:(posAccuracy,spinAccuracy,completeSolvingRequired)
                1:(0.1,0.1,True),
                2:(0.01,0.01,True),
                3:(0.001,0.001,False),
                4:(0.0001,0.0001,False),
                5:(0.00001,0.00001,False)
                }
        return solverControlData


    def getRigid(self,objectName):
        """Get a Rigid by objectName."""
        return self.rigidByName.get(objectName)

    def removeFaultyConstraints(self, doc):
        """
        Remove constraints where referenced objects do not exist anymore.
        """
        constraints = [ obj for obj in doc.Objects if 'ConstraintInfo' in obj.Content]

        faultyConstraintList = []
        for c in constraints:
            constraintOK = True
            for attr in ['Object1','Object2']:
                objectName = getattr(c, attr, None)
                o = doc.getObject(objectName)
                if o is None:
                    constraintOK = False
            if not constraintOK:
                faultyConstraintList.append(c)

        if len(faultyConstraintList) > 0:
            for fc in faultyConstraintList:
                FreeCAD.Console.PrintMessage(translate("A2plus", "Remove faulty constraint '{}'").format(fc.Label) + "\n")
                doc.removeObject(fc.Name)

    def loadSystem(self, doc, matelist=None, checkFaultyConstraints=True, buildDOFInfo=False, geometryCache=None):
        self.clear()
        self.doc = doc
        self.status = "loading"

        if checkFaultyConstraints:
            self.removeFaultyConstraints(doc)

        self.convergencyCounter = 0
        self.lastPositionError = SOLVER_CONVERGENCY_ERROR_INIT_VALUE
        self.lastAxisError = SOLVER_CONVERGENCY_ERROR_INIT_VALUE
        #
        self.constraints = []
        constraints =[]             # temporary list
        if matelist is not None:        # Transfer matelist to the temp list
            for obj in matelist:
                if 'ConstraintInfo' in obj.Content:
                    constraints.append(obj)
        else:
            # if there is not a list of my mates get the list from the doc
            constraints = [ obj for obj in doc.Objects if 'ConstraintInfo' in obj.Content]
        # check for Suppressed mates here and transfer mates to self.constraints
        for obj in constraints:
            if hasattr(obj, 'Suppressed') and obj.Suppressed:
                continue
            self.constraints.append(obj)
        #
        # Extract all the objectnames which are affected by constraints..
        self.objectNames = []
        objectNamesSet = set()
        for c in self.constraints:
            for attr in ['Object1','Object2']:
                objectName = getattr(c, attr, None)
                if objectName is not None and objectName not in objectNamesSet:
                    objectNamesSet.add(objectName)
                    self.objectNames.append(objectName)
        self.touchedObjectNames = set(self.objectNames)
        #
        # create a Rigid() dataStructure for each of these objectnames...
        for o in self.objectNames:
            ob1 = doc.getObject(o)
            if hasattr(ob1, "fixedPosition"):
                fx = ob1.fixedPosition
            else:
                fx = False
            if hasattr(ob1, "debugmode"):
                debugMode = ob1.debugmode
            else:
                debugMode = False
            rig = Rigid(
                o,
                ob1.Label,
                fx,
                ob1.Placement,
                debugMode
                )
            if buildDOFInfo:
                rig.rigidCenter = ob1.Shape.BoundBox.Center
            else:
                rig.rigidCenter = Base.Vector(ob1.Placement.Base)
            rig.spinCenter = Base.Vector(rig.rigidCenter)
            self.rigids.append(rig)
            self.rigidByName[o] = rig
        #
        # link constraints to rigids using dependencies
        deleteList = [] # a list to collect broken constraints
        # Use provided geometry cache or create a fresh one
        depCache = geometryCache if geometryCache is not None else {}
        for c in self.constraints:
            rigid1 = self.getRigid(c.Object1)
            rigid2 = self.getRigid(c.Object2)

            # create and update list of constrained rigids
            if rigid2 is not None and not rigid2 in rigid1.linkedRigids: rigid1.linkedRigids.append(rigid2);
            if rigid1 is not None and not rigid1 in rigid2.linkedRigids: rigid2.linkedRigids.append(rigid1);

            try:
                Dependency.Create(doc, c, self, rigid1, rigid2, depCache)
            except:
                self.status = "loadingDependencyError"
                deleteList.append(c)


        for rig in self.rigids:
            rig.hierarchyLinkedRigids.extend(rig.linkedRigids)

        if len(deleteList) > 0:
            msg = translate("A2plus", "The following constraints are broken:") + "\n"
            for c in deleteList:
                msg += "{}\n".format(c.Label)
            msg += translate("A2plus", "Do you want to delete them?")

            flags = QtGui.QMessageBox.StandardButton.Yes | QtGui.QMessageBox.StandardButton.No
            response = QtGui.QMessageBox.critical(
                QtGui.QApplication.activeWindow(),
                translate("A2plus", "Delete broken constraints?"),
                msg,
                flags
                )
            if response == QtGui.QMessageBox.Yes:
                for c in deleteList:
                    a2plib.removeConstraint(c)

        if self.status == "loadingDependencyError":
            return

        for rig in self.rigids:
            rig.calcSpinCenter()
            rig.calcRefPointsBoundBoxSize()

        if buildDOFInfo:
            self.retrieveDOFInfo() # function only once used here at this place in whole program
        self.status = "loaded"

    def syncFromDocument(self, doc):
        if doc is None:
            return False
        self.doc = doc
        for rig in self.rigids:
            ob = doc.getObject(rig.objectName)
            if ob is None:
                return False
            rig.fixed = getattr(ob, 'fixedPosition', False)
            rig.debugMode = getattr(ob, 'debugmode', False)
            rig.syncToPlacement(ob.Placement)
        self.touchedObjectNames = set(self.objectNames)
        return True

    def DOF_info_to_console(self):
        doc = FreeCAD.activeDocument()

        dofGroup = doc.getObject("dofLabels")
        if dofGroup is None:
            dofGroup=doc.addObject("App::DocumentObjectGroup", "dofLabels")
        else:
            for lbl in dofGroup.Group:
                doc.removeObject(lbl.Name)
            doc.removeObject("dofLabels")
            dofGroup=doc.addObject("App::DocumentObjectGroup", "dofLabels")

        self.loadSystem(doc, buildDOFInfo=True)

        # look for unconstrained objects and label them
        solverObjectNames = []
        for rig in self.rigids:
            solverObjectNames.append(rig.objectName)
        shapeObs = a2plib.filterShapeObs(doc.Objects)
        for so in shapeObs:
            if so.Name not in solverObjectNames:
                ob = doc.getObject(so.Name)
                if ob.ViewObject.Visibility == True:
                    bbCenter = ob.Shape.BoundBox.Center
                    dofLabel = doc.addObject("App::AnnotationLabel","dofLabel")
                    dofLabel.LabelText = translate("A2plus", "FREE")
                    dofLabel.BasePosition.x = bbCenter.x
                    dofLabel.BasePosition.y = bbCenter.y
                    dofLabel.BasePosition.z = bbCenter.z
                    #
                    dofLabel.ViewObject.BackgroundColor = a2plib.BLUE
                    dofLabel.ViewObject.TextColor = a2plib.WHITE
                    dofGroup.addObject(dofLabel)


        numdep = 0
        self.retrieveDOFInfo() #function only once used here at this place in whole program
        for rig in self.rigids:
            dofCount = rig.currentDOF()
            ob = doc.getObject(rig.objectName)
            if ob.ViewObject.Visibility == True:
                bbCenter = ob.Shape.BoundBox.Center
                dofLabel = doc.addObject("App::AnnotationLabel","dofLabel")
                if rig.fixed:
                    dofLabel.LabelText = translate("A2plus", "Fixed")
                else:
                    dofLabel.LabelText = translate("A2plus", "DOFs: {}").format(dofCount)
                dofLabel.BasePosition.x = bbCenter.x
                dofLabel.BasePosition.y = bbCenter.y
                dofLabel.BasePosition.z = bbCenter.z

                if rig.fixed:
                    dofLabel.ViewObject.BackgroundColor = a2plib.RED
                    dofLabel.ViewObject.TextColor = a2plib.BLACK
                elif dofCount == 0:
                    dofLabel.ViewObject.BackgroundColor = a2plib.RED
                    dofLabel.ViewObject.TextColor = a2plib.BLACK
                elif dofCount < 6:
                    dofLabel.ViewObject.BackgroundColor = a2plib.YELLOW
                    dofLabel.ViewObject.TextColor = a2plib.BLACK
                dofGroup.addObject(dofLabel)


            rig.beautyDOFPrint()
            numdep+=rig.countDependencies()
        Msg(translate("A2plus", "There are {:.0f} dependencies").format(numdep/2) + "\n")

    def retrieveDOFInfo(self):
        """
        Method used to retrieve all info related to DOF handling.
        the method scans each rigid, and on each not tempfixed rigid scans the list of linkedobjects
        then for each linked object compile a dict where each linked object has its dependencies
        then for each linked object compile a dict where each linked object has its dof position
        then for each linked object compile a dict where each linked object has its dof rotation
        """
        for rig in self.rigids:

            #if not rig.tempfixed:  #skip already fixed objs

            for linkedRig in rig.linkedRigids:
                tmplinkedDeps = []
                tmpLinkedPointDeps = []
                for dep in rig.dependencies:
                    if linkedRig==dep.dependedRigid:
                        #be sure pointconstraints are at the end of the list
                        if dep.isPointConstraint :
                            tmpLinkedPointDeps.append(dep)
                        else:
                            tmplinkedDeps.append(dep)
                #add at the end the point constraints
                tmplinkedDeps.extend(tmpLinkedPointDeps)
                rig.depsPerLinkedRigids[linkedRig] = tmplinkedDeps

            #dofPOSPerLinkedRigid is a dict where for each
            for linkedRig in rig.depsPerLinkedRigids.keys():
                linkedRig.pointConstraints = []
                _dofPos = linkedRig.posDOF
                _dofRot = linkedRig.rotDOF
                for dep in rig.depsPerLinkedRigids[linkedRig]:
                    _dofPos, _dofRot = dep.calcDOF(_dofPos,_dofRot, linkedRig.pointConstraints)
                rig.dofPOSPerLinkedRigids[linkedRig] = _dofPos
                rig.dofROTPerLinkedRigids[linkedRig] = _dofRot

            #ok each rigid has a dict for each linked objects,
            #so we now know the list of linked objects and which
            #dof rot and pos both limits.



    # TODO: maybe instead of traversing from the root every time, save a list of objects on current distance
    # and use them to propagate next distance to their children
    def assignParentship(self, doc):
        # Start from fixed parts
        for rig in self.rigids:
            if rig.fixed:
                rig.disatanceFromFixed = 0
                haveMore = True
                distance = 0
                while haveMore:
                    haveMore = rig.assignParentship(distance)
                    distance += 1

        if A2P_DEBUG_LEVEL > 0:
            Msg(20*"=" + "\n")
            Msg(translate("A2plus", "Hierarchy:") + "\n")
            Msg(20*"=" + "\n")
            for rig in self.rigids:
                if rig.fixed: rig.printHierarchy(0)
            Msg(20*"=" + "\n")

        #self.visualizeHierarchy()

    def visualizeHierarchy(self):
        '''
        Generate an html file with constraints structure.

        The html file is in the same folder
        with the same filename of the assembly
        '''
        out_file = os.path.splitext(self.doc.FileName)[0] + '_asm_hierarchy.html'
        Msg(translate("A2plus", "Writing visual hierarchy to: '{}'").format(out_file) + "\n")
        f = open(out_file, "w")

        f.write("<!DOCTYPE html>\n")
        f.write("<html>\n")
        f.write("<head>\n")
        f.write('    <meta charset="utf-8">\n')
        f.write('    <meta http-equiv="X-UA-Compatible" content="IE=edge">\n')
        f.write('    <title>' + translate("A2plus", "A2P assembly hierarchy visualization") + '</title>\n')
        f.write("</head>\n")
        f.write("<body>\n")
        f.write('<div class="mermaid">\n')

        f.write("graph TD\n")
        for rig in self.rigids:
            rigLabel = a2plib.to_str(rig.label).replace(u' ',u'_')
            # No children, add current rogod as a leaf entry
            if len(rig.childRigids) == 0:
                message = u"{}\n".format(rigLabel)
                f.write(message)
            else:
                # Rigid have children, add them based on the dependency list
                for d in rig.dependencies:
                    if d.dependedRigid in rig.childRigids:
                        dependedRigLabel = a2plib.to_str(d.dependedRigid.label).replace(u' ',u'_')
                        if rig.fixed:
                            message = "{}({}<br>*" + translate("A2plus", "FIXED") + "*) -- {} --> {}\n".format(rigLabel, rigLabel, d.Type, dependedRigLabel)
                            f.write(message)
                        else:
                            message = u"{} -- {} --> {}\n".format(rigLabel, d.Type, dependedRigLabel)
                            f.write(message)

        f.write("</div>\n")
        f.write('    <script src="https://unpkg.com/mermaid@7.1.2/dist/mermaid.js"></script>\n')
        f.write("    <script>\n")
        f.write('        mermaid.initialize({startOnLoad: true});\n')
        f.write("    </script>\n")
        f.write("</body>")
        f.write("</html>")
        f.close()

    def calcMoveData(self,doc):
        for rig in self.rigids:
            rig.calcMoveData(doc, self)

    def prepareRestart(self):
        for rig in self.rigids:
            rig.prepareRestart()
        self.partialSolverCurrentStage = PARTIAL_SOLVE_STAGE1

    def detectUnmovedParts(self):
        doc = FreeCAD.activeDocument()
        self.unmovedParts = []
        for rig in self.rigids:
            if rig.fixed: continue
            if not rig.moved:
                self.unmovedParts.append(
                    doc.getObject(rig.objectName)
                    )

    def _prepareAccuracyLoop(self):
        solverControlData = self.getSolverControlData()
        self.level_of_accuracy = 1
        self.mySOLVER_POS_ACCURACY = solverControlData[self.level_of_accuracy][0]
        self.mySOLVER_SPIN_ACCURACY = solverControlData[self.level_of_accuracy][1]
        self.lastPositionError = SOLVER_CONVERGENCY_ERROR_INIT_VALUE
        self.lastAxisError = SOLVER_CONVERGENCY_ERROR_INIT_VALUE
        self.convergencyCounter = 0
        self.maxAxisError = 0.0
        self.maxSingleAxisError = 0.0
        self.maxPosError = 0.0
        self.timingChainMs = 0.0
        self.unmovedParts = []
        for rig in self.rigids:
            rig.moved = False
        return solverControlData

    def _runAccuracyLoop(self, doc, solverControlData):
        while True:
            self.prepareRestart()
            tChain0 = time.perf_counter()
            systemSolved = self.calculateChain(doc)
            self.timingChainMs += (time.perf_counter() - tChain0) * 1000.0
            if self.level_of_accuracy == 1:
                self.detectUnmovedParts()
            if a2plib.SOLVER_ONESTEP > 0:
                systemSolved = True
                break
            if systemSolved:
                self.level_of_accuracy += 1
                if self.level_of_accuracy > len(solverControlData):
                    self.solutionToParts(doc)
                    break
                self.mySOLVER_POS_ACCURACY = solverControlData[self.level_of_accuracy][0]
                self.mySOLVER_SPIN_ACCURACY = solverControlData[self.level_of_accuracy][1]
            else:
                completeSolvingRequired = solverControlData[self.level_of_accuracy][2]
                if not completeSolvingRequired:
                    systemSolved = True
                break

        self.maxAxisError = 0.0
        self.maxSingleAxisError = 0.0
        self.maxPosError = 0.0
        for rig in self.rigids:
            if rig.maxPosError > self.maxPosError:
                self.maxPosError = rig.maxPosError
            if rig.maxAxisError > self.maxAxisError:
                self.maxAxisError = rig.maxAxisError
            if rig.maxSingleAxisError > self.maxSingleAxisError:
                self.maxSingleAxisError = rig.maxSingleAxisError
        if not a2plib.SIMULATION_STATE:
            Msg(translate("A2plus", "TARGET   POS-ACCURACY :{}").format(self.mySOLVER_POS_ACCURACY) + "\n")
            Msg(translate("A2plus", "REACHED  POS-ACCURACY :{}").format(self.maxPosError) + "\n")
            Msg(translate("A2plus", "TARGET  SPIN-ACCURACY :{}").format(self.mySOLVER_SPIN_ACCURACY) + "\n")
            Msg(translate("A2plus", "REACHED SPIN-ACCURACY :{}").format(self.maxAxisError) + "\n")
            Msg(translate("A2plus", "SA      SPIN-ACCURACY :{}").format(self.maxSingleAxisError) + "\n")
        return systemSolved

    def solveAccuracySteps(self,doc, matelist=None, checkFaultyConstraints=True):
        solverControlData = self._prepareAccuracyLoop()
        tLoad0 = time.perf_counter()
        self.loadSystem(doc, matelist, checkFaultyConstraints, buildDOFInfo=False)
        self.timingLoadMs = (time.perf_counter() - tLoad0) * 1000.0
        if self.status == "loadingDependencyError":
            return
        return self._runAccuracyLoop(doc, solverControlData)

    def solveAccuracyStepsLoaded(self, doc):
        solverControlData = self._prepareAccuracyLoop()
        return self._runAccuracyLoop(doc, solverControlData)

    def solveSystem(self,doc,matelist=None, showFailMessage=True, checkFaultyConstraints=True):
        if not a2plib.SIMULATION_STATE:
            Msg("===== " + translate("A2plus", "Start Solving System") + " =====\n")

        systemSolved = self.solveAccuracySteps(doc, matelist, checkFaultyConstraints)
        if self.status == "loadingDependencyError":
            return systemSolved
        if systemSolved:
            self.status = "solved"
            if not a2plib.SIMULATION_STATE:
                Msg("===== " + translate("A2plus", "System solved using partial + recursive unfixing") + " =====\n")
                self.checkForUnmovedParts()
        else:
            if a2plib.SIMULATION_STATE == True:
                self.status = "unsolved"
                return systemSolved

            else: # a2plib.SIMULATION_STATE == False
                self.status = "unsolved"
                if showFailMessage == True:
                    Msg("===== " + translate("A2plus", "Could not solve system") + " =====\n")
                    msg = \
translate("A2plus",
'''
Constraints inconsistent. Cannot solve System.
Please run the conflict finder tool!
'''
)
                    QtGui.QMessageBox.information(
                        QtGui.QApplication.activeWindow(),
                        translate("A2plus", "Constraint mismatch"),
                        msg
                        )
                return systemSolved

    def solveSystemLoaded(self, doc, showFailMessage=True, syncBefore=True):
        if not a2plib.SIMULATION_STATE:
            Msg("===== " + translate("A2plus", "Start Solving System") + " =====\n")

        if syncBefore:
            tSync0 = time.perf_counter()
            if not self.syncFromDocument(doc):
                self.status = "sessionInvalid"
                return False
            self.timingLoadMs = (time.perf_counter() - tSync0) * 1000.0

        systemSolved = self.solveAccuracyStepsLoaded(doc)
        if systemSolved:
            self.status = "solved"
            if not a2plib.SIMULATION_STATE:
                Msg("===== " + translate("A2plus", "System solved using partial + recursive unfixing") + " =====\n")
                self.checkForUnmovedParts()
        else:
            if a2plib.SIMULATION_STATE:
                self.status = "unsolved"
                return systemSolved

            self.status = "unsolved"
            if showFailMessage:
                Msg("===== " + translate("A2plus", "Could not solve system") + " =====\n")
                msg = \
translate("A2plus",
'''
Constraints inconsistent. Cannot solve System.
Please run the conflict finder tool!
'''
)
                QtGui.QMessageBox.information(
                    QtGui.QApplication.activeWindow(),
                    translate("A2plus", "Constraint mismatch"),
                    msg
                    )
        return systemSolved

    def checkForUnmovedParts(self):
        """
        If there are parts, which are constrained but have no
        constraint path to a fixed part, the solver will
        ignore them and they are not moved.
        This function detects this and signals it to the user.
        """
        if len(self.unmovedParts) != 0:
            FreeCADGui.Selection.clearSelection()
            for obj in self.unmovedParts:
                FreeCADGui.Selection.addSelection(obj)
                msg = translate("A2plus",
'''
The highlighted parts were not moved. They are
not constrained (also over constraint chains)
to a fixed part!
''')
            if a2plib.SHOW_WARNING_FLOATING_PARTS: #dialog is not needet during conflict finding
                QtGui.QMessageBox.information(
                    QtGui.QApplication.activeWindow(),
                    translate("A2plus", "Could not move some parts"),
                    msg
                    )
            else:
                print ('')
                print (msg) # during conflict finding do a print to console output
                print ('')

    def printList(self, name, l):
        Msg("{} = (".format(name))
        for e in l:
            Msg( "{} ".format(e.label) )
        Msg("):\n")

    def calculateChain(self, doc):
        # Initialize step count and work list
        self.stepCount = 0
        workList = []
        workListSet = set()

        if not a2plib.PARTIAL_PROCESSING_ENABLED:
            # Solve complete system at once if partial processing is disabled
            workList = self.rigids
            return self.calculateWorkList(doc, workList)

        # Normal partial solving when partial processing is enabled
        # Load initial worklist with all fixed parts
        workList.extend(rig for rig in self.rigids if rig.fixed)
        workListSet = set(workList)

        if len(workList) == 0:
            workList = self.rigids
            return self.calculateWorkList(doc, workList)

        while True:
            addList = set()
            newRigFound = False
            
            # Check linked rigids for possible additions to the work list
            for rig in workList:
                for linkedRig in rig.linkedRigids:
                    if linkedRig not in workListSet and rig.isFullyConstrainedByRigid(linkedRig):
                        addList.add(linkedRig)
                        newRigFound = True
                        break
            
            if not newRigFound:
                # If no new rigids found, consider candidates for addition to the work list
                for rig in workList:
                    addList.update(rig.getCandidates())

            if addList:
                # Update cached state for rigids being added to the work list
                for rig in addList:
                    rig.updateCachedState(rig.placement)
                workList.extend(addList)
                workListSet.update(addList)
                solutionFound = self.calculateWorkList(doc, workList)
                if not solutionFound:
                    return False
            else:
                break

            if a2plib.SOLVER_ONESTEP > 2:
                break
            
        return True

    def calculateWorkList(self, doc, workList):
        reqPosAccuracy = self.mySOLVER_POS_ACCURACY
        reqSpinAccuracy = self.mySOLVER_SPIN_ACCURACY

        workListSet = set(workList)
        for rig in workList:
            rig.enableDependencies(workList, workListSet)
        for rig in workList:
            rig.calcSpinBasicDataDepsEnabled()

        self.lastPositionError = SOLVER_CONVERGENCY_ERROR_INIT_VALUE
        self.lastAxisError = SOLVER_CONVERGENCY_ERROR_INIT_VALUE
        self.convergencyCounter = 0

        goodAccuracy = False
        while not goodAccuracy:
            # Check for async cancellation
            if self._asyncCancelCheck is not None and self._asyncCancelCheck():
                return False
            
            maxPosError = 0.0
            maxAxisError = 0.0
            maxSingleAxisError = 0.0

            self.stepCount += 1
            self.convergencyCounter += 1
            
            # Emit progress every 100 steps (if async)
            if self._asyncProgressCallback is not None and self.stepCount % 100 == 0:
                self._asyncProgressCallback(self.stepCount, maxPosError)
            
            # First calculate all the movement vectors
            for w in workList:
                w.moved = True
                w.calcMoveData(doc, self)
                if w.maxPosError > maxPosError:
                    maxPosError = w.maxPosError
                if w.maxAxisError > maxAxisError:
                    maxAxisError = w.maxAxisError
                if w.maxSingleAxisError > maxSingleAxisError:
                    maxSingleAxisError = w.maxSingleAxisError

            # Perform the move
            for w in workList:
                w.move(doc)

            # The accuracy is good, apply the solution to FreeCAD's objects
            if (maxPosError <= reqPosAccuracy and   # relevant check
                maxAxisError <= reqSpinAccuracy and # relevant check
                maxSingleAxisError <= reqSpinAccuracy * 10  # additional check for insolvable assemblies
                                                            # sometimes spin can be solved but singleAxis not..
                ) or (a2plib.SOLVER_ONESTEP > 0):
                # The accuracy is good, we're done here
                goodAccuracy = True

                # Mark the rigids as tempfixed and add its constrained rigids to pending list to be processed next
                for r in workList:
                    r.applySolution(doc, self)
                    r.tempfixed = True

            if self.convergencyCounter > self.convergencyCheckSteps:
                if (
                    maxPosError  >= self.convergencyFactor * self.lastPositionError or
                    maxAxisError >= self.convergencyFactor * self.lastAxisError
                    ):
                    foundRigidToUnfix = False
                    # search for unsolved dependencies...
                    for rig in workList:
                        if rig.fixed or rig.tempfixed: continue
                        #if rig.maxAxisError >= maxAxisError or rig.maxPosError >= maxPosError:
                        if rig.maxAxisError > reqSpinAccuracy or rig.maxPosError > reqPosAccuracy:
                            for r in rig.linkedRigids:
                                if r.tempfixed and not r.fixed:
                                    r.tempfixed = False
                                    #Msg("unfixed Rigid {}\n".format(r.label))
                                    foundRigidToUnfix = True

                    if foundRigidToUnfix:
                        self.lastPositionError = SOLVER_CONVERGENCY_ERROR_INIT_VALUE
                        self.lastAxisError = SOLVER_CONVERGENCY_ERROR_INIT_VALUE
                        self.convergencyCounter = 0
                        continue
                    else:
                        Msg('\n')
                        Msg('convergency-conter: {}\n'.format(self.convergencyCounter))
                        Msg(translate("A2plus", "No convergency anymore, retrying") + "\n")
                        pass

                self.lastPositionError = maxPosError
                self.lastAxisError = maxAxisError
                self.maxSingleAxisError = maxSingleAxisError
                self.convergencyCounter = 0

            if self.stepCount > self.maxSteps:
                Msg(translate("A2plus", "Reached max calculations count: {}").format(self.maxSteps) + "\n")
                return False
        return True

    def solutionToParts(self,doc):
        for rig in self.rigids:
            rig.applySolution(doc, self);

#------------------------------------------------------------------------------
_SOLVER_SESSIONS = {}

def _constraintValueToFloat(value):
    if value is None:
        return None
    try:
        return float(value)
    except:
        pass
    try:
        return float(value.Value)
    except:
        return None

class SolverSession(object):
    def __init__(self, doc):
        self.doc = doc
        self.constraintEntryByName = {}
        self.activeConstraints = []
        self.activeConstraintByName = {}
        self.constraintsByObject = {}
        self.solverCache = {}
        # Session-level geometry cache - persists across multiple solves
        # Keys: 'objects', 'pos', 'axis', 'face', 'edge'
        # This dramatically speeds up repeated solves (e.g., during drag)
        self.geometryCache = {}

    def invalidate(self):
        for solver in self.solverCache.values():
            solver.clear()
        self.solverCache = {}
        self.geometryCache = {}  # Clear geometry cache on invalidation

    def _normalizeChangedConstraints(self, changedConstraints):
        if changedConstraints is None:
            return None
        if isinstance(changedConstraints, (list, tuple, set)):
            return [c for c in changedConstraints if c is not None]
        return [changedConstraints]

    def refreshConstraintCacheChanged(self, changedConstraints):
        changedList = self._normalizeChangedConstraints(changedConstraints)
        if changedList is None:
            return False
        if len(changedList) == 0:
            return len(self.activeConstraints) > 0
        if len(self.activeConstraints) == 0:
            return False

        changed = False
        for c in changedList:
            cName = getattr(c, 'Name', None)
            if cName is None:
                return False
            cInDoc = self.doc.getObject(cName)
            if cInDoc is None:
                return False
            c = cInDoc
            if cName not in self.activeConstraintByName:
                return False
            if hasattr(c, 'Suppressed') and c.Suppressed:
                return False

            oldEntry = self.constraintEntryByName.get(cName)
            newEntry = self._constraintSignatureEntry(c)
            if oldEntry is None:
                return False

            if newEntry[:6] != oldEntry[:6]:
                return False

            if newEntry != oldEntry:
                self.constraintEntryByName[cName] = newEntry
                self.activeConstraintByName[cName] = c
                changed = True

        if changed:
            self.invalidate()
        return True

    def _constraintSignatureEntry(self, c):
        obj1 = getattr(c, 'Object1', None)
        obj2 = getattr(c, 'Object2', None)
        return (
            c.Name,
            getattr(c, 'Type', None),
            obj1,
            getattr(c, 'SubElement1', None),
            obj2,
            getattr(c, 'SubElement2', None),
            getattr(c, 'directionConstraint', None),
            _constraintValueToFloat(getattr(c, 'offset', None)),
            _constraintValueToFloat(getattr(c, 'angle', None)),
            bool(getattr(c, 'lockRotation', False)),
            )

    def refreshConstraintCache(self):
        allConstraints = []
        constraintsByObject = {}
        signatureChanged = False
        oldEntries = self.constraintEntryByName
        newEntries = {}

        for obj in self.doc.Objects:
            if 'ConstraintInfo' not in getattr(obj, 'Content', ''):
                continue
            if hasattr(obj, 'Suppressed') and obj.Suppressed:
                continue
            allConstraints.append(obj)
            for attr in ('Object1', 'Object2'):
                objectName = getattr(obj, attr, None)
                if objectName is None:
                    continue
                constraintsByObject.setdefault(objectName, []).append(obj)
            entry = self._constraintSignatureEntry(obj)
            newEntries[obj.Name] = entry
            if oldEntries.get(obj.Name) != entry:
                signatureChanged = True

        if not signatureChanged and len(oldEntries) != len(newEntries):
            signatureChanged = True

        self.activeConstraints = allConstraints
        self.activeConstraintByName = {c.Name: c for c in allConstraints}
        self.constraintsByObject = constraintsByObject
        self.constraintEntryByName = newEntries

        if signatureChanged:
            self.invalidate()

    def getConstraintGraph(self):
        self.refreshConstraintCache()
        return self.activeConstraints, self.constraintsByObject

    def _normalizeMateList(self, matelist):
        if matelist is None:
            return list(self.activeConstraints)

        constraints = []
        names = set()
        for c in matelist:
            if c is None:
                continue
            cName = getattr(c, 'Name', None)
            if cName is None or cName in names:
                continue
            activeConstraint = self.activeConstraintByName.get(cName)
            if activeConstraint is None:
                continue
            names.add(cName)
            constraints.append(activeConstraint)
        return constraints

    def _scopeKey(self, constraints):
        return tuple(sorted(c.Name for c in constraints))

    def getSolver(self, matelist, fastMode, checkFaultyConstraints, changedConstraints=None, skipConstraintRefresh=False):
        if skipConstraintRefresh and len(self.activeConstraints) > 0:
            pass
        elif not self.refreshConstraintCacheChanged(changedConstraints):
            self.refreshConstraintCache()

        scopeConstraints = self._normalizeMateList(matelist)
        scopeKey = self._scopeKey(scopeConstraints)
        cacheKey = (scopeKey, bool(fastMode), bool(checkFaultyConstraints))
        cachedSolver = self.solverCache.get(cacheKey)
        if cachedSolver is not None:
            tSync0 = time.perf_counter()
            if cachedSolver.syncFromDocument(self.doc):
                cachedSolver.timingLoadMs = (time.perf_counter() - tSync0) * 1000.0
                return cachedSolver, True
            self.solverCache.pop(cacheKey, None)

        solver = SolverSystem()
        solver.configureMode(fastMode)
        tLoad0 = time.perf_counter()
        solver.loadSystem(
            self.doc,
            scopeConstraints,
            checkFaultyConstraints=checkFaultyConstraints,
            buildDOFInfo=False,
            geometryCache=self.geometryCache  # Pass session-level geometry cache
            )
        solver.timingLoadMs = (time.perf_counter() - tLoad0) * 1000.0
        if solver.status != "loadingDependencyError":
            self.solverCache[cacheKey] = solver
        return solver, False

def _getSolverSession(doc):
    if doc is None:
        return None
    key = doc.Name
    session = _SOLVER_SESSIONS.get(key)
    if session is None or session.doc is not doc:
        session = SolverSession(doc)
        _SOLVER_SESSIONS[key] = session
    return session

def clearSolverSession(doc=None):
    if doc is None:
        for session in _SOLVER_SESSIONS.values():
            session.invalidate()
        _SOLVER_SESSIONS.clear()
        return
    key = doc.Name
    session = _SOLVER_SESSIONS.pop(key, None)
    if session is not None:
        session.invalidate()

def _sessionOptionsFromCache(cache):
    changedConstraints = None
    skipConstraintRefresh = False
    if isinstance(cache, dict):
        changedConstraints = cache.get('changedConstraints', None)
        skipConstraintRefresh = bool(cache.get('skipConstraintRefresh', False))
    return changedConstraints, skipConstraintRefresh

def _solveWithSession(
    session,
    doc,
    matelist,
    fastMode,
    showFailMessage,
    checkFaultyConstraints,
    changedConstraints=None,
    skipConstraintRefresh=False
    ):
    solver, isSynced = session.getSolver(
        matelist,
        fastMode,
        checkFaultyConstraints,
        changedConstraints=changedConstraints,
        skipConstraintRefresh=skipConstraintRefresh
        )
    if solver.status == "loadingDependencyError":
        return False, solver
    return solver.solveSystemLoaded(doc, showFailMessage=showFailMessage, syncBefore=not isSynced), solver

def solveConstraints(
    doc,
    cache=None,
    useTransaction=True,
    matelist=None,
    showFailMessage=True,
    compatibilityFallback=True
    ):

    if doc is None:
        QtGui.QMessageBox.information(
                    QtGui.QApplication.activeWindow(),
                    translate("A2plus", "No active document found!"),
                    translate("A2plus", "Before running solver, you have to open an assembly file.")
                    )
        return

    if useTransaction:
        doc.openTransaction("a2p_systemSolving")
    t0 = time.perf_counter()

    changedConstraints, skipConstraintRefresh = _sessionOptionsFromCache(cache)

    session = _getSolverSession(doc)
    systemSolved, usedSolver = _solveWithSession(
        session,
        doc,
        matelist,
        fastMode=True,
        showFailMessage=False,
        checkFaultyConstraints=False,
        changedConstraints=changedConstraints,
        skipConstraintRefresh=skipConstraintRefresh
        )

    if compatibilityFallback and not systemSolved and not a2plib.SIMULATION_STATE:
        Msg(translate("A2plus", "Aggressive solve did not converge, retrying in compatibility mode") + "\n")
        systemSolved, usedSolver = _solveWithSession(
            session,
            doc,
            matelist,
            fastMode=False,
            showFailMessage=showFailMessage,
            checkFaultyConstraints=True,
            changedConstraints=changedConstraints,
            skipConstraintRefresh=skipConstraintRefresh
            )

    if useTransaction:
        doc.commitTransaction()

    touchedObjects = [doc.getObject(name) for name in usedSolver.touchedObjectNames]
    a2plib.unTouchA2pObjects(touchedObjects)

    if not a2plib.SIMULATION_STATE:
        dt = (time.perf_counter() - t0) * 1000.0
        Msg(translate("A2plus", "Solver runtime: {:.1f} ms").format(dt) + "\n")
        Msg(translate("A2plus", "  load: {:.1f} ms, chain: {:.1f} ms").format(
            usedSolver.timingLoadMs,
            usedSolver.timingChainMs
            ) + "\n")
    return systemSolved

def _collectActiveConstraints(doc):
    session = _getSolverSession(doc)
    if session is None:
        return [], {}
    return session.getConstraintGraph()

def _walkConstraintComponent(allConstraints, constraintsByObject, seedObjects):
    if len(allConstraints) == 0 or len(seedObjects) == 0:
        return None

    visitedObjects = set()
    visitedConstraints = set()
    pendingObjects = list(seedObjects)

    while len(pendingObjects) > 0:
        objectName = pendingObjects.pop()
        if objectName in visitedObjects:
            continue
        visitedObjects.add(objectName)
        for c in constraintsByObject.get(objectName, []):
            if c.Name in visitedConstraints:
                continue
            visitedConstraints.add(c.Name)
            obj1 = getattr(c, 'Object1', None)
            obj2 = getattr(c, 'Object2', None)
            if obj1 is not None and obj1 not in visitedObjects:
                pendingObjects.append(obj1)
            if obj2 is not None and obj2 not in visitedObjects:
                pendingObjects.append(obj2)

    if len(visitedConstraints) == 0:
        return None

    return [c for c in allConstraints if c.Name in visitedConstraints]

def getConstraintComponent(doc, changedConstraints):
    if doc is None or changedConstraints is None:
        return None

    if isinstance(changedConstraints, (list, tuple, set)):
        seedConstraints = list(changedConstraints)
    else:
        seedConstraints = [changedConstraints]

    allConstraints, constraintsByObject = _collectActiveConstraints(doc)

    seedObjects = []
    for c in seedConstraints:
        if c is None:
            continue
        if 'ConstraintInfo' not in getattr(c, 'Content', ''):
            continue
        for attr in ('Object1', 'Object2'):
            objectName = getattr(c, attr, None)
            if objectName is not None:
                seedObjects.append(objectName)

    if len(seedObjects) == 0:
        return None

    return _walkConstraintComponent(allConstraints, constraintsByObject, seedObjects)

def getConstraintComponentByObjects(doc, changedObjects):
    if doc is None or changedObjects is None:
        return None

    if isinstance(changedObjects, (list, tuple, set)):
        seedObjects = [name for name in changedObjects if name is not None]
    else:
        seedObjects = [changedObjects]

    if len(seedObjects) == 0:
        return None

    allConstraints, constraintsByObject = _collectActiveConstraints(doc)
    return _walkConstraintComponent(allConstraints, constraintsByObject, seedObjects)

def solveConstraintsIncremental(
    doc,
    changedConstraints,
    cache=None,
    useTransaction=True,
    showFailMessage=True,
    compatibilityFallback=True
    ):
    matelist = getConstraintComponent(doc, changedConstraints)
    if matelist is None:
        return solveConstraints(
            doc,
            cache=cache,
            useTransaction=useTransaction,
            showFailMessage=showFailMessage,
            compatibilityFallback=compatibilityFallback
            )

    if not a2plib.SIMULATION_STATE:
        touchedObjects = set()
        for c in matelist:
            obj1 = getattr(c, 'Object1', None)
            obj2 = getattr(c, 'Object2', None)
            if obj1 is not None:
                touchedObjects.add(obj1)
            if obj2 is not None:
                touchedObjects.add(obj2)
        Msg(translate("A2plus", "Incremental solve: {} constraints, {} objects").format(
            len(matelist),
            len(touchedObjects)
            ) + "\n")

    solved = solveConstraints(
        doc,
        cache=cache,
        useTransaction=useTransaction,
        matelist=matelist,
        showFailMessage=False,
        compatibilityFallback=compatibilityFallback
        )

    if solved:
        return True

    if not compatibilityFallback:
        return False

    if not a2plib.SIMULATION_STATE:
        Msg(translate("A2plus", "Incremental solve failed, retrying full solve") + "\n")

    return solveConstraints(
        doc,
        cache=cache,
        useTransaction=useTransaction,
        matelist=None,
        showFailMessage=showFailMessage,
        compatibilityFallback=compatibilityFallback
        )

def autoSolveConstraints( doc, callingFuncName, cache=None, useTransaction=True, matelist=None):
    if not a2plib.getAutoSolveState():
        return
    if callingFuncName is not None:
        """
        print (
            translate("A2plus", "AutoSolveConstraints called from '{}'").format(
                callingFuncName
                )
               )
        """
    solveConstraints(doc, cache=cache, useTransaction=useTransaction, matelist=matelist)

class a2p_SolverCommand:
    def Activated(self):
        solveConstraints( FreeCAD.ActiveDocument ) #the new iterative solver

    def GetResources(self):
        return {
            'Pixmap'  : path_a2p + '/icons/a2p_Solver.svg',
            'MenuText': translate("A2plus", "Solve constraints"),
            'ToolTip' : translate("A2plus", "Solves constraints")
            }

FreeCADGui.addCommand('a2p_SolverCommand', a2p_SolverCommand())
#------------------------------------------------------------------------------

if __name__ == "__main__":
    DebugMsg(A2P_DEBUG_1, translate("A2plus", "Starting solveConstraints latest script...") + "\n")
    doc = FreeCAD.activeDocument()
    solveConstraints(doc)
