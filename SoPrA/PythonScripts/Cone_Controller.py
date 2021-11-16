#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import Sofa

import datetime
import os

#import TransformationFunctions as TF
#import EvaluationConstants as EC

def purgeSerial(SerialObj):
    Finished = False
    print('Purging serial ...')		
    while Finished != True:
        ReceivedText = ''
        ReceivedText = SerialObj.readline()
        print(ReceivedText)        
        if (ReceivedText == ''):
            Finished = True
            print('Purging serial ... Finished!')

def unwrap(InputList):
    OutputList = []
    for i in range(0,len(InputList)):
        OutputList.append(InputList[i][0])
    return OutputList

class Controller(Sofa.PythonScriptController):   

    def initGraph(self, node):
        pass
        self.node = node
        self.Counter = 0
        self.IterationCounter = 0
        self.DistributionStride = 5        
        self.node = node                
        self.RootNode = node.getRoot()        
        
        #self.ConnectToOptiTrack = self.RootNode.getObject('Connect')
        
        self.updateBasePoints = True
        self.PersistentBasePoints = np.array([0,0,0])
        
        self.ModelNode = self.RootNode.getChild('model')
        self.VolumeEffector1 = self.ModelNode.getChild('Cavity01').getObject('VolumeEffector')
        self.VolumeEffector2 = self.ModelNode.getChild('Cavity02').getObject('VolumeEffector')
        self.VolumeEffector3 = self.ModelNode.getChild('Cavity03').getObject('VolumeEffector')
        self.VolumeEffector4 = self.ModelNode.getChild('Cavity04').getObject('VolumeEffector')
        self.FPA1 = self.ModelNode.getChild('ForcePointActuators').getObject('FPA1')                
        self.FPA2 = self.ModelNode.getChild('ForcePointActuators').getObject('FPA2')                
        #self.Cable = self.ModelNode.getChild('cables').getChild('cable1').getObject('head')        
        self.FPAsMOs = self.ModelNode.getChild('ForcePointActuators').getObject('FPAsMOs')
        
    
        self.TetrasMO = self.ModelNode.getObject('tetras')       
    
        self.SamplingPositions = [0,-3]
        self.NSamplingIntervals = 0 # how many intervals there will be in the sampling (there will be N+1 points!)                 
        
        self.ForcePositionAndMagnitude = self.SamplingPositions[0] 
        self.OutputArray = []
        self.OutputIndentations = []
        self.VolumesCavity1 = np.array([])
        self.VolumesCavity2 = np.array([])        
        self.DesiredVolume1 = 0
        self.DesiredVolume2 = 0        
        self.LiveVolumeDataPath = '/home/stefan/VolumeData.txt'
        self.LivePressureDataPath = '/home/stefan/PressureData.txt'
        self.LiveFingerApproachDirectionsPath = '/home/stefan/FingerApproachDirections.txt'
        self.TeachingPath = '/home/stefan/Teaching.txt'
        self.TeachingCableDisplacements = np.array([0,0,0])
        self.TeachingCableDisplacements = self.TeachingCableDisplacements[np.newaxis,:] # make array 2D
                
        self.PressureSensing = True
        self.OldCableDisplacement = np.array([0,0,0])
        self.Point0Array = np.empty((0,3))
        self.Point3Array = np.empty((0,3))
        
        # Stuff for data collection         
        self.NStabilizingFrames = 7
        self.StabilizingFrameCount = 0
        self.PoseCounter = 0
        self.StartSampling = False
        #self.EvalSet = EC.RepeatabilityHigh_R 
        #self.EvalSetName = self.EvalSet[0]
        #self.EvalPointsList = self.EvalSet[1]
        #print('EvalPointsList:')
        #print(self.EvalSetName)
        #print(self.EvalPointsList)
        self.DoTracking = False
        
        self.ExperimentDirName = 'Repeatability2/'
        self.ResultsList = np.empty((0,15),float)
        
    def onBeginAnimationStep(self,deltaTime):                

        
        if self.PressureSensing:
            self.Data = np.loadtxt(self.LivePressureDataPath)
            self.RealPressures = self.Data[:4] # in kPa
            self.States = self.Data[4:]
            print('Pressures: ' +  str(self.RealPressures))
            print('States: ' + str(self.States))
            
            self.FingerApproachDirections = np.loadtxt(self.LiveFingerApproachDirectionsPath)
            print('FingerApproachDirections: ' +  str(self.FingerApproachDirections))
            
            #CableDisplacement = self.Cable.cableInitialLength - self.Cable.cableLength
            #print('CableDisplacement: ' + str(CableDisplacement))
            Point0Coords = self.FPAsMOs.position[0]
            print('Point0Coords: ' + str(Point0Coords))
            Point3Coords = self.FPAsMOs.position[3]
            print('Point3Coords: ' + str(Point3Coords))
            
            self.Point0Array = np.append(self.Point0Array, [Point0Coords],0)
            self.Point3Array = np.append(self.Point3Array, [Point3Coords],0)
            
            self.setFPAs(self.FingerApproachDirections)

            
            self.InitialVolume1 = self.VolumeEffector1.getData('initialCavityVolume').value           
            self.InitialVolume2 = self.VolumeEffector2.getData('initialCavityVolume').value
            self.InitialVolume3 = self.VolumeEffector3.getData('initialCavityVolume').value           
            self.InitialVolume4 = self.VolumeEffector4.getData('initialCavityVolume').value
            
            
            #Factor =  1.515#0.1
            Factor = 1.7
            #Factor = 17
            AtmPressure = 101 #kPa
            # P*V is constant, Ci are theses constants as found by the initial volume. A factor is introduced to account for tubing and volume inside the pressure sensor
            C1 = self.InitialVolume1 * AtmPressure 
            C2 = self.InitialVolume2 * AtmPressure              
            C3 = self.InitialVolume3 * AtmPressure    
            C4 = self.InitialVolume4 * AtmPressure    
           
            RealVolumeChange1 = (self.InitialVolume1 - C1/(AtmPressure + self.RealPressures[0])) * Factor
            RealVolumeChange2 = (self.InitialVolume2 - C2/(AtmPressure + self.RealPressures[1])) * Factor
            RealVolumeChange3 = (self.InitialVolume3 - C3/(AtmPressure + self.RealPressures[2])) * Factor
            RealVolumeChange4 = (self.InitialVolume4 - C4/(AtmPressure + self.RealPressures[3])) * Factor
            #RealVolumeChange3 = (self.InitialVolume3 - C3/(AtmPressure + self.RealPressures[2])) * Factor
            
            self.RealVolumeChanges = [RealVolumeChange1, RealVolumeChange2, RealVolumeChange3, RealVolumeChange4]
            self.setDesiredVolumeChanges(self.RealVolumeChanges) 
        else:
            self.Data = np.loadtxt(self.LiveVolumeDataPath)
            CorrectiveFactor = 4
            self.RealVolumes = self.Data[:3]*1000000 * CorrectiveFactor# in micro L
            self.setDesiredVolumeChanges(self.RealVolumes) 
            print(self.RealVolumes)              
                            
        #------------------------------
        # Calculate transforms and frames
        #------------------------------
        
        # Get points from OptiTrack and scale them (m --> mm)                                        
#        if self.DoTracking:
#        
#            self.TrackedPointsRaw.position = TF.scaleTrackingPoints(self.ConnectToOptiTrack.BarycenterMarkers)       
#            
#            if not self.updateBasePoints: # Since base markers get easily blocked when the robot moves, we can keep a previous position. This will work as long as the setup as a whole doesn't move
#                #print('TrackedPintsRaw: ' + str(self.TrackedPointsRaw.position))
#                NPTrackedPointsRaw = np.array(self.TrackedPointsRaw.position)
#                NewArray = np.append(self.PersistentBasePoints, NPTrackedPointsRaw[3:],0)
#                #print('NewArray: ' + str(NewArray))
#                self.TrackedPointsRaw.position = NewArray.tolist()
#            
#            # Get Tracking frame, which is defined by the markers on the base of the platform. The robot is placed in the tracking setup such, that one of the edges of the triangle 
#            # representing the base tracking markers is approx. parallel to the OptiTrack frame. See the Optitrack GUI for confirmation
#            R_Tracking = TF.calcTrackingBaseFrame(self.TrackedPointsRaw.position[:3])
#            Quat = R_Tracking.as_quat()
#            BarycenterBase = np.mean(self.TrackedPointsRaw.position[:3],0)
#            self.BaseFrameTracking.position = [BarycenterBase[0],BarycenterBase[1],BarycenterBase[2], Quat[0],Quat[1],Quat[2],Quat[3]] 
#            # Knowing the orientation and translation from the tracking points with respect to the robot's base we transform the point into SOFA coords.                        
#            TransformedPoints = TF.calcRotationTrackingToBase(np.array(self.TrackedPointsRaw.position) - BarycenterBase, R_Tracking) + np.array([0,0,TF.BaseMarkerHeight])
#            #print('TransformedPoints: ' + str(TransformedPoints))
#            self.TrackedPointsInBase.position = TransformedPoints.tolist()
#            
#            # Now look dedicatedly at the tip points
#            TipPoints = TransformedPoints[3:]
#            #print('TipPoints: ' + str(TipPoints))        
#            T_TipFrame, R_TipFrame, T_TipBarycenter, R_TipBarycenter = TF.calcTrackingTipFrame(TipPoints)
#            
#            Quat2 = R_TipBarycenter.as_quat()
#            self.TrackedTipBarycenter.position = [T_TipBarycenter[0], T_TipBarycenter[1], T_TipBarycenter[2], Quat2[0], Quat2[1], Quat2[2], Quat2[3]]
#            #self.TrackedTipBarycenter.position = [0,0,0, Quat2[0], Quat2[1], Quat2[2], Quat2[3]]
#            Quat3 = R_TipFrame.as_quat()
#            #self.TrackedT_TipFrame.position = [T_TipFrame[0], T_TipFrame[1], T_TipFrame[2], 0, 0, 0, 1]
#            self.TrackedTipFrame.position = [T_TipFrame[0], T_TipFrame[1], T_TipFrame[2], Quat3[0], Quat3[1], Quat3[2], Quat3[3]]
#            
#            #print('TipModel.position: ' + str(self.ModelTipFrame.position))
#            R_ModelTip = R.from_quat(self.ModelTipFrame.position[0][3:])
#            
#            #------------------------------
#            # Calculate difference between model and tracking
#            #------------------------------
#    
#            TF.calcErrorMetrics(self.TrackedTipFrame.position[0], self.ModelTipFrame.position[0])       
#            
                
    
    def setDesiredVolumeChanges(self, Volumes): 
    
        #self.ForceBM.reinit()       
        
        #Volumes[0] = Volumes[0]*0.794718462  #using a correction factor here
        print('Desired volume changes (V1,V2): ' + str(Volumes))
        self.VolumeEffector1.getData('desiredVolume').value = self.VolumeEffector1.getData('initialCavityVolume').value - Volumes[0]
        self.VolumeEffector2.getData('desiredVolume').value = self.VolumeEffector2.getData('initialCavityVolume').value - Volumes[1]
        self.VolumeEffector3.getData('desiredVolume').value = self.VolumeEffector3.getData('initialCavityVolume').value - Volumes[2]
        self.VolumeEffector4.getData('desiredVolume').value = self.VolumeEffector4.getData('initialCavityVolume').value - Volumes[3]
        
    def setFPAs(self, FingerApproachDirections):
        
        print('FingerApproachDirections setFPAs: ' + str(FingerApproachDirections))
        if FingerApproachDirections.size == 1:
            self.setFPA(FingerApproachDirections, self.FPA1)        
            self.FPA2.indices = [] 
        else:
            self.setFPA(FingerApproachDirections[0], self.FPA1)        
            self.setFPA(FingerApproachDirections[1], self.FPA2)
    
    
    def setFPA(self, FingerApproachDirection, FPA):
        if FingerApproachDirection == -1: # push on a non-movable point!
            self.FPA1.indices = [3]
            #FPA.direction = [0,0,1]
            self.FPA2.indices = [] 
            return
        else:            
            if FingerApproachDirection == 0:
                FPA.indices = [0]
                FPA.direction = [0,0,1]
            elif FingerApproachDirection == 1:
                FPA.indices = [1]
                FPA.direction = [0,1,0]
            elif FingerApproachDirection == 2:
                FPA.indices = [2]
                FPA.direction = [0,0,-1]
            if FingerApproachDirection == 3:
                FPA.indices = [3]
                FPA.direction = [0,0,1]
            elif FingerApproachDirection == 4:
                FPA.indices = [4]
                FPA.direction = [0,1,0]
            elif FingerApproachDirection == 5:
                FPA.indices = [5]
                FPA.direction = [0,0,-1]
            
    
            
    def onEndAnimationStep(self,deltaTime):        
        
        if self.StartSampling == True:
            
            if self.StabilizingFrameCount >= self.NStabilizingFrames:                            
                
                # Trigger the change to the new state, i.e. set the next displacements in simulation and in reality             
                if self.PoseCounter < len(self.EvalPointsList):                                    
                    
                                        
                    #  First Collect the data from the current stable state                
                    DiffTranslation, NormDiffTranslation, DiffAngleDeg, InclinationAngle = TF.calcErrorMetrics(self.TrackedTipFrame.position[0], self.ModelTipFrame.position[0], Print=True)                                        
                    
                    # Get estimated cable lengths
                    Cable1 = self.ModelNode.cables.cable1.head        
                    Cable2 = self.ModelNode.cables.cable2.head
                    Cable3 = self.ModelNode.cables.cable3.head
                
                    self.CableDisplacements = np.array([Cable1.displacement, Cable2.displacement, Cable3.displacement])
        
                    CurrentPose = self.EvalPointsList[self.PoseCounter]                        
                                        
                    self.ResultsList = np.append(self.ResultsList, [[NormDiffTranslation, 
                                                                     DiffAngleDeg,                                                                      
                                                                     DiffTranslation[0], 
                                                                     DiffTranslation[1],
                                                                     DiffTranslation[2],                                                                     
                                                                     CurrentPose[0], 
                                                                     CurrentPose[1],
                                                                     CurrentPose[2],
                                                                     self.CableDisplacements[0],
                                                                     self.CableDisplacements[1],
                                                                     self.CableDisplacements[2],
                                                                     self.RealPressures[0],
                                                                     self.RealPressures[1],
                                                                     self.RealPressures[2],
                                                                     InclinationAngle]],0)                    
                    # Prepare the next step
                    self.PoseCounter = self.PoseCounter + 1
                    
                    NewPose = None
                    
                    if self.PoseCounter >= len(self.EvalPointsList):               
                        NewPose = self.EvalPointsList[-1]                                               
                    else:
                        NewPose = self.EvalPointsList[self.PoseCounter]
                    
                    
                    print('New displacements: ' + str(-NewPose))                                        
                    
                else:                    
                    t = datetime.datetime.now()
                    DateTimeStr = t.strftime('%Y-%m-%d_%H:%M:%S')                
                    Path='/home/stefan/Repos/PneumaticSensing/Fetch/Data/Inverse/' + self.ExperimentDirName
                    
                    if not os.path.exists(Path):
                        os.makedirs(Path)
                        
                    print(self.ResultsList)                    
                    np.savetxt(Path + DateTimeStr + '_' + self.EvalSetName + '.txt', self.ResultsList)
                    # Reset everything
                    self.StartSampling = False
                    self.PoseCounter = 0 
                    self.ResultsList = np.empty((0,9),float)
                    print('Warning: reached limits of sampling poseses array')                    
                    
                self.StabilizingFrameCount = 0
                
                
            else:                          
                self.StabilizingFrameCount = self.StabilizingFrameCount + 1
        
        else:
            pass # do nothing
        
        
    def fromSensorCoordsToMM(self, SensorCoords):
        # sensor coordinates range from [0,0] to [1,1]
        DistanceBetweenSensors = 5#mm
        AbsoluteOffsetFromCenter = 5/float(2) # the corners are 18.5 mm away from each axis
        Offset = [-AbsoluteOffsetFromCenter,-AbsoluteOffsetFromCenter]
        CoordsInMM = [Offset[0] + SensorCoords[0]*DistanceBetweenSensors, Offset[1] + SensorCoords[1]*DistanceBetweenSensors]
        return CoordsInMM

    def onKeyPressed(self,c):                    
        
        if (c == "1"):            
            print('Saving Point0Array!')
            np.savetxt('/home/stefan/Point0Array.txt', self.Point0Array)            
            pass
        
        if (c == "2"):            
            print('Saving Point1Array!')
            np.savetxt('/home/stefan/Point3Array.txt', self.Point3Array)
        
        elif (c == "0"):
            pass
            #self.MotorController.driveLinear([0,0,0])
            
        if (c == "+"):
            pass
            #self.MotorController.driveLinear([5,5,5])
            
    
#        self.InitialVolume1 = self.VolumeEffector1.getData('initialCavityVolume').value           
#        self.InitialVolume2 = self.VolumeEffector2.getData('initialCavityVolume').value
#        self.InitialVolume3 = self.VolumeEffector3.getData('initialCavityVolume').value
#        VolumeIncrement = 10
#        
#        if (c == "1"):
#           self.DesiredVolume1 = self.DesiredVolume1 + VolumeIncrement           
#           self.VolumeEffector1.getData('desiredVolume').value = self.InitialVolume1 - self.DesiredVolume1                      
#           
#        elif (c == "2"):
#           self.DesiredVolume1 = self.DesiredVolume1 - VolumeIncrement
#           if(self.DesiredVolume1 < 0):
#               self.desiredVolume1 = 0           
#        
#           self.VolumeEffector1.getData('desiredVolume').value = self.InitialVolume1 - self.DesiredVolume1
#           
#           
#        if (c == "4"):
#           self.DesiredVolume2 = self.DesiredVolume2 + VolumeIncrement           
#           self.VolumeEffector2.getData('desiredVolume').value = self.InitialVolume2 - self.DesiredVolume2           
#           
#           
#        elif (c == "5"):
#           self.DesiredVolume2 = self.DesiredVolume2 - VolumeIncrement
#           if(self.DesiredVolume2 < 0):
#               self.desiredVolume2 = 0           
#        
#           self.VolumeEffector2.getData('desiredVolume').value = self.InitialVolume2 - self.DesiredVolume2
#        
#        if (c == "7"):
#           self.DesiredVolume3 = self.DesiredVolume3 + VolumeIncrement           
#           self.VolumeEffector3.getData('desiredVolume').value = self.InitialVolume3 - self.DesiredVolume3
#           
#           
#        elif (c == "8"):
#           self.DesiredVolume3 = self.DesiredVolume3 - VolumeIncrement
#           if(self.DesiredVolume3 < 0):
#               self.desiredVolume3 = 0           
#        
#           self.VolumeEffector3.getData('desiredVolume').value = self.InitialVolume3 - self.DesiredVolume3
#           
#           
#        elif (c == "+"):
#            fig = plt.figure()
#            ax = fig.add_subplot(111)
#            print(self.VolumesCavity1)
#            print(self.VolumesCavity2)
#            print(self.VolumesCavity3)
#            ax.scatter(range(0,len(self.VolumesCavity1)),self.VolumesCavity1, c='r')
#            ax.scatter(range(0,len(self.VolumesCavity2)),self.VolumesCavity2, c='g')
#            ax.scatter(range(0,len(self.VolumesCavity3)),self.VolumesCavity3, c='b')
#            plt.show()
#            
#        self.VolumesCavity1 = np.append(self.VolumesCavity1, self.VolumeEffector1.getData('volumeGrowth').value)
#        self.VolumesCavity2 = np.append(self.VolumesCavity2, self.VolumeEffector2.getData('volumeGrowth').value)
#        self.VolumesCavity3 = np.append(self.VolumesCavity3, self.VolumeEffector3.getData('volumeGrowth').value)
#           
            
#        elif (c == "-"):
#            fig = plt.figure()
#            ax = fig.add_subplot(111)
#            print(self.VolumesCavity2)
#            ax.scatter(range(0,len(self.VolumesCavity2)),self.VolumesCavity2)
#            plt.show()




    
    
            
    
            
