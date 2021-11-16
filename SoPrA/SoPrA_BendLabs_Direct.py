import Sofa
import os
import numpy as np
#from stlib.physics.deformable import ElasticMaterialObject
#from stlib.physics.constraints import FixedBox
#from stlib.physics.collision import CollisionMesh


#from stlib.visuals import VisualModel

#from softrobots.actuators import VolumeEffector
#from softrobots.sensors  import PneumaticSensor
path = os.path.dirname(os.path.abspath(__file__))+'/Meshes/'
MeshesPath = os.path.dirname(os.path.abspath(__file__))+'/Geometries/'
GeneratedMeshesPath = os.path.dirname(os.path.abspath(__file__))+'/Geometries/'

from scipy.spatial.transform import Rotation as R
#TempPath = os.path.dirname(os.path.abspath(__file__))+'


import Constants as Const
import rigidification

def fromBendLabsToSpherical(alpha, beta):
    # theta is the angle from the vertical (z-axis)
    # phi is the angle in the xy-plane
    abs_alpha = np.abs(alpha)
    abs_beta = np.abs(beta)
    sgn_alpha = np.sign(alpha)
    sgn_beta = np.sign(beta)
    theta = np.sqrt(alpha**2+beta**2) # the angle is kind of the hypothenuse of a triangle where the sides lengths are both angles
    phi = abs_beta/(abs_alpha+abs_beta)*90 # if beta==0, we're in the xz plane, if beta==1 where in the yz plane
    Offset = 0
    if sgn_alpha==-1:
        if sgn_beta==-1:
            Offset = 180
            phi = phi+Offset
        else:
            Offset = 180
            phi = Offset-phi
    elif sgn_beta==-1:
        Offset = 360
        phi = 360-phi
    
    return theta, phi

def calcAlphaAndBeta(RotationAngle, Height, RInXY):
    x = RInXY * np.cos(np.deg2rad(RotationAngle))
    y = RInXY * np.sin(np.deg2rad(RotationAngle))
    print("x, y: " + str(x) + ", " + str(y) )
    alpha = np.rad2deg(np.arctan2(x,Height))
    beta = np.rad2deg(np.arctan2(y,Height))
    return (alpha,beta)

class OrientationSweepController(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)

        #self.BendLabsEffector= kwargs["BendLabsEffector"]
        self.SPCNode = kwargs["SPCNode"]
        self.alpha = 0
        self.beta = 0
        self.InclinationAngle = 0
        self.RInXY = np.cos(np.deg2rad(90-self.InclinationAngle))
        self.Height = np.cos(np.deg2rad(self.InclinationAngle))
        self.RotationAngle = 0
        
        #self.LiveAngleDataPath = TempPath + "AngleData.txt"
        self.Data = np.array([])
        self.LastData = self.Data
    
    def onKeypressedEvent(self, EventData):
        Key = EventData['key']
        if Key == '+':
  
            CurrentPressure = np.array(self.SPCNode.Cavity01.SPC1.minPressure.value)            
            CurrentPressure = CurrentPressure + 5
            self.SPCNode.Cavity01.SPC1.minPressure.value = CurrentPressure.tolist()
        if Key == '-':
            CurrentPressure = np.array(self.SPCNode.Cavity01.SPC1.minPressure.value)
            CurrentPressure = CurrentPressure - 5
            self.SPCNode.Cavity01.SPC1.minPressure.value = CurrentPressure.tolist()
        print(EventData)
        
    def onAnimateBeginEvent(self, dt):
        pass
        

##        self.RotationAngle = (self.RotationAngle + 0.5)%360
##        
##        
##        
##        Amplitude = 45
##        self.InclinationAngle = Amplitude #* np.abs(np.sin(np.deg2rad(2*self.RotationAngle)))
##        self.RInXY = np.cos(np.deg2rad(90-self.InclinationAngle))
##        self.Height = np.cos(np.deg2rad(self.InclinationAngle))
##        
##        (alpha, beta) = calcAlphaAndBeta(self.RotationAngle, self.Height, self.RInXY)
#               
#        try:
#            self.Data = np.loadtxt(self.LiveAngleDataPath)
#            self.RealPressures = self.Data[:2] # in kPa            
#        except Exception as e:
#            print("Warning: couldn't read pressures from file")
##            self.Data = self.LastData
##            self.RealPressures = self.Data[:6] # in kPa   
#            return
#        
#        if self.Data.shape[0] == 0:
#            return
#        
#        alpha = self.Data[1]
#        beta = self.Data[0]
##        print("setting alpha to: " + str(alpha))
##        print("setting beta to: " + str(beta))
##        self.BendLabsEffector.alpha.value = alpha
##        self.BendLabsEffector.beta.value = beta
#        
#        #        self.BendLabsEffector.alpha.value = alpha
##        self.BendLabsEffector.beta.value = beta
##        
#        theta, phi = fromBendLabsToSpherical(alpha, beta)
#        #theta = alpha
#        #phi = beta
#        self.BendLabsEffector.theta.value = theta
#        self.BendLabsEffector.phi.value = phi
#        
#        
#        print("theta: " + str(theta))       
#        print("phi: " +str(phi))
   
def createScene(rootNode):

                rootNode.addObject('RequiredPlugin', pluginName='SofaPython3 SoftRobots')
                rootNode.addObject('VisualStyle', displayFlags='hideWireframe showBehaviorModels hideCollisionModels hideBoundingCollisionModels showForceFields showInteractionForceFields')

                rootNode.findData('gravity').value = [0, 0, -9810] #
                #rootNode.findData('gravity').value = [0, 0, -1000] #
                rootNode.findData('dt').value = 0.02

                rootNode.addObject('FreeMotionAnimationLoop')
                #rootNode.addObject('QPInverseProblemSolver', printLog='1', epsilon="1e-1", maxIterations="1000", tolerance="1e-5")
                #rootNode.addObject('QPInverseProblemSolver', printLog='1', epsilon="1", maxIterations="1000", tolerance="1e-5")

                rootNode.addObject('GenericConstraintSolver', tolerance="1e-12", maxIterations="10000")

                #rootNode.addObject('GenericConstraintSolver', tolerance="1e-12", maxIterations="10000")

                #rootNode.addObject('CollisionPipeline', verbose="0")
                #rootNode.addObject('BruteForceDetection', name="N2")
                #rootNode.addObject('CollisionResponse', response="FrictionContact", responseParams="mu=0")
                #rootNode.addObject('LocalMinDistance', name="Proximity", alarmDistance="5", contactDistance="1")

                #rootNode.addObject('BackgroundSetting', color='0 0.168627 0.211765')
                rootNode.addObject('BackgroundSetting', color='0.85 0.85 0.85')
                
                
                rootNode.addObject('LightManager')
                rootNode.addObject('PositionalLight', name="light1", color="0.8 0.8 0.8", position="0 60 50")                
                rootNode.addObject('PositionalLight', name="light2", color="0.8 0.8 0.8", position="0 -60 -50")                
                #rootNode.addObject('SpotLight', name="light2", color="1 1 1", position="0 40 100", direction="0 0 -1", cutoff="30", exponent="1")
                

                #VolumetricMeshPath = GeneratedMeshesPath + 'Cone_Volumetric.vtk'
                VolumetricMeshPath = GeneratedMeshesPath + 'Segment1_Volumetric.vtk'
                                
                      
                #SurfaceMeshPath = GeneratedMeshesPath + 'Cone_Surface.stl'
                SurfaceMeshPath = GeneratedMeshesPath + 'Segment1_Surface.stl'
                
                
                
                  #                
                #----------------------
                # Rigidification - start
                #----------------------            
                
                completeMesh = rootNode.addChild('completeMesh')
                
                #completeMesh.addObject('RegularGrid',name='hexaGrid', nx="3", ny="3", nz="9", xmin="0", xmax="3", ymin="0", ymax="3", zmin="0", zmax="19")
                completeMesh.addObject('MeshVTKLoader', name='loader', filename=VolumetricMeshPath)
                completeMesh.addObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
                completeMesh.init()
                MeshTetra = completeMesh.addObject('Mesh', name="AllMesh", src='@loader')
                boxTip = completeMesh.addObject('BoxROI', name='Tip', box=Const.TipBoxCoords, drawBoxes=True,tetrahedra="@AllMesh.tetrahedra" , position="@AllMesh.position")
                boxBody = completeMesh.addObject('BoxROI', name='MainBody', box=Const.MainBodyBoxCoords, drawBoxes=True, tetrahedra="@AllMesh.tetrahedra" , position="@AllMesh.position")
                #boxFixed = completeMesh.addObject('BoxROI', name='FixedPart', box=[-40, -40, -62, 40, 40, -46], drawBoxes='true', tetrahedra="@AllMesh.tetrahedra" , position="@AllMesh.position")   
                boxTip.init()
                boxBody.init()
                #boxFixed.init()

                
                positionAllPoints = MeshTetra.findData('position').value;
                nbPoints = len(positionAllPoints)
                
                print(boxTip.indices.value)
                indicesTip= np.array(boxTip.indices.value);
                #indicesFixed= boxFixed.indices;
                #indicesRigid = indicesFixed + indicesTip
                indicesRigid = np.array(indicesTip)
                print('indicesRigid ' + str(indicesRigid))
                indicesRigid = indicesRigid.flatten()
                print('indicesRigid flatten ' + str(indicesRigid))
                indicesTip = indicesTip.flatten()
                #indicesFixed = flatten(indicesFixed)
                
                #rigidBlocks = [indicesFixed+indicesTip]
                rigidBlocks = [indicesTip]            
                
                indicesDeformable= np.array(boxBody.findData('indices').value);
                indicesDeformable = indicesDeformable.flatten()
                freeBlocks = indicesDeformable
                print(nbPoints)
                print('+++++++++++++++++++++++++++++++++++')
                print(rigidBlocks)
                print('+++++++++++++++++++++++++++++++++++')
                print(freeBlocks)
                print ('+++++++++++++++++++++++++++++++++++')
#
                indexPairs = np.array(rigidification.fillIndexPairs(nbPoints,freeBlocks,rigidBlocks))
                print('indexPairs ')
                print(indexPairs)
                print('End of indexPairs ')
                pointsBody = completeMesh.MainBody.pointsInROI
                deformablePoints = pointsBody
                pointsTip = np.array(completeMesh.Tip.pointsInROI.value).flatten().tolist()
                #pointsFixed = completeMesh.FixedPart.pointsInROI
                #rigidPoints = pointsFixed + pointsTip
                rigidPoints = pointsTip

                nbRigidPoints = len(rigidPoints)
                rigidIndexPerPoint = [0] * (nbRigidPoints-1) 
                #rigidIndexPerPoint[len(pointsFixed):nbRigidPoints]=[1]*len(pointsTip)
             
                
    
                solverNode = rootNode.addChild("solverNode")
                solverNode.addObject('EulerImplicitSolver',rayleighStiffness="0.1", rayleighMass="0.1")
                solverNode.addObject('SparseLDLSolver',name='preconditioner')
                solverNode.addObject('GenericConstraintCorrection', solverName='preconditioner')
                solverNode.addObject('MechanicalMatrixMapper', template='Vec3d,Rigid3d', object1='@./deformableNode/DeformableMech', object2='@./RigidNode/RigidMesh', nodeToParse='@./deformableNode/model' )
                solverNode.addObject('MechanicalMatrixMapper', template='Vec3d,Rigid3d', object1='@./deformableNode/DeformableMech', object2='@./RigidNode/RigidMesh', nodeToParse='@./deformableNode/model/FiberNode' )
#                solverNode.addObject('MechanicalMatrixMapper', template='Vec3d,Rigid3d', object1='@./deformableNode/DeformableMech', object2='@./RigidNode/RigidMesh', nodeToParse='@./deformableNode/model/FiberNode2' )
#                solverNode.addObject('MechanicalMatrixMapper', template='Vec3d,Rigid3d', object1='@./deformableNode/DeformableMech', object2='@./RigidNode/RigidMesh', nodeToParse='@./deformableNode/model/FiberNode3' )
#                solverNode.addObject('MechanicalMatrixMapper', template='Vec3d,Rigid3d', object1='@./deformableNode/DeformableMech', object2='@./RigidNode/RigidMesh', nodeToParse='@./deformableNode/model/FiberNode4' )
#                solverNode.addObject('MechanicalMatrixMapper', template='Vec3d,Rigid3d', object1='@./deformableNode/DeformableMech', object2='@./RigidNode/RigidMesh', nodeToParse='@./deformableNode/model/FiberNode5' )
                
                #solverNode.addObject('MechanicalMatrixMapper', template='Rigid,Rigid', object1='@./RigidNode/RigidMesh', object2='@./RigidNode/RigidMesh', nodeToParse='@./RigidNode/RigidifiedNode', stopAtNodeToParse=True )
                deformableNode = solverNode.addChild("deformableNode")
                deformableNode.addObject('PointSetTopologyContainer', position='@'+completeMesh.getPathName()+'/MainBody.pointsInROI')
                deformableNode.addObject('MechanicalObject', name='DeformableMech')
                #deformableNode.addObject('BoxROI', name='BoxForSliding', box=Constants.SlidingBoxCoords, drawBoxes='true')
                #deformableNode.addObject('PartialFixedConstraint', indices='@BoxForSliding.indices', fixedDirections=[0,0,1])
                
                
                
                RigidNode= solverNode.addChild('RigidNode')

                #RigidNode.addObject("MechanicalObject",template="Rigid3d",name="RigidMesh", position=[[0, 0, Const.Height, 0.23622992, 0.30639972, 0.12644464, 0.91341469]], showObject=True, showObjectScale=5) # orientation is 240 deg away from scene origin
                
                #TipOrientation = [0.14224426, 0.0996005 , 0.56486252, 0.80670728]#  #[0.       , 0.       , 0.5      , 0.8660254] #[0, 0., 0.70710678, -0.70710678] #
                #TipOrientation = [0,0,0,1]
                #TipOrientation = [0.70710678, 0.        , 0.        , 0.70710678]
                TipOrientation = [1, 0, 0, 0]
                RigidNode.addObject("MechanicalObject",template="Rigid3d",name="RigidMesh", position=[[0, 0, Const.Height]+TipOrientation], showObject=True, showObjectScale=5) 

                #RigidNode.addObject('BoxROI', name='BoxForSliding', box=Constants.SlidingBoxCoords, drawBoxes='true')
                #RigidNode.addObject('PartialFixedConstraint', indices=[0], fixedDirections=[0,0,0,1,1,0])
             
                RigidifiedNode= RigidNode.addChild('RigidifiedNode')
                
                RigidifiedNode.addObject('MechanicalObject', name='RigidifiedMesh', position = rigidPoints, template = 'Vec3d')
                
                RigidifiedNode.addObject("RigidMapping",globalToLocalCoords="true", rigidIndexPerPoint=rigidIndexPerPoint)  
                
                model = deformableNode.addChild('model')
                RigidifiedNode.addChild(model)
                
                	#Heart
                model.addObject('EulerImplicitSolver', name='odesolver')
                #model.addObject('ShewchukPCGLinearSolver', iterations='15', name='linearsolver', tolerance='1e-5', preconditioners='preconditioner', use_precond=True, update_step='1')
                model.addObject('ShewchukPCGLinearSolver', iterations='15', name='linearsolver', tolerance='1e-5', update_step='1')
    
                model.addObject('MeshVTKLoader', name='loader', filename=VolumetricMeshPath)
                model.addObject('TetrahedronSetTopologyContainer', src='@loader', name='container')

                model.addObject('MechanicalObject', name='tetras', template='Vec3', showIndices=False, showIndicesScale='4e-5', rx='0', dz='0')
                model.addObject('UniformMass', totalMass='0.001')
                model.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio='0.3',  youngModulus='18000')

                model.addObject('BoxROI', name='boxROI', box=Const.BaseFixedBoxCoords, drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                model.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness='1e12')

#               model.addObject('SparseLDLSolver', name='preconditioner')
#                model.addObject('LinearSolverConstraintCorrection', solverName='preconditioner')
#    
#                model.addObject('MeshVTKLoader', name='loader', filename=VolumetricMeshPath)
#                model.addObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
#                
#                model.addObject('MechanicalObject', name='tetras', template='Vec3d', showIndices='false', showIndicesScale='4e-5')
#                model.addObject('UniformMass', totalMass='0.3')
#                model.addObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio=Constants.PoissonRation,  youngModulus=Constants.YoungsModulus, drawAsEdges="0")   
#                   
                model.addObject("SubsetMultiMapping",name="subsetMapping",template="Vec3d,Vec3d", input='@'+deformableNode.getPathName()+'/DeformableMech' + ' ' + '@'+RigidifiedNode.getPathName()+'/RigidifiedMesh' , output='@./tetras', indexPairs=indexPairs.tolist())
                
                
                PointsArray1 = np.array([[10,15,-60],[19,10,-60],[21.5,5,-60],[22.5,0,-60],[21.5,-5,-60],[19,-10,-60],[10,-15,-60],
                           [10,16.5,-20],[19,12.2,-20],[21.5,8.5,-20],[24,0,-20],[21.5,-8.5,-20],[19,-12.2,-20],[10,-16.5,-20],
                           [10,15.7,-40],[19,10.9,-40],[21.5,6.8,-40],[23.2,0,-40],[21.5,-6.8,-40],[19,-10.9,-40],[10,-15.7,-40],
                           [10,14.4,-80],[19,8.9,-80],[21.5,3.5,-80],[21,0,-80],[21.5,-3.5,-80],[19,-8.9,-80],[10,-14.4,-80],
                           [10,11,-100],[19,8,-100],[21.5,1.5,-100],[19.6,0,-100],[21.5,-1.5,-100],[19,-8,-100],[10,-11,-100]])
                PointsArray1 = PointsArray1 + np.array([2,0.2,0])
                MyRotation1 = R.from_euler("XYZ", [0,0,-2*np.pi/3])
                MyRotation2 = R.from_euler("XYZ", [0,0,2*np.pi/3])
                
                PointsArray2=MyRotation1.apply(PointsArray1)
                PointsArray3=MyRotation2.apply(PointsArray1)
                PointsArray1=PointsArray1.tolist()
                PointsArray2=PointsArray2.tolist()
                PointsArray3=PointsArray3.tolist()
                PointsArray=PointsArray1+PointsArray2+PointsArray3
                edges_list=[]
                for i in range(len(PointsArray)-1):
                    if (i+1)%7!=0:
                        edges_list.append([i,i+1])
                    
                FiberNode = model.addChild("FiberNode")
                FiberNode.addObject("Mesh",name="Mesh", position=PointsArray, edges=edges_list)
                FiberNode.addObject("MechanicalObject")
                FiberNode.addObject("MeshSpringForceField",linesStiffness=1e12)
                FiberNode.addObject("BarycentricMapping")
     
                
                
#                for Points in PointsArray:
#                    FiberNode = model.addChild("FiberNode"+str(count))
#                    FiberNode.addObject("Mesh",name="Mesh", position=Points, edges=[[0,1],[1,2],[2,3],[3,4],[4,5],[5,6]])
#                    FiberNode.addObject("MechanicalObject")
#                    FiberNode.addObject("MeshSpringForceField",linesStiffness=1e12)
#                    FiberNode.addObject("BarycentricMapping")
#                    count+=1
#     

                
#                #----------------------
#                # Rigidification - end
#                #----------------------
                   
#                model = rootNode.addChild('model')
#                model.addObject('EulerImplicit', name='odesolver')
#                model.addObject('PCGLinearSolver', name='linearSolver',iterations='25', tolerance='1.0e-9', preconditioners="precond")
#                model.addObject('SparseLDLSolver', name='precond')
#
#                model.addObject('MeshVTKLoader', name='loader', filename=VolumetricMeshPath, scale3d=[1, 1, 1])
#                model.addObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
#                model.addObject('TetrahedronSetTopologyModifier')
#
#                model.addObject('MechanicalObject', name='tetras', template='Vec3d', showIndices='false', showIndicesScale='4e-5')
#                model.addObject('UniformMass', totalMass='0.1')
#                model.addObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio=Const.PoissonRatio,  youngModulus=Const.YoungsModulus)
#
#                #model.addObject('BoxROI', name='boxROI', box='-10 -15 50 10 15 90', drawBoxes='true')
#                #model.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness="1e2")
#                
#                model.addObject('BoxROI', name='BoxROI1', box=Const.BaseFixedBoxCoords, drawBoxes=True)
#                model.addObject('RestShapeSpringsForceField', points='@BoxROI1.indices', stiffness='1e12')        
#
#                model.addObject('LinearSolverConstraintCorrection', name='GCS', solverName='precond')



		        ##########################################
                # Visualization                          #
                ##########################################
                modelVisu = model.addChild('visu')
                modelVisu.addObject('MeshSTLLoader', filename=SurfaceMeshPath, name="loader")
                modelVisu.addObject('OglModel', src="@loader", scale3d=[1, 1, 1])
                modelVisu.addObject('BarycentricMapping')
                
                ##########################################
                # Effector                               #
                ########################################## 

                #MOPositions = [[0,0,0,0,0,1,0], [0,0,-100,0,0,1,0]]
                #Sensor = RigidNode.addChild('BendLabsSensor')
                
                #RigidNode.addObject('MechanicalObject', name='SensorPoints', template="Rigid3d", showObject=True, showObjectScale=5, position=MOPositions)
                
                #goal = rootNode.addChild('goal')
                #goal.addObject('MechanicalObject', name="goalMO", template='Rigid3d', position=[[0,10,-90,0., 0. , 0.25881905, -0.96592583]], showObject=True, showObjectScale=10)
                #RigidNode.addObject('PositionEffector', template='Rigid3d', indices=[0], effectorGoal="@../../goal/goalMO.position", useDirections='0 1 0 0 0 1')
                
                
                #effector.addObject('PositionEffector', indices=[i for i in range(len(goalMO.position.value))], template='Vec3', effectorGoal="@../../goal/goalMO.position", useDirections='1 1 1')
              
                #RigidNode.addObject('BendLabsEffector', template='Rigid3d', alpha=0.39269908169872414, beta=0.39269908169872414)
               
                
                BaseOrientation = TipOrientation
                #BaseOrientation = [0,0,0,1]
                #BLE = RigidNode.addObject('BendLabsEffector', template='Rigid3d', BaseCoord = [0,0,0]+BaseOrientation, theta=0, phi=0)
                #RigidNode.addObject('BarycentricMapping')
                
                
#                MOPositions = [[0,0,0], [0,0,-100]]
#                RigidFrame = model.addChild('RigidFrame')
#                RigidFrame.addObject("MechanicalObject",template="Rigid3d",name="RigidMesh", position=[0, 0,-100, 0, 0, 1, 0], showObject=True, showObjectScale=20) # orientation is 240 deg away from scene origin
#               
#                RigidFrame.addObject('BarycentricMapping')


##                ##########################################
##                # Actuation by forces or torques         #
##                ########################################## 
##                 

#
#                # Use force point actuators on the tip (rigid)
#                FPANode = RigidNode.addChild('FPANode')
#                FPANode.addObject('MechanicalObject', template='Rigid3', position=[0,0,Const.Height,0,0,1,0])
#                FPANode.addObject('ForcePointActuator', name='FPA1', template='Rigid3', direction='0 0 0 0 1 0', indices=0, maxForce=1000000, minForce=-1000000, showForce=True, visuScale=20) #, showDirection=True, showVisuScale=10)                
#                FPANode.addObject('ForcePointActuator', name='FPA2', template='Rigid3', direction='0 0 0 1 0 0', indices=0, maxForce=1000000, minForce=-1000000, showForce=True, visuScale=20)                
#                #FPANode.addObject('ForcePointActuator', name='FPA3', template='Rigid3', direction='0 0 1 0 0 0', indices=0, maxForce=100, minForce=-100, showForce=True, visuScale=20)                
#                FPANode.addObject("IdentityMapping")
#                


#                # Use force point actuators on the tip (rigid)
#                FPANode = RigidNode.addChild('FPANode')
#                FPANode.addObject('MechanicalObject', template='Rigid3', position=[0,0,Const.Height,0,0,1,0])
#                FPANode.addObject('ForcePointActuator', name='FPA1', template='Rigid3', direction='0 0 0 0 1 0', indices=0, maxForce=1000000, minForce=-1000000, showForce=True, visuScale=20) #, showDirection=True, showVisuScale=10)                
#                FPANode.addObject('ForcePointActuator', name='FPA2', template='Rigid3', direction='0 0 0 1 0 0', indices=0, maxForce=1000000, minForce=-1000000, showForce=True, visuScale=20)                
#                #FPANode.addObject('ForcePointActuator', name='FPA3', template='Rigid3', direction='0 0 1 0 0 0', indices=0, maxForce=100, minForce=-100, showForce=True, visuScale=20)                
#                FPANode.addObject("IdentityMapping")
#                



#                ##########################################
#                # Actuation by pneumatic cavities        #
#                ########################################## 
#                 

                # Use force point actuators on the tip (rigid)
                SPCNode = model.addChild('SPCNode')
                for i in range(1,4):
                    CurrentCavity = SPCNode.addChild('Cavity0'+str(i))
                    CurrentCavity.addObject('MeshSTLLoader', name='loader', filename=GeneratedMeshesPath+'Cavity0'+str(i)+ '.stl')
                    CurrentCavity.addObject('MeshTopology', src='@loader', name='topo')
                    CurrentCavity.addObject('MechanicalObject', name='SPCNode')
                    CurrentCavity.addObject('SurfacePressureConstraint', name='SPC'+str(i), template='Vec3', triangles='@topo.triangles', maxPressure='100', minPressure='0', drawPressure='1', drawScale='0.02')
                    CurrentCavity.addObject('BarycentricMapping', name='mapping',  mapForces=False, mapMasses=False)

                
                
                rootNode.addObject(OrientationSweepController(name="OrientationSweeController", SPCNode=SPCNode))
                #RigidNode.addObject('ForcePointActuator', name="FPA1", indices=[0], showForce=1, visuScale=0.1)
                #FPAs.addObject('ForcePointActuator', name="FPA2", indices=[0], direction=[0,1,0], showForce=1, visuScale=0.1)
                #FPAs.addObject('ForcePointActuator', name="FPA3", indices=[0], direction=[1,0,0], showForce=1, visuScale=0.1)                
                #FPAs.addObject('ForcePointActuator', name="FPA2", indices=[], direction=[0,0,1], showForce=1, visuScale=0.1)                
                #FPAs.addObject('BarycentricMapping', mapForces="true", mapMasses="false")

                # This create a BarycentricMapping. A BarycentricMapping is a key element as it will create a bi-directional link
                # between the cable's DoFs and the finger's ones so that movements of the cable's DoFs will be mapped
                # to the finger and vice-versa;
                
                
                #model.addObject('PythonScriptController', filename="PythonScripts/Finger_InverseController.py", classname="Controller")           


                return rootNode
