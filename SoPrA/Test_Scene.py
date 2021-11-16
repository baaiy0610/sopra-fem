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
                VolumetricMeshPath = GeneratedMeshesPath + 'test.vtk'
                                
                     
                
                
                  #                
                #----------------------
                # Rigidification - start
                #----------------------            
                
                model = rootNode.addChild('model')                
                	#Heart
                model.addObject('EulerImplicitSolver', name='odesolver')
                #model.addObject('ShewchukPCGLinearSolver', iterations='15', name='linearsolver', tolerance='1e-5', preconditioners='preconditioner', use_precond=True, update_step='1')
                model.addObject('ShewchukPCGLinearSolver', iterations='15', name='linearsolver', tolerance='1e-5', update_step='1')
    
                model.addObject('MeshVTKLoader', name='loader', filename=VolumetricMeshPath)
                model.addObject('TetrahedronSetTopologyContainer', src='@loader', name='container')

                model.addObject('MechanicalObject', name='tetras', template='Vec3', showIndices=False, showIndicesScale='4e-5', rx='0', dz='0')
                model.addObject('UniformMass', totalMass='0.09')
                model.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio='0.3',  youngModulus='18000')

                model.addObject('BoxROI', name='boxROI', box=Const.BaseFixedBoxCoords, drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                model.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness='1e12')
           
                return rootNode
