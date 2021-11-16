import numpy as np
import threading
from multiprocessing import Process
import serial
import time
import SignalBufferTime
import os

TempPath = os.path.dirname(os.path.abspath(__file__))+'/../../Temp/'
#from keras.models import model_from_json

class FuncThread(threading.Thread):
    def __init__(self,t,*a):
        self._t=t
        self._a=a
        threading.Thread.__init__(self)
    def run(self):
        self._t(*self._a)

class StreamingApp():    

    def __init__(self, DataPlotterApp, StartTime, ACMNrs=[0], Plot2DPos=False):
        # Path for saving current reading for data exchanges with sofa
        self.AngleDataFilePath = TempPath + 'AngleData.txt'
        self.SaveDataToFile = True
        
        
        # timing stuff
        self.starttime = 0
        self.endtime = 0
        self.summation = 0
        self.DeltaT = 0;
        self.AppStartTime = StartTime #time.time()
        self.CurrentTimeStamp = time.time()-self.AppStartTime # Timestamps start counting at 0, i.e. the beginning of the app
        self.TimeWindowSize = 2 #seconds        
        
        # Plotting ...
        
        self.initGUIElements(DataPlotterApp)

        # Plotting ... end
        
        self.SerialObj1 = serial.Serial('/dev/ttyACM'+str(ACMNrs[0]), 115200, timeout=0.5) #port used by the arduino mega board                        
        self.OldValues = [0,0,0]
        self.CurrentValues = self.OldValues
        self.Onset = [0,0,0]
        self.Release = [0,0,0]
        self.States = [0,0,0] # +1 -> onset, -1 release
        self.LastStateEventTimestamp = time.time()
        self.ControlFPS = 4
        
        self.Buff1 = SignalBufferTime.SignalBufferTimeAngle(self, 'alpha')
        self.Buff2 = SignalBufferTime.SignalBufferTimeAngle(self, 'beta')
        self.Buff3 = SignalBufferTime.SignalBufferTimeAngle(self, 'Pressure', SensorModel=2)
        
        
        # running stuff
        self.PauseRunning = False
        self.StopRunning = False
        self.Plotting = False  

        
        self.ExceptionMaxCount = 10
        self.ExceptionCounter = 0     
        
        self.ReadoutThread = FuncThread(self.calculate)
        
        #self.ReadoutThread = Process(target=self.calculate,args=[])
        self.ReadoutThread.start()
        
        print("Finding offsets, don't move for 5 sec!")
        time.sleep(5)
        self.reinitOffsets()
        self.ReleaseTimestamp = time.time()
        
        
    def readFromSerial(self, SerialObj):
        Res = ''        
        try:
            # For some reason during the operation of multiple devices errors during reading are more likely
            #SerialObj.reset_input_buffer()
       
            Res = SerialObj.readline()         
            SerialObj.flush()
            Res = SerialObj.readline()            
            return Res
        except: # SerialError as e:
            print('Error reading from serial: ' + SerialObj.port)                        
            print(self.StopRunning)
            self.ExceptionCounter = self.ExceptionCounter + 1
            if (self.ExceptionCounter > self.ExceptionMaxCount):
                print('stopping readout due to (to many) errors')
                self.stop()
                return
            
            
    
    def pause(self):
        self.PauseRunning = True
        
    def unpause(self):
        self.PauseRunning = False
        
    def reinit(self, DataPlotterApp, StartTime):
        self.initGUIElements(DataPlotterApp)
        self.AppStartTime = StartTime
        self.CurrentTimeStamp = time.time() - self.AppStartTime
        self.Buff1.flushBuffer()
        self.Buff2.flushBuffer()
        
        
        
    def initGUIElements(self, DataPlotterApp):
        # Plotting ...        
        
        #Define colors, slightly unsaturated in order not to strain the eye, especially for the papers
        Magenta = [215,44,215]
        Green = [33,219,33]
        Blue = [55,59,216]
        Cyan = [54,216,216]
        Red = [216,30,30]
        Yellow = [216,216,26]
        BlueForce = [0,3,126]
        
        self.DataPlotterApp = DataPlotterApp        
        self.PlotAngleIdx = self.DataPlotterApp.addPlot('Angle', [-100,100], 'deg or kPa')
        self.SignalAngleIdx1 = self.DataPlotterApp.addSignalToPlot(self.PlotAngleIdx, Red)        
        self.SignalAngleIdx2 = self.DataPlotterApp.addSignalToPlot(self.PlotAngleIdx, Green)
        self.SignalAngleIdx3 = self.DataPlotterApp.addSignalToPlot(self.PlotAngleIdx, Blue)
        
                         
        
        self.PlotDeltaPIdx = self.DataPlotterApp.addPlot('Rate of change', [-300,300], 'deg/s')
        self.SignalDeltaPIdx1 = self.DataPlotterApp.addSignalToPlot(self.PlotDeltaPIdx, Red)        
        self.SignalDeltaPIdx2 = self.DataPlotterApp.addSignalToPlot(self.PlotDeltaPIdx, Green)
        self.SignalDeltaPIdx3 = self.DataPlotterApp.addSignalToPlot(self.PlotDeltaPIdx, Blue)
        
                
        self.PlotVoltageIdx = self.DataPlotterApp.addPlot('Voltage', [0,5], 'V')
        self.SignalVoltageIdx1 = self.DataPlotterApp.addSignalToPlot(self.PlotVoltageIdx, Red)        
        self.SignalVoltageIdx2 = self.DataPlotterApp.addSignalToPlot(self.PlotVoltageIdx, Green)
        self.SignalVoltageIdx3 = self.DataPlotterApp.addSignalToPlot(self.PlotVoltageIdx, Blue)
        

    def reinitOffsets(self):

        self.Buff1.findOffset()
        self.Buff2.findOffset()
        self.Buff3.findOffset()

        self.Buff1.PushedOutVol[-1] = 0
        self.Buff2.PushedOutVol[-1] = 0
        self.Buff3.PushedOutVol[-1] = 0

        
        print (self.Buff1.Name + ', Mean: ' + str(self.Buff1.Offset) + ', Std: ' + str(self.Buff1.Std))
        print (self.Buff2.Name + ', Mean: ' + str(self.Buff2.Offset) + ', Std: ' + str(self.Buff2.Std))          
        print (self.Buff2.Name + ', Mean: ' + str(self.Buff2.Offset) + ', Std: ' + str(self.Buff2.Std))            
        


    def updateBuffers(self):    
        
#        self.CurrentValues1AsStr = self.readFromSerial(self.SerialObj1)
#        self.CurrentValues2AsStr = self.readFromSerial(self.SerialObj2)
#        
        #self.CurrentValues1AsStr = self.SerialObj1.readline()
        #print('1 wee')
        try:
            self.CurrentValues1AsStr = self.readFromSerial(self.SerialObj1)            
            
            #self.CurrentValues2AsStr = self.SerialObj2.readline()
            
            try:                
                StringObj = self.CurrentValues1AsStr.decode()
                #print('StringObj: ' + StringObj)
                Val1,Val2,Val3 = StringObj.split(',')                        
                self.CurrentValues = [float(Val1), float(Val2), float(Val3)] # we ignore the last value of each board for now                    

                self.OldValues = self.CurrentValues            
            except:
                print('asd')
                print('Warning: could not convert either: ' + StringObj)
                self.CurrentValues = self.OldValues                 
                    
            self.CurrentTimeStamp = time.time() - self.AppStartTime
            # Ordering here is due to the convention assumed at the time of constructing the kidney sensor
            # The order corresponds to the kidney viewed from above and then enumarating from top to buttom the cavitie's edges one first encounters
            self.Buff1.updateBuffer(self.CurrentValues[0], self.CurrentTimeStamp) 
            self.Buff2.updateBuffer(self.CurrentValues[1], self.CurrentTimeStamp)               
            self.Buff3.updateBuffer(self.CurrentValues[2], self.CurrentTimeStamp)
            # Plotting ...
            # Flow
            
            self.Plotting = self.DataPlotterApp.isPlotting()
            if(self.Plotting):
                self.DataPlotterApp.updateSignal(self.PlotAngleIdx, self.SignalAngleIdx1, self.Buff1.TimeStamps, self.Buff1.DataIn_deg)
                self.DataPlotterApp.updateSignal(self.PlotAngleIdx, self.SignalAngleIdx2, self.Buff2.TimeStamps, self.Buff2.DataIn_deg)
                self.DataPlotterApp.updateSignal(self.PlotAngleIdx, self.SignalAngleIdx3, self.Buff3.TimeStamps, self.Buff3.DataIn_deg)
                
                
                # DeltaP
                
                self.DataPlotterApp.updateSignal(self.PlotDeltaPIdx, self.SignalDeltaPIdx1, self.Buff1.TimeStamps, self.Buff1.DataIn_degPerSec)
                self.DataPlotterApp.updateSignal(self.PlotDeltaPIdx, self.SignalDeltaPIdx2, self.Buff2.TimeStamps, self.Buff2.DataIn_degPerSec)
                self.DataPlotterApp.updateSignal(self.PlotDeltaPIdx, self.SignalDeltaPIdx3, self.Buff3.TimeStamps, self.Buff3.DataIn_degPerSec)
             
                # Voltage
                
                self.DataPlotterApp.updateSignal(self.PlotVoltageIdx, self.SignalVoltageIdx1, self.Buff1.TimeStamps, self.Buff1.PushedOutVol)
                self.DataPlotterApp.updateSignal(self.PlotVoltageIdx, self.SignalVoltageIdx2, self.Buff2.TimeStamps, self.Buff2.PushedOutVol)
                self.DataPlotterApp.updateSignal(self.PlotVoltageIdx, self.SignalVoltageIdx3, self.Buff3.TimeStamps, self.Buff3.PushedOutVol)
                
            self.CurrentVoltages = [self.Buff1.PushedOutVol[-1],
                                   self.Buff2.PushedOutVol[-1],
                                   self.Buff3.PushedOutVol[-1]]
            
            if(self.SaveDataToFile):
                np.savetxt(self.AngleDataFilePath, [self.Buff1.DataRaw[-1], #order for kidney is from left to right
                                                     self.Buff2.DataRaw[-1],
                                                     self.Buff3.DataRaw[-1],                                                     
                                                     self.States[0],
                                                     self.States[1],
                                                     self.States[2]])
    
            self.endtime = time.time()
            self.DeltaT = self.endtime - self.starttime
            self.starttime = time.time()
        
        except Exception as ex:            
            template = "An exception of type {0} occurred. Arguments:\n{1!r}"
            message = template.format(type(ex).__name__, ex.args)
            print(message)
            print('could not update buffers, triggering reintialization through stop')
            self.stop()
#            
    def stop(self):
        self.StopRunning = True
        
    def calculate(self):        
        while not self.StopRunning:    
            if(not self.PauseRunning):
                self.updateBuffers()                                                                        
                self.Onset = self.detectOnset()
                self.Release = self.detectRelease()
                CurrentTime = time.time()               
                
                if (CurrentTime - self.LastStateEventTimestamp) < 1.5 * 1.0/self.ControlFPS :                    
                    self.States = self.States | self.Onset - self.Release              
                else:                                  
                    self.States = self.Onset - self.Release
                    self.LastStateEventTimestamp = CurrentTime             
                
                if np.sum(np.abs(self.States)) > 0:
                    #pass
                    print('States: ' + str(self.States))
                
        
        print('Finished sampling flow sensors ...')
        
        return
            
    def detectOnset(self):
        Onset = np.array([0,0,0])
        OnsetThreshold = 15 # deg/s
        if (self.Buff1.DataIn_degPerSec[-1] > OnsetThreshold):
            Onset[0] = 1
        
        if (self.Buff2.DataIn_degPerSec[-1] > OnsetThreshold):
            Onset[1] = 1    
        
        if (self.Buff3.DataIn_degPerSec[-1] > OnsetThreshold):
            Onset[2] = 1     
        
        if np.sum(Onset) > 0:
            pass
            #print('Onset: ' + str(Onset))
        
        return Onset
    def detectRelease(self):
        Release = np.array([0,0,0])
        ReleaseThreshold = -35 # deg/s
        if (self.Buff1.DataIn_degPerSec[-1] < ReleaseThreshold):
            Release[0] = 1
        
        if (self.Buff2.DataIn_degPerSec[-1] < ReleaseThreshold):
            Release[1] = 1
        
        if (self.Buff3.DataIn_degPerSec[-1] < ReleaseThreshold):
            Release[1] = 1
        
        if np.sum(Release) > 0:
            pass
            #print('Release: ' + str(Release))
        return Release
    
    def manageState(self, Onset, Release):
        pass
        
        
        
    
    
    
    
    
    