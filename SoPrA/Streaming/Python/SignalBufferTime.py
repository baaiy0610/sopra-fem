import numpy as np
import datetime
import time

class SignalBufferTimeAngle:
    def __init__(self, Parent, Name='Signal', TimeWindowSize=2, SensorModel=0):
        self.Name = Name
        self.Parent = Parent        
        self.DataRaw = np.array([0,0]) # put in some values so that operations later don't fail ...        
        self.Data = np.array([0,0])
        self.DataIn_deg = np.array([0,0])
        self.TimeStamps = np.array([0,0])
        self.Offset = 0
        self.Mean = 0
        self.Std = 0
        self.Threshold = 1
        self.PushedOutVol = np.array([0,0]) # Integral of the signal
        self.DataIn_degPerSec = np.array([0,0]) # Derivative of the signal       
        self.TimeWindowSize = TimeWindowSize #seconds
        self.SensorModel = SensorModel # Allow to connect different sensors to one arduino board and convert values accordingly
        
    def flushBuffer(self):        
        self.DataRaw = np.array([0,0]) # put in some values so that operations later don't fail ...        
        self.Data = np.array([0,0])
        self.DataIn_deg = np.array([0,0])
        self.TimeStamps = np.array([0,0])        
        self.PushedOutVol = np.array([0,0]) # Integral of the signal
        self.DataIn_degPerSec = np.array([0,0]) # Derivative of the signal               
        
    def updateBuffer(self, NewValue, TimeStamp):               
        
        self.CurrentTimeStamp = TimeStamp
        self.TimeStamps = np.append(self.TimeStamps, TimeStamp)
        self.DataRaw = np.append(self.DataRaw, NewValue)        
        self.Mean = np.mean(self.DataRaw)
        self.Data = self.DataRaw - self.Offset
        
        AD2V = 1.0/205.0;
        V2kPa_MPX4250 = 1000/19  # MPX4250 is rated at 19mv/kPa i.e. (1000/19)kPa/V = ~52.6
        #LPerMin2LPerSec = 1.0/60.0;
   
            
   
        if self.SensorModel==0:     
            self.DataIn_deg = self.DataRaw
        elif self.SensorModel==1:
            self.DataIn_deg = self.DataRaw
        elif self.SensorModel==2:
            self.DataIn_deg = self.Data * AD2V * V2kPa_MPX4250

               
        # Calculate derivative of flow rate (= flow acceleration)
        
        SmoothDerivativeWindowSize = 15
        
        if len(self.TimeStamps) > SmoothDerivativeWindowSize:
            DeltaT = self.TimeStamps[-1] - self.TimeStamps[-SmoothDerivativeWindowSize]
            self.DataIn_degPerSec = np.append(self.DataIn_degPerSec, (self.DataIn_deg[-1]-self.DataIn_deg[-SmoothDerivativeWindowSize])/DeltaT)                
        else:
            self.DataIn_degPerSec = np.append(self.DataIn_degPerSec, 0)                
        
        # Calculate (pseudo) integral to have an approximate measure of total volume (not reliable in case of saturation)
               
        #if self.Parent!=None:
            #if self.Parent.ContactDetected:
                ##if abs(self.DataIn_deg[-1]) > 0.000006: # for the future (needs more robustness)
                #if abs(self.DataIn_deg[-1]) > 0.00001: # for the future (needs more robustness)
                    ##if self.Data[-1] > 2:
                    #self.PushedOutVol = np.append(self.PushedOutVol, self.PushedOutVol[-2] + self.Parent.DeltaT*abs(self.DataIn_deg[-1]))
                #else:
                    #self.PushedOutVol = np.append(self.PushedOutVol, self.PushedOutVol[-2])
            #else:                
                #self.PushedOutVol = np.append(self.PushedOutVol, 0)
        #self.PushedOutVol = np.append(self.PushedOutVol, self.DataIn_deg[-1])
        self.PushedOutVol = np.append(self.PushedOutVol, self.DataRaw[-1])
                
        TruncationIdx = self.findTimeTruncationIdx(self.TimeWindowSize, self.CurrentTimeStamp)
        self.truncateBuffer(TruncationIdx)
        
        
        
    def truncateBuffer(self, NElements):
        # truncate the buffer from the beginning, so that the first NElements are thrown out of the array
        # This function is necessary to be able to hold only elements within a time range in the array
        # The length of the buffer will no longer be constant!
        self.TimeStamps = self.TimeStamps[NElements:]
        self.DataRaw = self.DataRaw[NElements:]
        self.Data = self.Data[NElements:]
        self.DataIn_deg = self.DataIn_deg[NElements:]        
        self.DataIn_degPerSec = self.DataIn_degPerSec[NElements:]
        self.PushedOutVol = self.PushedOutVol[NElements:]
        
         
    def findOffset(self):
       
        self.Offset = self.Mean
        self.Std = np.std(self.DataRaw)
    
              
    def findTimeTruncationIdx(self, TimeWindowSize, CurrentTimeStamp):
        Idx = 0
        Found = False
        TimeThreshold = CurrentTimeStamp - TimeWindowSize
        while not Found:
            if self.TimeStamps[Idx] < TimeThreshold:
                Idx = Idx + 1
            else:
                Found = True                
        
        return Idx
    
    def saveBuffer(self, Name, Path='/home/prostatesim/Repos/PneumaticSensing/ForceTestBed/Data'):
        t = datetime.datetime.now()
        DateTimeStr = t.strftime('%Y-%m-%d_%H:%M:%S')
        np.savez(Path+'/' + DateTimeStr + '_' + Name, DataRaw=self.DataRaw)
