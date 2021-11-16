#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Feb  5 18:22:40 2019

@author: stefan
"""

import DataPlotter
import time

#import FlowSensors
import Streaming

#import MotorControl

AppStartTime = time.time()
#MotorController = MotorControl.MotorControlClass(USBNr=1)
#DataPlottingApp = DataPlotter.PlotApp(True, MotorController)
DataPlottingApp = DataPlotter.PlotApp(True)
#DataPlottingApp = DataPlotter.FakePlotApp(False)
MyAngleSensors = Streaming.StreamingApp(DataPlottingApp, AppStartTime, [0], False)
#MyFlowSensors = FlowSensors.FlowSensorsApp(DataPlottingApp, AppStartTime, [0], False)
