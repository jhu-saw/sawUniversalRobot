# Start UR controller

import os, sys, ctypes

# Set RTLD_GLOBAL flag for dynamic loading (on Linux)
try:
   flags = sys.getdlopenflags()
   sys.setdlopenflags(flags | ctypes.RTLD_GLOBAL)
except AttributeError as e:
    print('Skipping dlopen flags, ' + str(e))

import cisstCommonPython as cisstCommon

# Set up cisst logging system to print errors, warnings, and verbose (but not debug)
cisstCommon.cmnLogger.SetMask(cisstCommon.CMN_LOG_ALLOW_VERBOSE)
cisstCommon.cmnLogger.SetMaskFunction(cisstCommon.CMN_LOG_ALLOW_VERBOSE)
cisstCommon.cmnLogger.SetMaskDefaultLog(cisstCommon.CMN_LOG_ALLOW_VERBOSE)
cisstCommon.cmnLogger.AddChannelToStdOut(cisstCommon.CMN_LOG_ALLOW_ERRORS_AND_WARNINGS)

def log():
   os.system('tail cisstLog.txt')

import cisstMultiTaskPython as cisstMultiTask
import cisstParameterTypesPython as cisstParameterTypes
import numpy

LCM = cisstMultiTask.mtsManagerLocal.GetInstance()
LCM.CreateAll()
LCM.StartAll()

arg = cisstMultiTask.mtsTaskContinuousConstructorArg('URserver', 256, True)
URserver = cisstMultiTask.mtsLoadAndCreateServer('sawUniversalRobot',
                                                 'mtsUniversalRobotScriptRT',
                                                 'URserver', arg)
if URserver:
   print('Configuring UR server.')
   # Python2 uses raw_input and Python3 uses input
   try:
       ipAddr = raw_input('Enter IP address: ')
   except NameError:
       ipAddr = input('Enter IP address: ')
   URserver.Configure(ipAddr)
   # robot is the required interface
   robot = cisstMultiTask.mtsCreateClientInterface('URclient', 'URserver', 'control')

LCM.CreateAllAndWait(2.0)
LCM.StartAllAndWait(2.0)
print('System ready. Type dir(robot) to see available commands.')
