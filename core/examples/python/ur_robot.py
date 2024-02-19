# Start UR controller

import os, sys, ctypes

# Set RTLD_GLOBAL flag for dynamic loading (on Linux)
try:
   flags = sys.getdlopenflags()
   sys.setdlopenflags(flags | ctypes.RTLD_GLOBAL)
except AttributeError, e:
    print 'Skipping dlopen flags, ', e

import cisstCommonPython as cisstCommon

# Set up cisst logging system to print errors, warnings, and verbose (but not debug)
cisstCommon.cmnLogger.SetMask(cisstCommon.CMN_LOG_ALLOW_VERBOSE)
cisstCommon.cmnLogger.SetMaskFunction(cisstCommon.CMN_LOG_ALLOW_VERBOSE)
cisstCommon.cmnLogger.SetMaskDefaultLog(cisstCommon.CMN_LOG_ALLOW_VERBOSE)
cisstCommon.cmnLogger.AddChannelToStdOut(cisstCommon.CMN_LOG_ALLOW_ERRORS_AND_WARNINGS)

def log():
   os.system('tail cisstLog.txt')

import cisstMultiTaskPython as cisstMultiTask
import numpy

LCM = cisstMultiTask.mtsManagerLocal_GetInstance()
print 'Creating UR client'
URclient = cisstMultiTask.mtsComponentWithManagement('URclient')
LCM.AddComponent(URclient)
LCM.CreateAll()
LCM.StartAll()

Manager = URclient.GetManagerComponentServices()
print 'Loading sawUniversalRobotController'
if not Manager.Load('sawUniversalRobotController'):
    print 'Failed to load sawUniversalRobotController (see cisstLog.txt)'

print 'Creating UR server (mtsUniversalRobotScriptRT)'
# Or, could use LCM.CreateComponentDynamically
Manager.ComponentCreate('mtsUniversalRobotScriptRT', 'URserver')
URserver = LCM.GetComponent('URserver')
print 'Configuring UR server.'
ipAddr = raw_input('Enter IP address: ')
URserver.Configure(ipAddr)
URserver.Create()

print 'Connecting UR client to UR server'
# robot is the required interface
robot = URclient.AddInterfaceRequiredAndConnect(('URserver', 'control'))

print 'Starting UR server'
URserver.Start()

print 'System ready. Type dir(robot) to see available commands.'
