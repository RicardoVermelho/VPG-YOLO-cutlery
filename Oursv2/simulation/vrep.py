# This file is part of the REMOTE API
# 
# Copyright 2006-2017 Coppelia Robotics GmbH. All rights reserved. 
# marc@coppeliarobotics.com
# www.coppeliarobotics.com
# 
# The REMOTE API is licensed under the terms of GNU GPL:
# 
# -------------------------------------------------------------------
# The REMOTE API is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# THE REMOTE API IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
# WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
# AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
# DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
# MISUSING THIS SOFTWARE.
# 
# See the GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with the REMOTE API.  If not, see <http://www.gnu.org/licenses/>.
# -------------------------------------------------------------------
#
# This file was automatically created for V-REP release V3.4.0 rev. 1 on April 5th 2017

import platform
import struct
import sys
import os
import ctypes as ct
from .vrepConst import *

#load library
libsimx = None
try:
    file_extension = '.so'
    if platform.system() =='cli':
        file_extension = '.dll'
    elif platform.system() =='Windows':
        file_extension = '.dll'
    elif platform.system() == 'Darwin':
        file_extension = '.dylib'
    else:
        file_extension = '.so'
    libfullpath = os.path.join(os.path.dirname(__file__), 'remoteApi' + file_extension)
    libsimx = ct.CDLL(libfullpath)
except:
    print ('----------------------------------------------------')
    print ('The remoteApi library could not be loaded. Make sure')
    print ('it is located in the same folder as "vrep.py", or')
    print ('appropriately adjust the file "vrep.py"')
    print ('----------------------------------------------------')
    print ('')

#ctypes wrapper prototypes 
c_GetJointPosition          = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.POINTER(ct.c_float), ct.c_int32)(("simxGetJointPosition", libsimx))
c_SetJointPosition          = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.c_float, ct.c_int32)(("simxSetJointPosition", libsimx))
c_GetJointMatrix            = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.POINTER(ct.c_float), ct.c_int32)(("simxGetJointMatrix", libsimx))
c_SetSphericalJointMatrix   = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.POINTER(ct.c_float), ct.c_int32)(("simxSetSphericalJointMatrix", libsimx))
c_SetJointTargetVelocity    = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.c_float, ct.c_int32)(("simxSetJointTargetVelocity", libsimx))
c_SetJointTargetPosition    = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.c_float, ct.c_int32)(("simxSetJointTargetPosition", libsimx))
c_GetJointForce             = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.POINTER(ct.c_float), ct.c_int32)(("simxGetJointForce", libsimx))
c_SetJointForce             = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.c_float, ct.c_int32)(("simxSetJointForce", libsimx))
c_ReadForceSensor           = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.POINTER(ct.c_ubyte), ct.POINTER(ct.c_float), ct.POINTER(ct.c_float), ct.c_int32)(("simxReadForceSensor", libsimx))
c_BreakForceSensor          = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.c_int32)(("simxBreakForceSensor", libsimx))
c_ReadVisionSensor          = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.POINTER(ct.c_ubyte), ct.POINTER(ct.POINTER(ct.c_float)), ct.POINTER(ct.POINTER(ct.c_int32)), ct.c_int32)(("simxReadVisionSensor", libsimx))
c_GetObjectHandle           = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.POINTER(ct.c_char), ct.POINTER(ct.c_int32), ct.c_int32)(("simxGetObjectHandle", libsimx))
c_GetVisionSensorImage      = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.POINTER(ct.c_int32), ct.POINTER(ct.POINTER(ct.c_byte)), ct.c_ubyte, ct.c_int32)(("simxGetVisionSensorImage", libsimx))
c_SetVisionSensorImage      = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.POINTER(ct.c_byte), ct.c_int32, ct.c_ubyte, ct.c_int32)(("simxSetVisionSensorImage", libsimx))
c_GetVisionSensorDepthBuffer= ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.POINTER(ct.c_int32), ct.POINTER(ct.POINTER(ct.c_float)), ct.c_int32)(("simxGetVisionSensorDepthBuffer", libsimx))
c_GetObjectChild            = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.c_int32, ct.POINTER(ct.c_int32), ct.c_int32)(("simxGetObjectChild", libsimx))
c_GetObjectParent           = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.POINTER(ct.c_int32), ct.c_int32)(("simxGetObjectParent", libsimx))
c_ReadProximitySensor       = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.POINTER(ct.c_ubyte), ct.POINTER(ct.c_float), ct.POINTER(ct.c_int32), ct.POINTER(ct.c_float), ct.c_int32)(("simxReadProximitySensor", libsimx))
c_LoadModel                 = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.POINTER(ct.c_char), ct.c_ubyte, ct.POINTER(ct.c_int32), ct.c_int32)(("simxLoadModel", libsimx))
c_LoadUI                    = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.POINTER(ct.c_char), ct.c_ubyte, ct.POINTER(ct.c_int32), ct.POINTER(ct.POINTER(ct.c_int32)), ct.c_int32)(("simxLoadUI", libsimx))
c_LoadScene                 =  ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.POINTER(ct.c_char), ct.c_ubyte, ct.c_int32)(("simxLoadScene", libsimx))
c_StartSimulation           = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32)(("simxStartSimulation", libsimx))
c_PauseSimulation           = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32)(("simxPauseSimulation", libsimx))
c_StopSimulation            = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32)(("simxStopSimulation", libsimx))
c_GetUIHandle               = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.POINTER(ct.c_char), ct.POINTER(ct.c_int32), ct.c_int32)(("simxGetUIHandle", libsimx))
c_GetUISlider               = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.c_int32, ct.POINTER(ct.c_int32), ct.c_int32)(("simxGetUISlider", libsimx))
c_SetUISlider               = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.c_int32, ct.c_int32, ct.c_int32)(("simxSetUISlider", libsimx))
c_GetUIEventButton          = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.POINTER(ct.c_int32), ct.POINTER(ct.c_int32), ct.c_int32)(("simxGetUIEventButton", libsimx))
c_GetUIButtonProperty       = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.c_int32, ct.POINTER(ct.c_int32), ct.c_int32)(("simxGetUIButtonProperty", libsimx))
c_SetUIButtonProperty       = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.c_int32, ct.c_int32, ct.c_int32)(("simxSetUIButtonProperty", libsimx))
c_AddStatusbarMessage       = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.POINTER(ct.c_char), ct.c_int32)(("simxAddStatusbarMessage", libsimx))
c_AuxiliaryConsoleOpen      = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.POINTER(ct.c_char), ct.c_int32, ct.c_int32, ct.POINTER(ct.c_int32), ct.POINTER(ct.c_int32), ct.POINTER(ct.c_float), ct.POINTER(ct.c_float), ct.POINTER(ct.c_int32), ct.c_int32)(("simxAuxiliaryConsoleOpen", libsimx))
c_AuxiliaryConsoleClose     = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.c_int32)(("simxAuxiliaryConsoleClose", libsimx))
c_AuxiliaryConsolePrint     = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.POINTER(ct.c_char), ct.c_int32)(("simxAuxiliaryConsolePrint", libsimx))
c_AuxiliaryConsoleShow      = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.c_ubyte, ct.c_int32)(("simxAuxiliaryConsoleShow", libsimx))
c_GetObjectOrientation      = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.c_int32, ct.POINTER(ct.c_float), ct.c_int32)(("simxGetObjectOrientation", libsimx))
c_GetObjectPosition         = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.c_int32, ct.POINTER(ct.c_float), ct.c_int32)(("simxGetObjectPosition", libsimx))
c_SetObjectOrientation      = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.c_int32, ct.POINTER(ct.c_float), ct.c_int32)(("simxSetObjectOrientation", libsimx))
c_SetObjectPosition         = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.c_int32, ct.POINTER(ct.c_float), ct.c_int32)(("simxSetObjectPosition", libsimx))
c_SetObjectParent           = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.c_int32, ct.c_ubyte, ct.c_int32)(("simxSetObjectParent", libsimx))
c_SetUIButtonLabel          = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.c_int32, ct.POINTER(ct.c_char), ct.POINTER(ct.c_char), ct.c_int32)(("simxSetUIButtonLabel", libsimx))
c_GetLastErrors             = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.POINTER(ct.c_int32), ct.POINTER(ct.POINTER(ct.c_char)), ct.c_int32)(("simxGetLastErrors", libsimx))
c_GetArrayParameter         = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.POINTER(ct.c_float), ct.c_int32)(("simxGetArrayParameter", libsimx))
c_SetArrayParameter         = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.POINTER(ct.c_float), ct.c_int32)(("simxSetArrayParameter", libsimx))
c_GetBooleanParameter       = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.POINTER(ct.c_ubyte), ct.c_int32)(("simxGetBooleanParameter", libsimx))
c_SetBooleanParameter       = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.c_ubyte, ct.c_int32)(("simxSetBooleanParameter", libsimx))
c_GetIntegerParameter       = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.POINTER(ct.c_int32), ct.c_int32)(("simxGetIntegerParameter", libsimx))
c_SetIntegerParameter       = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.c_int32, ct.c_int32)(("simxSetIntegerParameter", libsimx))
c_GetFloatingParameter      = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.POINTER(ct.c_float), ct.c_int32)(("simxGetFloatingParameter", libsimx))
c_SetFloatingParameter      = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.c_float, ct.c_int32)(("simxSetFloatingParameter", libsimx))
c_GetStringParameter        = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.POINTER(ct.POINTER(ct.c_char)), ct.c_int32)(("simxGetStringParameter", libsimx))
c_GetCollisionHandle        = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.POINTER(ct.c_char), ct.POINTER(ct.c_int32), ct.c_int32)(("simxGetCollisionHandle", libsimx))
c_GetDistanceHandle         = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.POINTER(ct.c_char), ct.POINTER(ct.c_int32), ct.c_int32)(("simxGetDistanceHandle", libsimx))
c_GetCollectionHandle       = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.POINTER(ct.c_char), ct.POINTER(ct.c_int32), ct.c_int32)(("simxGetCollectionHandle", libsimx))
c_ReadCollision             = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.POINTER(ct.c_ubyte), ct.c_int32)(("simxReadCollision", libsimx))
c_ReadDistance              = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.POINTER(ct.c_float), ct.c_int32)(("simxReadDistance", libsimx))
c_RemoveObject              = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.c_int32)(("simxRemoveObject", libsimx))
c_RemoveModel               = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.c_int32)(("simxRemoveModel", libsimx))
c_RemoveUI                  = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.c_int32)(("simxRemoveUI", libsimx))
c_CloseScene                = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32)(("simxCloseScene", libsimx))
c_GetObjects                = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.POINTER(ct.c_int32), ct.POINTER(ct.POINTER(ct.c_int32)), ct.c_int32)(("simxGetObjects", libsimx))
c_DisplayDialog             = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.POINTER(ct.c_char), ct.POINTER(ct.c_char), ct.c_int32, ct.POINTER(ct.c_char), ct.POINTER(ct.c_float), ct.POINTER(ct.c_float), ct.POINTER(ct.c_int32), ct.POINTER(ct.c_int32), ct.c_int32)(("simxDisplayDialog", libsimx))
c_EndDialog                 = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.c_int32)(("simxEndDialog", libsimx))
c_GetDialogInput            = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.POINTER(ct.POINTER(ct.c_char)), ct.c_int32)(("simxGetDialogInput", libsimx))
c_GetDialogResult           = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.POINTER(ct.c_int32), ct.c_int32)(("simxGetDialogResult", libsimx))
c_CopyPasteObjects          = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.POINTER(ct.c_int32), ct.c_int32, ct.POINTER(ct.POINTER(ct.c_int32)), ct.POINTER(ct.c_int32), ct.c_int32)(("simxCopyPasteObjects", libsimx))
c_GetObjectSelection        = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.POINTER(ct.POINTER(ct.c_int32)), ct.POINTER(ct.c_int32), ct.c_int32)(("simxGetObjectSelection", libsimx))
c_SetObjectSelection        = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.POINTER(ct.c_int32), ct.c_int32, ct.c_int32)(("simxSetObjectSelection", libsimx))
c_ClearFloatSignal          = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.POINTER(ct.c_char), ct.c_int32)(("simxClearFloatSignal", libsimx))
c_ClearIntegerSignal        = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.POINTER(ct.c_char), ct.c_int32)(("simxClearIntegerSignal", libsimx))
c_ClearStringSignal         = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.POINTER(ct.c_char), ct.c_int32)(("simxClearStringSignal", libsimx))
c_GetFloatSignal            = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.POINTER(ct.c_char), ct.POINTER(ct.c_float), ct.c_int32)(("simxGetFloatSignal", libsimx))
c_GetIntegerSignal          = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.POINTER(ct.c_char), ct.POINTER(ct.c_int32), ct.c_int32)(("simxGetIntegerSignal", libsimx))
c_GetStringSignal           = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.POINTER(ct.c_char), ct.POINTER(ct.POINTER(ct.c_ubyte)), ct.POINTER(ct.c_int32), ct.c_int32)(("simxGetStringSignal", libsimx))
c_SetFloatSignal            = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.POINTER(ct.c_char), ct.c_float, ct.c_int32)(("simxSetFloatSignal", libsimx))
c_SetIntegerSignal          = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.POINTER(ct.c_char), ct.c_int32, ct.c_int32)(("simxSetIntegerSignal", libsimx))
c_SetStringSignal           = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.POINTER(ct.c_char), ct.POINTER(ct.c_ubyte), ct.c_int32, ct.c_int32)(("simxSetStringSignal", libsimx))
c_AppendStringSignal        = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.POINTER(ct.c_char), ct.POINTER(ct.c_ubyte), ct.c_int32, ct.c_int32)(("simxAppendStringSignal", libsimx))
c_WriteStringStream         = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.POINTER(ct.c_char), ct.POINTER(ct.c_ubyte), ct.c_int32, ct.c_int32)(("simxWriteStringStream", libsimx))
c_GetObjectFloatParameter   = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.c_int32, ct.POINTER(ct.c_float), ct.c_int32)(("simxGetObjectFloatParameter", libsimx))
c_SetObjectFloatParameter   = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.c_int32, ct.c_float, ct.c_int32)(("simxSetObjectFloatParameter", libsimx))
c_GetObjectIntParameter     = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.c_int32, ct.POINTER(ct.c_int32), ct.c_int32)(("simxGetObjectIntParameter", libsimx))
c_SetObjectIntParameter     = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.c_int32, ct.c_int32, ct.c_int32)(("simxSetObjectIntParameter", libsimx))
c_GetModelProperty          = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.POINTER(ct.c_int32), ct.c_int32)(("simxGetModelProperty", libsimx))
c_SetModelProperty          = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_int32, ct.c_int32, ct.c_int32)(("simxSetModelProperty", libsimx))
c_Start                     = ct.CFUNCTYPE(ct.c_int32,ct.POINTER(ct.c_char), ct.c_int32, ct.c_ubyte, ct.c_ubyte, ct.c_int32, ct.c_int32)(("simxStart", libsimx))
c_Finish                    = ct.CFUNCTYPE(None, ct.c_int32)(("simxFinish", libsimx))
c_GetPingTime               = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.POINTER(ct.c_int32))(("simxGetPingTime", libsimx))
c_GetLastCmdTime            = ct.CFUNCTYPE(ct.c_int32,ct.c_int32)(("simxGetLastCmdTime", libsimx))
c_SynchronousTrigger        = ct.CFUNCTYPE(ct.c_int32,ct.c_int32)(("simxSynchronousTrigger", libsimx))
c_Synchronous               = ct.CFUNCTYPE(ct.c_int32,ct.c_int32, ct.c_ubyte)(("simxSynchronous", libsimx))
c_PauseCommunicatio