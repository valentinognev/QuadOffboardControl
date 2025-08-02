topic2portDict = {}
       
topicMavlinkPort             = 7790
topicMavlinkAltitude         = b'ALTITUDE'
topicMavlinkAttitude         = b'ATTITUDE'
topicMavlinkAttitudeRate     = b'ATTITUDE_RATE'
topicMavlinkHighresIMU       = b'HIGHRES_IMU'
topicMavlinkAttitudeQuat     = b'ATTITUDE_QUATERNION'
topicMavlinkLocalPositionNed = b'LOCAL_POSITION_NED'
topicMavlinkVelocityNed      = b'VELOCITY_NED'
topicMavlinkOdometry         = b'ODOMETRY'
topicMavlinkGlobalPositionInt= b'GLOBAL_POSITION_INT'
topicMavlinkAttitudeTarget   = b'ATTITUDE_TARGET'
topicMavlinkCurrentMode      = b'CURRENT_MODE'
topicMavlinkHeartbeat        = b'HEARTBEAT'
topicMavlinkVFR_HUD          = b'VFR_HUD'
topicMavlinkDistanceSensor   = b'DISTANCE_SENSOR'
topicMavlinkOpticalFlow      = b'OPTICAL_FLOW'

topic2portDict[topicMavlinkAltitude]         = topicMavlinkPort
topic2portDict[topicMavlinkAttitude]         = topicMavlinkPort
topic2portDict[topicMavlinkAttitudeRate]     = topicMavlinkPort
topic2portDict[topicMavlinkHighresIMU]       = topicMavlinkPort
topic2portDict[topicMavlinkAttitudeQuat]     = topicMavlinkPort
topic2portDict[topicMavlinkLocalPositionNed] = topicMavlinkPort
topic2portDict[topicMavlinkVelocityNed]      = topicMavlinkPort
topic2portDict[topicMavlinkOdometry]         = topicMavlinkPort
topic2portDict[topicMavlinkGlobalPositionInt] = topicMavlinkPort    
topic2portDict[topicMavlinkAttitudeTarget]   = topicMavlinkPort
topic2portDict[topicMavlinkCurrentMode]      = topicMavlinkPort
topic2portDict[topicMavlinkHeartbeat]        = topicMavlinkPort
topic2portDict[topicMavlinkVFR_HUD]          = topicMavlinkPort
topic2portDict[topicMavlinkDistanceSensor]   = topicMavlinkPort
topic2portDict[topicMavlinkOpticalFlow]      = topicMavlinkPort

topicGuidenceCmdPort         = 7793
topicGuidenceCmdAttitude     = b'quadAttitudeCmd'
topicGuidenceCmdVelNedYaw       = b'quadAttitudeVelNedYawCmd'
topicGuidenceCmdVelBodyYawRate   = b'quadAttitudeVelBodyYawRateCmd'
topicGuidenceCmdAccYaw       = b'quadAttitudeAccYawCmd'
topicGuidenceCmdTakeoff      = b'quadTakeoffCmd'
topicGuidenceCmdLand          = b'quadLandCmd'
topicGuidanceCmdArm           = b'quadArmCmd'
topic2portDict[topicGuidenceCmdAttitude] = topicGuidenceCmdPort
topic2portDict[topicGuidenceCmdVelNedYaw] = topicGuidenceCmdPort
topic2portDict[topicGuidenceCmdVelBodyYawRate] = topicGuidenceCmdPort
topic2portDict[topicGuidenceCmdAccYaw] = topicGuidenceCmdPort
topic2portDict[topicGuidenceCmdTakeoff] = topicGuidenceCmdPort
topic2portDict[topicGuidenceCmdLand] = topicGuidenceCmdPort
topic2portDict[topicGuidanceCmdArm] = topicGuidenceCmdPort



