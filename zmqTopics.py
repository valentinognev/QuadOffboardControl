topic2portDict = {}
       
topicMavlinkPort             = 7790
topicMavlinkFlightData       = b'FLIGHT_DATA'
topic2portDict[topicMavlinkFlightData]       = topicMavlinkPort

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



