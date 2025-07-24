
import time
import zmq
import zmqTopics
import zmqWrapper
import pickle

import time


topicsList = [
               [zmqTopics.topicMavlinkAttitude,         zmqTopics.topicMavlinkPort],
               [zmqTopics.topicMavlinkHighresIMU,       zmqTopics.topicMavlinkPort],
               [zmqTopics.topicMavlinkAttitudeQuat,     zmqTopics.topicMavlinkPort],
               [zmqTopics.topicMavlinkLocalPositionNed, zmqTopics.topicMavlinkPort],
               [zmqTopics.topicMavlinkGlobalPositionInt,zmqTopics.topicMavlinkPort],
               [zmqTopics.topicMavlinkAttitudeTarget,   zmqTopics.topicMavlinkPort],
            ]

sockSub = []
mpsDict = {}

for topic in topicsList:
    sockSub.append(zmqWrapper.subscribe([topic[0]], topic[1]))

while 1:
    socks = zmq.select(sockSub, [], [], 0.001)[0]
    if len(socks)>0:
        for sock in socks:
            ret = sock.recv_multipart()
            print(ret)

    time.sleep(0.01)