import zmq


context = zmq.Context()

def subscribe(topics, port, ip='127.0.0.1'):
    zmqSub = context.socket(zmq.SUB)
    zmqSub.setsockopt(zmq.SNDHWM, 20)
    zmqSub.setsockopt(zmq.RCVHWM, 20)
    zmqSub.connect("tcp://%s:%d" % (ip,port))
    for topic in topics:
        zmqSub.setsockopt(zmq.SUBSCRIBE,topic)
    return zmqSub

def publisher(port,ip='127.0.0.1'):
    zmqPub = context.socket(zmq.PUB)
    zmqPub.bind("tcp://%s:%d" % (ip,port) )
    return zmqPub
