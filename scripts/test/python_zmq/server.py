import zmq

context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("tcp://*:10000")
while True:
    msg = socket.recv_string()
    print(msg)
    socket.send_string("ok, thanks")