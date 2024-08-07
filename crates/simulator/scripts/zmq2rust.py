import zmq
import json

class ZMQ_REQ:
    def __init__(self, localhost) -> None:
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)  # 请求（Request）套接字
        self.socket.connect(localhost)
    
    def send_array(self, data):
        # 将数组序列化为 JSON 字符串
        message = json.dumps(data)
        # 发送 JSON 字符串
        self.socket.send_string(message)

    def receive_command(self):
        # 等待接收回复
        message = self.socket.recv_string()
        # 反序列化为 Python 列表
        cmd = json.loads(message)
        return cmd