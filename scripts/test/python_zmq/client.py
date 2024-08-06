import zmq
import json
import sys

context = zmq.Context()
socket = context.socket(zmq.REQ)  # 请求（Request）套接字
socket.connect("tcp://localhost:5555")
# 定义一个整数数组
array = [1.1, 2.2, 3.3, 4.4, 5.5]

# 将数组序列化为 JSON 字符串
message = json.dumps(array)

# 发送 JSON 字符串
socket.send_string(message)
print("Sent array:", array)

# 等待接收回复
message = socket.recv_string()
print("Received JSON string:", message)

# 反序列化为 Python 列表
array = json.loads(message)
for i in array:
    print(i)
