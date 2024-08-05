use zmq::Context;

fn main() {
    // 创建一个新的zmq上下文
    let context = Context::new();

    // 创建一个REP套接字
    let socket = context.socket(zmq::REP).expect("Failed to create socket");

    // 绑定到tcp地址
    socket.bind("tcp://*:5555").expect("Failed to bind socket");

    println!("Server is running and waiting for requests...");

    loop {
        // 等待接收请求
        let message = socket
            .recv_string(0)
            .expect("Failed to receive message")
            .unwrap();
        println!("Received request: {}", message);

        // 模拟处理一些工作
        std::thread::sleep(std::time::Duration::from_secs(1));

        // 发送回复
        let reply = "World";
        socket.send(reply, 0).expect("Failed to send reply");
        println!("Sent reply: {}", reply);
    }
}
