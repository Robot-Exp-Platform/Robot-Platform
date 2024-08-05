#[cfg(test)]
mod tests {
    use std::thread::{self, sleep};
    #[cfg(feature = "rszmq")]
    use zmq;
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
    #[test]
    fn zmq_version() {
        #[cfg(feature = "rszmq")]
        {
            let (major, minor, patch) = zmq::version();
            println!("Current 0MQ version is {}.{}.{}", major, minor, patch);
        }
    }
    #[test]
    fn zmq_pub_sub() {
        let zmq_pub = thread::spawn(|| {
            #[cfg(feature = "rszmq")]
            {
                let context = zmq::Context::new();
                let publisher = context.socket(zmq::PUB).unwrap();
                assert!(publisher.bind("tcp://*:5556").is_ok());
                print!("there is a publisher\n");
                sleep(std::time::Duration::from_secs(1));
                for i in 0..5 {
                    publisher
                        .send(format!("Hello, {}", i).as_bytes(), 0)
                        .unwrap();
                }
            }
        });
        let zmq_sub = thread::spawn(|| {
            #[cfg(feature = "rszmq")]
            {
                let context = zmq::Context::new();
                let subscriber = context.socket(zmq::SUB).unwrap();
                assert!(subscriber.connect("tcp://localhost:5556").is_ok());
                assert!(subscriber.set_subscribe("Hello,".as_bytes()).is_ok());
                print!("there is a subscriber\n");
                for _ in 0..5 {
                    let string = subscriber.recv_string(0).unwrap().unwrap();
                    println!("Received: {:?}", string.as_str());
                }
            }
        });

        zmq_pub.join().unwrap();
        zmq_sub.join().unwrap();
    }
}
