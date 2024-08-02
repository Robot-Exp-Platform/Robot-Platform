#[cfg(test)]
mod tests {
    use std::thread;
    #[test]
    fn ros_version() {
        #[cfg(feature = "ros")]
        {
            print!("ROS Thread Start\n");
        }
        #[cfg(feature = "ros2")]
        {
            print!("ROS2 Thread Start\n");
        }
    }
    #[test]
    fn ros_pub_sub() {
        #[cfg(feature = "ros")]
        {
            rosrust::init("ros_pub_sub");
        }
        let ros_pub = thread::spawn(|| {
            #[cfg(feature = "ros")]
            {
                print!("ROS Publisher\n");
                let chatter_pub = rosrust::publish("chatter", 100).unwrap();
                let mut count = 0;
                let rate = rosrust::rate(10.0);

                while rosrust::is_ok() {
                    let mut msg = rosrust_msg::std_msgs::String::default();
                    msg.data = format!("hello world {}", count);
                    print!("Publishing: {}\n", &msg.data);
                    chatter_pub.send(msg).unwrap();
                    count += 1;
                    rate.sleep();
                }
            }
            #[cfg(feature = "ros2")]
            {
                print!("ROS2 Publisher\n");
            }
        });
        let ros_sub = thread::spawn(|| {
            #[cfg(feature = "ros")]
            {
                print!("ROS Subscriber\n");
                let _subscriber_raii =
                    rosrust::subscribe("chatter", 100, |v: rosrust_msg::std_msgs::String| {
                        // Callback for handling received messages
                        println!("Received: {}", &v.data);
                    })
                    .unwrap();
                rosrust::spin();
            }
            #[cfg(feature = "ros2")]
            {
                print!("ROS2 Subscriber\n");
            }
        });

        ros_pub.join().unwrap();
        ros_sub.join().unwrap();
    }
}
