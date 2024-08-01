#[cfg(test)]
mod tests {
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
}
