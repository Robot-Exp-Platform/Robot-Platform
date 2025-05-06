use std::{io::Read, net::TcpListener};

fn main() {
    let tcp = TcpListener::bind("localhost:6651").unwrap();

    loop {
        match tcp.accept() {
            Ok((mut stream, addr)) => {
                println!("Accepted connection from {}", addr);
                loop {
                    let mut buffer = [0; 1024];
                    match stream.read(&mut buffer) {
                        Ok(size) => {
                            if let Ok(message) = String::from_utf8(buffer[..size].to_vec()) {
                                println!("Received {} bytes: {}", size, message);
                            } else {
                                println!(
                                    "Received {} bytes, but failed to convert to string",
                                    size
                                );
                            }
                        }
                        Err(e) => {
                            eprintln!("Failed to read from stream: {}", e);
                            break;
                        }
                    }
                }
            }
            Err(e) => {
                eprintln!("Failed to accept connection: {}", e);
            }
        }
    }
}
