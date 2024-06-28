use std::{
    error::Error,
    net::UdpSocket,
    time::{Duration, SystemTime},
};

const TELLO_IP: &'static str = "0.0.0.0:8889";
const RESPONSE_TIMEOUT: u64 = 7; // Seconds
const TAKEOFF_TIMEOUT: u64 = 20; // Seconds

pub struct Drone {
    socket: UdpSocket,
    is_flying: bool,
    stream_on: bool,
    retry_count: i32,
}

impl Drone {
    pub fn new() -> Drone {
        let socket = UdpSocket::bind(TELLO_IP).expect("couldn't bind to address");

        socket
            .set_read_timeout(Some(Duration::from_secs(RESPONSE_TIMEOUT)))
            .expect("set_read_timeout call failed");
        socket.set_nonblocking(true).unwrap();
        socket.connect(TELLO_IP).expect("connection failed");

        Drone {
            socket,
            is_flying: false,
            stream_on: false,
            retry_count: 3,
        }
    }

    pub fn takeoff(&mut self) {
        self.is_flying = true;
    }
}

/**
 * Private command methods for API
 */
impl Drone {
    fn send_command_without_return() {

    }

    fn send_command_with_return(&self, command: &str, timeout: u64) -> &'static str {

        "placeholder"
    }

    // Sends control command to Tello and waits for a response
    fn send_control_command(&self, command: &str, timeout: u64) -> Result<(), Box<dyn Error>> {
        for i in 0..self.retry_count {
            let response: &str = self.send_command_with_return(command, timeout);

            if response.to_lowercase().contains("ok") {
                return Ok(());
            }
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {}
