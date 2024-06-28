use std::{
    error::Error, net::UdpSocket, process, time::{Duration, Instant}
};

const TELLO_IP: &'static str = "0.0.0.0:8889";
const RESPONSE_TIMEOUT: u64 = 7; // Seconds
const TAKEOFF_TIMEOUT: u64 = 20; // Seconds

pub struct Drone {
    socket: UdpSocket,
    is_flying: bool,
    stream_on: bool,
    retry_count: i32,
    last_command_time: Instant,
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
            last_command_time: Instant::now(),
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

    // Some placeholder code, need to rewrite this
    fn send_command_with_return(&mut self, command: &str, timeout: u64) -> &'static str {
        let time_since_last_command = Instant::now().duration_since(self.last_command_time);

        println!("{:?}", time_since_last_command);

        self.last_command_time = Instant::now();
        "placeholder"
    }

    // Sends control command to Tello and waits for a response
    fn send_control_command(&mut self, command: &str, timeout: u64) -> Result<(), Box<dyn Error>> {
        for _ in 0..self.retry_count {
            let response: &str = self.send_command_with_return(command, timeout);

            if response.to_lowercase().contains("ok") {
                return Ok(());
            }
        }

        Err("Command {command} failed to run".into())
    }
}

#[cfg(test)]
mod tests {
    use std::time;
    use std::thread;

    use super::*;

    #[test]
    fn system_time_test() {
        let socket = UdpSocket::bind(TELLO_IP).expect("couldn't bind to address");

        let mut d = Drone {
            socket,
            is_flying: false,
            stream_on: false,
            retry_count: 3,
            last_command_time: Instant::now(),
        };

        d.send_command_with_return("a", 6);
    }
}
