use core::time;
use std::{
    error::Error,
    net::UdpSocket,
    sync::{Arc, Mutex},
    process,
    thread,
    time::{Duration, Instant},
};

const TELLO_IP: &'static str = "0.0.0.0:8889";
const RESPONSE_TIMEOUT: u64 = 7; // Seconds
const TAKEOFF_TIMEOUT: u64 = 20; // Seconds
const TIME_BTW_COMMANDS: f64 = 0.1; // Seconds

pub struct Drone {
    socket: UdpSocket,
    is_flying: bool,
    stream_on: bool,
    retry_count: i32,
    last_command_time: Instant,
    shared_response: Arc<Mutex<Option<String>>>,
    state: Vec<&'static str>, //Placeholder vec
}

impl Drone {
    pub fn new() -> Drone {
        let socket = UdpSocket::bind(TELLO_IP).expect("couldn't bind to address");

        socket
            .set_read_timeout(Some(Duration::from_secs(RESPONSE_TIMEOUT)))
            .expect("set_read_timeout call failed");
        socket.set_nonblocking(true).unwrap();

        // Shared response variable protected by mutex
        let shared_response = Arc::new(Mutex::new(None::<String>));

        let receiver_response = Arc::clone(&shared_response);

        thread::spawn(move || {
            let socket = UdpSocket::bind(TELLO_IP).expect("Couldn't bind receiver socket");
            let mut buf = [0; 1024];

            loop {
                let (amt, _src) = socket.recv_from(&mut buf).expect("Didn't receive message");
                let received = String::from_utf8_lossy(&buf[..amt]);

                let mut value = receiver_response.lock().unwrap();
                *value = Some(received.to_string());
            }
        });

        Drone {
            socket,
            is_flying: false,
            stream_on: false,
            retry_count: 3,
            last_command_time: Instant::now(),
            shared_response,
            state: vec!["placeholder"],
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
    fn send_command_without_return(&self, command: &str) {
        self.socket
            .send_to(command.as_bytes(), TELLO_IP)
            .expect("Sending command failed");
    }

    // Send command
    // Option doesn't really make sense, will sort later - Nvm note to self, was Option as i wanted None to be valid
    fn send_command_with_return(&mut self, command: &str, timeout: u64) -> Option<String> {
        let time_since_last_command = Instant::now().duration_since(self.last_command_time);
        if TIME_BTW_COMMANDS.min(time_since_last_command.as_secs_f64()) != TIME_BTW_COMMANDS {
            println!(
                "Command {} executed too soon, waiting {} seconds",
                command, TIME_BTW_COMMANDS
            );
        }

        // Insert time since command check

        let timestamp = Instant::now();

        self.socket
            .send_to(command.as_bytes(), TELLO_IP)
            .expect("Sending command failed");

        let value = self.shared_response.lock().unwrap();
        while value.is_none() {
            if Instant::now().duration_since(timestamp).as_secs() > timeout {
                // Timeout handling
                let temp = format!(
                    "Aborting command '{}'. Did not receive a response after {} seconds",
                    command, timeout
                );
                return Some(temp);
            }
        }

        self.last_command_time = Instant::now();
        let temp = value.clone();
        let mut temp = temp.unwrap();
        temp = String::from(temp.trim_end_matches("\r\n"));

        Some(temp) // Once again, option doesn't make sense?
    }

    // Sends control command to Tello and waits for a response
    fn send_control_command(&mut self, command: &str, timeout: u64) -> Result<(), Box<dyn Error>> {
        for _ in 0..self.retry_count {
            let response = self
                .send_command_with_return(command, timeout)
                .unwrap_or_else(|| String::from("Attempt failed, retrying"));

            if response.to_lowercase().contains("ok") {
                return Ok(());
            }
        }

        Err("Command {command} failed to run".into())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn system_time_test() {
        let socket = UdpSocket::bind(TELLO_IP).expect("couldn't bind to address");
        let shared_response = Arc::new(Mutex::new(None::<String>));

        let mut d = Drone {
            socket,
            is_flying: false,
            stream_on: false,
            retry_count: 3,
            last_command_time: Instant::now(),
            shared_response,
            state: vec!["a"],
        };

        d.send_command_with_return("a", 6);
    }
}
