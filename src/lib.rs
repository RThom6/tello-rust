use core::time;
use std::{
    error::Error,
    net::UdpSocket,
    sync::{Arc, Mutex},
    thread,
    time::{Duration, Instant},
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
    response: Arc<Mutex<Option<String>>>,
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
        let response = Arc::new(Mutex::new(None::<String>));

        let receiver_response = Arc::clone(&response);

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
            response,
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
    fn send_command_without_return() {}

    // Some placeholder code, need to rewrite this
    fn send_command_with_return(&mut self, command: &str, timeout: u64) -> Option<String> {
        let time_since_last_command = Instant::now().duration_since(self.last_command_time);
        // Insert time since command check

        let timestamp = Instant::now();

        self.socket
            .send_to(command.as_bytes(), TELLO_IP)
            .expect("Sending command failed");

        let value = self.response.lock().unwrap();
        while value.is_none() {
            if Instant::now().duration_since(timestamp).as_secs() > timeout {
                // Timeout handling
            }
        }

        self.last_command_time = Instant::now();

        // Clone the Option<String> value from the Mutex
        value.clone()
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

/**
 * This is run on a diff thread as a mutable ref as
 * only one mutable ref can exist at a time.
 * Hence cannot change response within drone freely as drone is being used
 */
struct ResponeReceiver {
    response: Option<&'static str>,
}

struct StateReceiver {
    state: Option<&'static str>, // Placeholder till states sorted
}

impl ResponeReceiver {
    fn new() -> Self {
        ResponeReceiver { response: None }
    }
}

impl StateReceiver {
    fn new() -> Self {
        StateReceiver { state: None }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn system_time_test() {
        let socket = UdpSocket::bind(TELLO_IP).expect("couldn't bind to address");
        let response = Arc::new(Mutex::new(None::<String>));

        let mut d = Drone {
            socket,
            is_flying: false,
            stream_on: false,
            retry_count: 3,
            last_command_time: Instant::now(),
            response,
            state: vec!["a"],
        };

        d.send_command_with_return("a", 6);
    }
}
