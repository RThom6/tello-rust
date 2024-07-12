use core::time;
use std::{
    collections::HashMap, error::Error, net::UdpSocket, process, sync::{Arc, Mutex}, thread, time::{Duration, Instant}
};
use log::{debug, error};

const TELLO_ADDR: &'static str = "0.0.0.0:8889"; // Need to change later
const TELLO_STATE_ADDR: &'static str = "0.0.0.0:8890"; // Would this not be the same ip as the former?
// 'Any' addr not important if only want working for 1 drone surely
const RESPONSE_TIMEOUT: u64 = 7; // Seconds
const TAKEOFF_TIMEOUT: u64 = 20; // Seconds
const TIME_BTW_COMMANDS: f64 = 0.1; // Seconds

// State field types: states not here are all strings
const INT_STATE_FIELDS: &[&str] = &[
    "mid", "x", "y", "z", "pitch", "roll", "yaw", "vgx", "vgy", "vgz", "templ", 
    "temph", "tof", "h", "bat", "time" 
];
const FLOAT_STATE_FIELDS: &[&str] = &["baro", "agx", "agy", "agz"];

pub struct Drone {
    socket: UdpSocket,
    is_flying: bool,
    stream_on: bool,
    retry_count: i32,
    last_command_time: Instant,
    shared_response: Arc<Mutex<Option<String>>>,
    state: HashMap<String, StateValue>, //State hashmap, closest thing to python dict
}

enum StateValue {
    Int(i32),
    Float(f64),
    Str(String),
}


impl Drone {
    pub fn new() -> Drone {
        let socket = UdpSocket::bind(TELLO_ADDR).expect("couldn't bind to address");

        socket
            .set_read_timeout(Some(Duration::from_secs(RESPONSE_TIMEOUT)))
            .expect("set_read_timeout call failed");
        socket.set_nonblocking(true).unwrap();

        // Shared response variable protected by mutex
        let shared_response = Arc::new(Mutex::new(None::<String>));
        let response_receiver = Arc::clone(&shared_response);
        start_receiver_thread(response_receiver);

        // Shared state response variable protected by mutex
        let state_response = Arc::new(Mutex::new(None::<String>));
        let state_receiver = Arc::clone(&state_response);
        start_state_receiver_thread(state_receiver);

        Drone {
            socket,
            is_flying: false,
            stream_on: false,
            retry_count: 3,
            last_command_time: Instant::now(),
            shared_response,
            state: HashMap::new(),
        }
    }

    pub fn takeoff(&mut self) {
        self.is_flying = true;
    }
}

fn start_receiver_thread(response_receiver: Arc<Mutex<Option<String>>>) {
    thread::spawn(move || {
        let socket = UdpSocket::bind(TELLO_ADDR).expect("Couldn't bind receiver socket");
        let mut buf = [0; 1024];

        loop {
            let (amt, _src) = socket.recv_from(&mut buf).expect("Didn't receive message");
            let received = String::from_utf8_lossy(&buf[..amt]);

            let mut value = response_receiver.lock().unwrap();
            *value = Some(received.to_string());
        }
    });
}

// Edit once addr magic figured out, udp_state_receiver() method in djitellopy
fn start_state_receiver_thread(response_receiver: Arc<Mutex<Option<String>>>) {
    thread::spawn(move || {
        let socket = UdpSocket::bind(TELLO_STATE_ADDR).expect("Couldn't bind receiver socket");
        let mut buf = [0; 1024];

        loop {
            let (amt, _src) = socket.recv_from(&mut buf).expect("Didn't receive message");
            let received = String::from_utf8_lossy(&buf[..amt]);

            let mut value = response_receiver.lock().unwrap();
            *value = Some(received.to_string());
        }
    });
}

/**
 * Private command methods for API
 */
impl Drone {
    fn send_command_without_return(&self, command: &str) {
        self.socket
            .send_to(command.as_bytes(), TELLO_ADDR)
            .expect("Sending command failed");

        println!("Send Command {}", command);
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
            .send_to(command.as_bytes(), TELLO_ADDR)
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
    fn send_control_command(&mut self, command: &str, timeout: u64) -> bool {
        for _ in 0..self.retry_count {
            let response = self
                .send_command_with_return(command, timeout)
                .unwrap_or_else(|| String::from("Attempt failed, retrying"));

            if response.to_lowercase().contains("ok") {
                return true;
            }
        }

        // raise_result_error ?
        return false;
    }

    fn get_current_state(&mut self) -> bool {
        

        return false;
    }
}

// Random Private methods
impl Drone {
    fn parse_state(&mut self, state_str: &str) -> bool {
        let state_str = state_str.trim();

        if state_str.eq("ok") {
            return false;
        }

        for field in state_str.split(';') {
            let split: Vec<&str> = field.split(':').collect();

            if split.len() < 2 {
                continue;
            }
            
            let key = split[0].to_string();
            let value_str = split[1];
            let value: StateValue = match self.state_field_converter(&key, value_str) {
                Ok(v) => v,
                Err(e) => {
                    debug!("Error parsing state value for {}: {} to {}", key, value_str, e);
                    error!("{}", e);
                    continue;
                }
            };

            self.state.insert(key, value);
        }

        true // Placeholder so vs doesnt scream at me
    }

    // Converts fields to the correct type based on field key
    fn state_field_converter(&mut self, key: &str, value_str: &str) -> Result<StateValue, String> {
        if INT_STATE_FIELDS.contains(&key) {
            value_str.parse::<i32>().map(StateValue::Int).map_err(|e| e.to_string())
        } else if FLOAT_STATE_FIELDS.contains(&key) {
            value_str.parse::<f64>().map(StateValue::Float).map_err(|e| e.to_string())
        } else {
            Ok(StateValue::Str(value_str.to_string()))
        }
    }
}

// Public user command methods
impl Drone {
    pub fn connect(&mut self) {
        self.send_control_command("command", RESPONSE_TIMEOUT);

        let reps = 20;
        for i in 0..reps {
            if self.get_current_state() {
                let t = i / reps;

            }

            thread::sleep(Duration::from_secs_f64(1.0 / reps as f64));
        }
    }
}

#[cfg(test)]
mod tests {
    use std::hash::Hash;

    use super::*;

    #[test]
    fn system_time_test() {
        let socket = UdpSocket::bind(TELLO_ADDR).expect("couldn't bind to address");
        let shared_response = Arc::new(Mutex::new(None::<String>));

        let mut d = Drone {
            socket,
            is_flying: false,
            stream_on: false,
            retry_count: 3,
            last_command_time: Instant::now(),
            shared_response,
            state: HashMap::new(),
        };

        d.send_command_with_return("a", 6);
    }
}
