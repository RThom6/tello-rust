use log::{debug, error};
use std::{
    collections::HashMap,
    error::Error,
    net::UdpSocket,
    process,
    sync::{Arc, Mutex},
    thread,
    time::{Duration, Instant},
};

const TELLO_ADDR: &'static str = "192.168.10.1:8889"; // Need to change later
const TELLO_STATE_ADDR: &'static str = "0.0.0.0:8890"; // Would this not be the same ip as the former?
                                                       // 'Any' addr not important if only want working for 1 drone surely
const RESPONSE_TIMEOUT: u64 = 7; // Seconds
const TAKEOFF_TIMEOUT: u64 = 20; // Seconds
const TIME_BTW_COMMANDS: f64 = 0.1; // Seconds

// State field types: states not here are all strings
const INT_STATE_FIELDS: &[&str] = &[
    "mid", "x", "y", "z", "pitch", "roll", "yaw", "vgx", "vgy", "vgz", "templ", "temph", "tof",
    "h", "bat", "time",
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
        let socket = UdpSocket::bind("0.0.0.0:8889")
            .expect(format!("couldn't bind to address {}", TELLO_ADDR).as_str());

        socket
            .set_read_timeout(Some(Duration::from_secs(RESPONSE_TIMEOUT)))
            .expect("set_read_timeout call failed");
        // let socket = Arc::new(Mutex::new(socket));
        let socket_recv = socket.try_clone().unwrap();

        // Shared response variable protected by mutex
        let shared_response = Arc::new(Mutex::new(None::<String>));
        let response_receiver = Arc::clone(&shared_response);

        thread::spawn(move || {
            let socket = socket_recv;
            let mut buf = [0; 1024];

            loop {
                match socket.recv_from(&mut buf) {
                    Ok((amt, _src)) => {
                        let received = String::from_utf8_lossy(&buf[..amt]);
                        if let Ok(mut value) = response_receiver.lock() {
                            *value = Some(received.to_string());
                        }
                    }
                    Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                        // Ignore the WouldBlock error and continue the loop
                        continue;
                    }
                    Err(e) => {
                        eprintln!("Error receiving message: {:?}", e);
                        break;
                    }
                }
            }
        });

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
}

// fn start_receiver_thread(socket: &UdpSocket, response_receiver: Arc<Mutex<Option<String>>>) {
//     thread::spawn(move || {
//         let socket = socket.clone();
//         let mut buf = [0; 1024];

//         loop {
//             let (amt, _src) = socket.recv_from(&mut buf).expect("Didn't receive message");
//             let received = String::from_utf8_lossy(&buf[..amt]);

//             let mut value = response_receiver.lock().unwrap();
//             *value = Some(received.to_string());
//         }
//     });
// }

// Edit once addr magic figured out, udp_state_receiver() method in djitellopy
fn start_state_receiver_thread(response_receiver: Arc<Mutex<Option<String>>>) {
    thread::spawn(move || {
        let socket = UdpSocket::bind("0.0.0.0:8890").expect("Couldn't bind receiver socket");
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
        // THERE IS AN ERROR WITH THIS, WILL FIX IT IN THE MORNING
        //
        let time_since_last_command = Instant::now().duration_since(self.last_command_time);
        if TIME_BTW_COMMANDS.min(time_since_last_command.as_secs_f64()) != TIME_BTW_COMMANDS {
            println!(
                "Command {} executed too soon, waiting {} seconds",
                command, TIME_BTW_COMMANDS
            );
            thread::sleep(Duration::from_secs(1));
        }

        // Insert time since command check

        let timestamp = Instant::now();

        self.socket
            .send_to(command.as_bytes(), TELLO_ADDR)
            .expect("Sending command failed");

        loop {
            let mut value = self.shared_response.lock().unwrap();

            if !value.is_none() {
                self.last_command_time = Instant::now();
                let temp = value.clone();
                let mut temp = temp.unwrap();
                temp = String::from(temp.trim_end_matches("\r\n"));
                *value = None;
                return Some(temp);
            }

            if Instant::now().duration_since(timestamp).as_secs() > timeout {
                println!("CONFUSED");
                // Timeout handling
                let temp = format!(
                    "Aborting command '{}'. Did not receive a response after {} seconds",
                    command, timeout
                );
                *value = None;
                return Some(temp);
            }

            println!("{}", Instant::now().duration_since(timestamp).as_millis());
        }
    }

    // Sends control command to Tello and waits for a response
    fn send_control_command(&mut self, command: &str, timeout: u64) -> bool {
        for _ in 0..self.retry_count {
            let response = self
                .send_command_with_return(command, timeout)
                .unwrap_or_else(|| String::from("Attempt failed, retrying"));

            if response.to_lowercase().contains("ok") {
                println!("{}", response);
                return true;
            } else {
                println!("{}", response);
            }

            // println!("tried {} times: {}", i, response);
        }

        // raise_result_error ?
        return false;
    }

    fn send_read_command(&mut self, command: &str) -> String {
        let response = self
            .send_command_with_return(command, RESPONSE_TIMEOUT)
            .unwrap();

        let error_words = ["error", "ERROR", "False"];
        if error_words.iter().any(|&word| response.contains(word)) {
            debug!("uh oh");
            error!("ruh roh");
        }

        return response;
    }

    // Send command to tello and wait for response, parses response into an integer
    fn send_read_command_int(&mut self, command: &str) -> i32 {
        self.send_read_command(command).parse::<i32>().unwrap()
    }

    // Send command to tello and wait for response, parses response into float
    fn send_read_command_float(&mut self, command: &str) -> f64 {
        self.send_read_command(command).parse::<f64>().unwrap()
    }

    fn raise_result_error(&self, command: &str, response: &str) {
        let tries = 1 + self.retry_count;
        // need some replacement for the raise python macro, easy enough will sort it soon
    }
}

// State field methods
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
                    debug!(
                        "Error parsing state value for {}: {} to {}",
                        key, value_str, e
                    );
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
            value_str
                .parse::<i32>()
                .map(StateValue::Int)
                .map_err(|e| e.to_string())
        } else if FLOAT_STATE_FIELDS.contains(&key) {
            value_str
                .parse::<f64>()
                .map(StateValue::Float)
                .map_err(|e| e.to_string())
        } else {
            Ok(StateValue::Str(value_str.to_string()))
        }
    }

    // Get a specific state field by name
    fn get_state_field(&self, key: &str) -> &StateValue {
        if !self.state.contains_key(&key.to_string()) {
            error!("Could not get state property: {}", key);
        }

        self.state.get(&key.to_string()).unwrap()
    }

    // Returns pitch in degree
    fn get_pitch(&self) -> &StateValue {
        self.get_state_field("pitch")
    }

    fn get_roll(&self) -> &StateValue {
        self.get_state_field("roll")
    }

    fn get_yaw(&self) -> &StateValue {
        self.get_state_field("yaw")
    }

    // Z axis speed
    fn get_speed_x(&self) -> &StateValue {
        self.get_state_field("vgx")
    }

    // Y axis speed
    fn get_speed_y(&self) -> &StateValue {
        self.get_state_field("vgy")
    }

    fn get_speed_z(&self) -> &StateValue {
        self.get_state_field("vgz")
    }

    // X axis acceleration
    fn get_acceleration_x(&self) -> &StateValue {
        self.get_state_field("agx")
    }

    // Y axis acceleration
    fn get_acceleration_y(&self) -> &StateValue {
        self.get_state_field("agy")
    }

    // Z axis acceleration
    fn get_acceleration_z(&self) -> &StateValue {
        self.get_state_field("agz")
    }

    // Get lowest temperature
    fn get_lowest_temperature(&self) -> &StateValue {
        self.get_state_field("templ")
    }

    // Get highest temperature
    fn get_highest_temperature(&self) -> &StateValue {
        self.get_state_field("temph")
    }

    // Get average temperature
    fn get_temperature(&self) -> i32 {
        let templ = self.get_lowest_temperature();
        let temph = self.get_highest_temperature();

        // Is this... a sin?
        // Change the type of templ and temph
        let templ = match templ {
            StateValue::Int(i) => *i,
            _ => panic!("Uh oh scoob"),
        };
        let temph = match temph {
            StateValue::Int(i) => *i,
            _ => panic!("Uh oh scoob"),
        };

        templ + temph
    }

    // Get current height in cm
    fn get_height(&self) -> i32 {
        match self.get_state_field("h") {
            StateValue::Int(i) => *i,
            _ => panic!("Uh oh"),
        }
    }

    // Get current distance value from TOF in cm
    fn get_distance_tof(&self) -> i32 {
        match self.get_state_field("tof") {
            StateValue::Int(i) => *i,
            _ => panic!("Uh oh"),
        }
    }

    // Get current barometer measurement in cm -> absolute height
    fn get_barometer(&self) -> i32 {
        match self.get_state_field("baro") {
            StateValue::Int(i) => *i * 100,
            _ => panic!("Uh oh"),
        }
    }

    // Get the time the motors have been active in seconds
    fn get_flight_time(&self) -> i32 {
        match self.get_state_field("time") {
            StateValue::Int(i) => *i,
            _ => panic!("Uh oh"),
        }
    }

    fn get_udp_video_address(&self) {
        // Placeholder, uses global variables in python implementation, will sort what this needs soon
    }

    fn get_frame_read(&self) {
        // Also placeholder for now
    }
}

// Public user command methods
impl Drone {
    pub fn connect(&mut self) {
        self.send_control_command("command", RESPONSE_TIMEOUT);

        let reps = 20;
        for i in 0..reps {
            if !self.state.is_empty() {
                // let t = i / reps;
                println!("trying");
                // Debug message
                break;
            }

            thread::sleep(Duration::from_secs_f64(1.0 / reps as f64));
        }

        if self.state.is_empty() {
            // raise error
        }
    }

    // Send a keepalive packet to keep the drone from landing after 15s
    pub fn send_keepalive(&mut self) {
        self.send_control_command("keepalive", RESPONSE_TIMEOUT);
    }

    pub fn turn_motor_on(&mut self) {
        self.send_control_command("motoron", RESPONSE_TIMEOUT);
    }

    pub fn turn_motor_off(&mut self) {
        self.send_control_command("motoroff", RESPONSE_TIMEOUT);
    }

    pub fn initiate_throw_takeoff(&mut self) {
        self.send_control_command("throwfly", RESPONSE_TIMEOUT);
    }

    pub fn takeoff(&mut self) {
        self.send_control_command("takeoff", TAKEOFF_TIMEOUT);
        self.is_flying = true;
    }

    pub fn land(&mut self) {
        self.send_control_command("land", RESPONSE_TIMEOUT);
        self.is_flying = false;
    }

    // pub fn streamon(&mut self) {

    // }
    pub fn rotate_clockwise(&mut self, distance: i32) {
        self.send_control_command(format!("cw {}", distance).as_str(), RESPONSE_TIMEOUT);
    }
}

#[cfg(test)]
mod tests {
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
