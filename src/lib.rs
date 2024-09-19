// Rust wrapper to interact with the Ryze Tello drone using the official Tello api
// By Ryan Thomas: https://github.com/RThom6
// Essentially a translation of DJITelloPy into Rust: https://github.com/damiafuentes/DJITelloPy/tree/master#
// Note: This wrapper only supports one drone at a time
//
// Tello API documentation:
// [1.3](https://dl-cdn.ryzerobotics.com/downloads/tello/20180910/Tello%20SDK%20Documentation%20EN_1.3.pdf),
// This Wrapper does not support EDU-only commands as of 01/08/2024, this may change in future
// [2.0 with EDU-only commands](https://dl-cdn.ryzerobotics.com/downloads/Tello/Tello%20SDK%202.0%20User%20Guide.pdf)

use log::{debug, error};
use std::{
    collections::HashMap,
    net::UdpSocket,
    sync::{Arc, Mutex},
    thread,
    time::{Duration, Instant},
};

const TELLO_ADDR: &'static str = "192.168.10.1:8889";
const TELLO_STATE_ADDR: &'static str = "0.0.0.0:8890";
const RESPONSE_TIMEOUT: u64 = 7; // Seconds
const TAKEOFF_TIMEOUT: u64 = 20; // Seconds
const TIME_BTW_COMMANDS: f64 = 0.1; // Seconds

// State field types: states not named here are all strings
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
    shared_state: Arc<Mutex<HashMap<String, StateValue>>>, //State hashmap, closest thing to python dict
    read_state: HashMap<String, StateValue>,
}

#[derive(Clone, Debug, PartialEq)]
#[allow(dead_code)]
enum StateValue {
    Int(i32),
    Float(f64),
    Str(String),
}

fn parse_state(state_str: &str) -> Option<HashMap<String, StateValue>> {
    let state_str = state_str.trim();

    if state_str.eq("ok") {
        return None;
    }

    let mut state_map: HashMap<String, StateValue> = HashMap::new();

    for field in state_str.split(';') {
        let split: Vec<&str> = field.split(':').collect();

        if split.len() < 2 {
            continue;
        }

        let key = split[0].to_string();
        let value_str = split[1];
        let value: StateValue = match state_field_converter(&key, value_str) {
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

        state_map.insert(key, value);
    }

    return Some(state_map);
}

/// Converts fields to the correct type based on field key
fn state_field_converter(key: &str, value_str: &str) -> Result<StateValue, String> {
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

/// State receiver thread to run in background\n
/// Private method ran when connected to drone, should kill program
/// before trying connect again or will end up with many background threads
fn start_state_receiver_thread(response_receiver: Arc<Mutex<HashMap<String, StateValue>>>) {
    thread::spawn(move || {
        let socket = UdpSocket::bind(TELLO_STATE_ADDR).expect("Couldn't bind receiver socket");
        let mut buf = [0; 1024];

        loop {
            let (amt, _src) = socket.recv_from(&mut buf).expect("Didn't receive message");
            let received = String::from_utf8_lossy(&buf[..amt]);

            let state_map = match parse_state(&received) {
                Some(map) => map,
                None => continue,
            };

            let mut value = response_receiver.lock().unwrap();
            *value = state_map;
        }
    });
}

impl Drone {
    /// Instantiate a new Drone object
    pub fn new() -> Self {
        let socket = UdpSocket::bind("0.0.0.0:8889")
            .expect(format!("couldn't bind to address {}", TELLO_ADDR).as_str());

        socket
            .set_read_timeout(Some(Duration::from_secs(RESPONSE_TIMEOUT)))
            .expect("set_read_timeout call failed");

        let socket_recv = socket.try_clone().unwrap(); // Cloned socket to have multiple of same socket

        // Shared response variable protected by mutex
        let shared_response = Arc::new(Mutex::new(None::<String>));
        let response_receiver = Arc::clone(&shared_response);

        // Receiver thread, listens for response from drone on command port
        thread::spawn(move || {
            let socket = socket_recv;
            let mut buf = [0; 1024];

            loop {
                match socket.recv_from(&mut buf) {
                    Ok((amt, src)) => {
                        if src.to_string() != TELLO_ADDR {
                            println!("{}", src.to_string());
                            continue;
                        }
                        let received = String::from_utf8_lossy(&buf[..amt]);
                        if let Ok(mut value) = response_receiver.lock() {
                            *value = Some(received.to_string()); // Save value to shared variable
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

        Drone {
            socket,
            is_flying: false,
            stream_on: false,
            retry_count: 3,
            last_command_time: Instant::now(),
            shared_response,
            shared_state: Arc::new(Mutex::new(HashMap::new())),
            read_state: HashMap::new(),
        }
    }
}

/**
 * Private command methods for API
 */
impl Drone {
    /// Send a command without waiting for a return\n
    /// Private Method called by library
    fn send_command_without_return(&self, command: &str) {
        self.socket
            .send_to(command.as_bytes(), TELLO_ADDR)
            .expect("Sending command failed");

        println!("Send Command {}", command);
    }

    /// Send command and wait for a return\n
    /// Private method called by library
    fn send_command_with_return(&mut self, command: &str, timeout: u64) -> Option<String> {
        let time_since_last_command = Instant::now().duration_since(self.last_command_time);
        if TIME_BTW_COMMANDS.min(time_since_last_command.as_secs_f64()) != TIME_BTW_COMMANDS {
            println!(
                "Command {} executed too soon, waiting {} seconds",
                command, TIME_BTW_COMMANDS
            );
            thread::sleep(Duration::from_secs_f64(TIME_BTW_COMMANDS));
        }

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

            if Instant::now().duration_since(timestamp).as_secs() >= timeout {
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

    /// Sends control command to Tello and waits for a response
    fn send_control_command(&mut self, command: &str, timeout: u64) -> bool {
        for i in 0..self.retry_count {
            let response = self
                .send_command_with_return(command, timeout)
                .unwrap_or_else(|| String::from("Attempt failed, retrying"));

            if response.to_lowercase().contains("ok") {
                println!("{}", response);
                return true;
            } else {
                println!("{}", response);
            }

            println!("tried {} times: {}", i, response);
        }

        return false;
    }

    /// Private function to handle sending and parsing read commands
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

    /// Send command to tello and wait for response, parses response into an integer
    fn send_read_command_int(&mut self, command: &str) -> i32 {
        self.send_read_command(command).parse::<i32>().unwrap()
    }

    /// Send command to tello and wait for response, parses response into float
    fn send_read_command_float(&mut self, command: &str) -> f64 {
        self.send_read_command(command).parse::<f64>().unwrap()
    }
}

// State field methods
impl Drone {
    /// Get a specific state field by name
    fn get_state_field(&mut self, key: &str) -> &StateValue {
        // This may be sacreligious
        // Clones the value of the r/w state variable then the lock is released
        self.read_state = self.shared_state.lock().unwrap().clone();

        // Checks whether our new state has the value sought after
        if !self.read_state.contains_key(&key.to_string()) {
            error!("Could not get state property: {}", key);
        }
        // Returns a reference to this value stored in read state
        self.read_state.get(&key.to_string()).unwrap()
    }

    /// Returns pitch in degree
    pub fn get_pitch(&mut self) -> i32 {
        match self.get_state_field("pitch") {
            StateValue::Int(i) => *i,
            _ => panic!("'pitch' state returned the incorrect Type"),
        }
    }

    /// Returns roll in degree
    pub fn get_roll(&mut self) -> i32 {
        match self.get_state_field("roll") {
            StateValue::Int(i) => *i,
            _ => panic!("'roll' state returned the incorrect Type"),
        }
    }

    /// Returns yaw in degree
    pub fn get_yaw(&mut self) -> i32 {
        match self.get_state_field("yaw") {
            StateValue::Int(i) => *i,
            _ => panic!("'yaw' state returned the incorrect Type"),
        }
    }

    /// X axis speed
    pub fn get_speed_x(&mut self) -> i32 {
        match self.get_state_field("vgx") {
            StateValue::Int(i) => *i,
            _ => panic!("'soeed_x' state returned the incorrect Type"),
        }
    }

    /// Y axis speed
    pub fn get_speed_y(&mut self) -> i32 {
        match self.get_state_field("vgy") {
            StateValue::Int(i) => *i,
            _ => panic!("'speed_y' state returned the incorrect Type"),
        }
    }

    /// Z axis speed
    pub fn get_speed_z(&mut self) -> i32 {
        match self.get_state_field("vgz") {
            StateValue::Int(i) => *i,
            _ => panic!("'speed_z' state returned the incorrect Type"),
        }
    }

    /// X axis acceleration
    pub fn get_acceleration_x(&mut self) -> f64 {
        match self.get_state_field("agx") {
            StateValue::Float(i) => *i,
            _ => panic!("'accel_x' state returned the incorrect Type"),
        }
    }

    /// Y axis acceleration
    pub fn get_acceleration_y(&mut self) -> f64 {
        match self.get_state_field("agy") {
            StateValue::Float(i) => *i,
            _ => panic!("'accel_y' state returned the incorrect Type"),
        }
    }

    /// Z axis acceleration
    pub fn get_acceleration_z(&mut self) -> f64 {
        match self.get_state_field("agz") {
            StateValue::Float(i) => *i,
            _ => panic!("'accel_z' state returned the incorrect Type"),
        }
    }

    /// Get lowest temperature
    pub fn get_lowest_temperature(&mut self) -> i32 {
        match self.get_state_field("templ") {
            StateValue::Int(i) => *i,
            _ => panic!("'lowest_temperature' state returned the incorrect Type"),
        }
    }

    /// Get highest temperature
    pub fn get_highest_temperature(&mut self) -> i32 {
        match self.get_state_field("temph") {
            StateValue::Int(i) => *i,
            _ => panic!("'highest_temperature' state returned the incorrect Type"),
        }
    }

    /// Get average temperature
    pub fn get_temperature(&mut self) -> i32 {
        let templ = self.get_lowest_temperature();
        let temph = self.get_highest_temperature();

        templ + temph
    }

    /// Get current height in cm
    pub fn get_height(&mut self) -> i32 {
        match self.get_state_field("h") {
            StateValue::Int(i) => *i,
            _ => panic!("'height' state returned the incorrect Type"),
        }
    }

    /// Get current distance value from TOF in cm
    pub fn get_distance_tof(&mut self) -> i32 {
        match self.get_state_field("tof") {
            StateValue::Int(i) => *i,
            _ => panic!("'distance_tof' state returned the incorrect Type"),
        }
    }

    /// Get current barometer measurement in cm -> absolute height
    pub fn get_barometer(&mut self) -> f64 {
        match self.get_state_field("baro") {
            StateValue::Float(i) => *i * 100.0,
            _ => panic!("'baro' state returned the incorrect Type"),
        }
    }

    /// Get the time the motors have been active in seconds
    pub fn get_flight_time(&mut self) -> i32 {
        match self.get_state_field("time") {
            StateValue::Int(i) => *i,
            _ => panic!("'time' state returned the incorrect Type"),
        }
    }

    pub fn get_battery(&mut self) -> i32 {
        match self.get_state_field("bat") {
            StateValue::Int(i) => *i,
            _ => panic!("'bat' state returned the incorrect Type"),
        }
    }
}

// Public user command methods
impl Drone {
    pub fn connect(&mut self) {
        // Shared state response variable protected by mutex
        let state_response = Arc::new(Mutex::new(HashMap::new()));
        let state_receiver = Arc::clone(&state_response);
        start_state_receiver_thread(state_receiver);
        self.send_control_command("command", RESPONSE_TIMEOUT);

        let reps = 20;
        for _ in 0..reps {
            // Make a method to clone shared state into read state?
            {
                self.read_state = self.shared_state.lock().unwrap().clone();
            }

            if !self.read_state.is_empty() {
                // let t = i / reps;
                println!("trying");
                // Debug message
                break;
            }

            thread::sleep(Duration::from_secs_f64(1.0 / reps as f64));
        }

        if self.read_state.is_empty() {
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

    // Turn on video streaming
    pub fn streamon(&mut self) {
        // UDP port magic need to implement
        self.send_control_command("streamon", RESPONSE_TIMEOUT);
        self.stream_on = true;
    }

    // Turn off video streaming
    pub fn streamoff(&mut self) {
        self.send_control_command("streamoff", RESPONSE_TIMEOUT);
        self.stream_on = false;

        // NEED TO IMPLEMENT BACKGROUND FRAME READ
        // if self.background_frame_read != None {
        //     self.background_frame_read.stop();
        //     self.background_frame_read = None;
        // }
    }

    // Stop all motors immediately
    pub fn emergency(&mut self) {
        self.send_command_without_return("emergency");
        self.is_flying = false;
    }

    /// Move in any chosen direction\n
    /// direction: up, down, left, right, forward or back\n
    /// x: 20-500
    pub fn move_any(&mut self, direction: &str, x: i32) {
        self.send_control_command(format!("{} {}", direction, x).as_str(), RESPONSE_TIMEOUT);
    }

    /// Move up
    /// x: 20-500 in cm
    pub fn move_up(&mut self, x: i32) {
        self.move_any("up", x);
    }

    /// Move down
    /// x: 20-500 in cm
    pub fn move_down(&mut self, x: i32) {
        self.move_any("down", x);
    }

    /// Move left
    /// x: 20-500 in cm
    pub fn move_left(&mut self, x: i32) {
        self.move_any("left", x);
    }

    /// Move right
    /// x: 20-500 in cm
    pub fn move_right(&mut self, x: i32) {
        self.move_any("right", x);
    }

    /// Move forward
    /// x: 20-500 in cm
    pub fn move_forward(&mut self, x: i32) {
        self.move_any("forward", x);
    }

    /// Move back
    /// x: 20-500 in cm
    pub fn move_back(&mut self, x: i32) {
        self.move_any("back", x);
    }

    /// Rotate x degree clockwise
    pub fn rotate_clockwise(&mut self, x: i32) {
        self.send_control_command(&format!("cw {}", x), RESPONSE_TIMEOUT);
    }

    /// Rotate x degree counter-clockwise
    pub fn rotate_counter_clockwise(&mut self, x: i32) {
        self.send_control_command(&format!("ccw {}", x), RESPONSE_TIMEOUT);
    }

    /// Do a flip maneuver, helper for flip_x functions
    pub fn flip(&mut self, direction: &str) {
        self.send_control_command(&format!("flip {}", direction), RESPONSE_TIMEOUT);
    }

    /// Flip left
    pub fn flip_left(&mut self) {
        self.flip("l");
    }

    /// Flip right
    pub fn flip_right(&mut self) {
        self.flip("r");
    }

    /// Flip forward
    pub fn flip_forward(&mut self) {
        self.flip("f");
    }

    /// Flip backwards
    pub fn flip_back(&mut self) {
        self.flip("b");
    }

    /// Fly to xyz relative to current position\n
    /// Speed in cm/s\n
    /// Argument ranges:\n
    /// x: -500 - 500\n
    /// y: -500 - 500\n
    /// z: -500 - 500\n
    /// speed 10 - 100
    pub fn go_xyz_speed(&mut self, x: i32, y: i32, z: i32, speed: i32) {
        let cmd = format!("go {} {} {} {}", x, y, z, speed);
        self.send_control_command(&cmd, RESPONSE_TIMEOUT);
    }

    /// Fly to x2 y2 z2 in a curve via x1 y1 z1, speed is travelling speed in cm/s\n
    /// Both points relative to current position\n
    /// Argument value ranges:\n
    /// x1: -500 - 500\n
    /// y1: -500 - 500\n
    /// z1: -500 - 500\n
    /// x2: -500 - 500\n
    /// y2: -500 - 500\n
    /// z2: -500 - 500\n
    /// speed: 10 - 60
    pub fn curve_xyz_speed(
        &mut self,
        x1: i32,
        y1: i32,
        z1: i32,
        x2: i32,
        y2: i32,
        z2: i32,
        speed: i32,
    ) {
        let cmd = format!("curve {} {} {} {} {} {} {}", x1, y1, z1, x2, y2, z2, speed);
        self.send_control_command(&cmd, RESPONSE_TIMEOUT);
    }

    /// Set speed in cm/s
    pub fn set_speed(&mut self, speed: i32) {
        let cmd = format!("speed {}", speed);
        self.send_control_command(&cmd, RESPONSE_TIMEOUT);
    }

    /// Send RC control via four channels. Command is sent every self.TIME_BTW_RC_CONTROL_COMMANDS seconds.
    /// Argument parameters:
    /// left_right_velocity: -100~100 (left/right)
    /// forward_backward_velocity: -100~100 (forward/backward)
    /// up_down_velocity: -100~100 (up/down)
    /// yaw_velocity: -100~100 (yaw)
    pub fn send_rc_control(
        &mut self,
        left_right_velocity: i32,
        forward_backward_velocity: i32,
        up_down_velocity: i32,
        yaw_velocity: i32,
    ) {
        let clamp100 = |x: i32| -> i32 { x.max(-100).min(100) };

        if Instant::now()
            .duration_since(self.last_command_time)
            .as_secs_f64()
            > TIME_BTW_COMMANDS
        {
            self.last_command_time = Instant::now();
            let cmd = format!(
                "rc {} {} {} {}",
                clamp100(left_right_velocity),
                clamp100(forward_backward_velocity),
                clamp100(up_down_velocity),
                clamp100(yaw_velocity)
            );
            self.send_command_without_return(&cmd);
        }
    }

    /// Set the WiFi SSID and Password for the Tello, will cause a reboot
    pub fn set_wifi_credentials(&mut self, ssid: &str, password: &str) {
        let cmd = format!("wifi {} {}", ssid, password);
        self.send_control_command(&cmd, RESPONSE_TIMEOUT);
    }

    /// Only works on Tello-EDU
    /// Connects to a WiFi with the SSID and Password
    pub fn connect_to_wifi(&mut self, ssid: &str, password: &str) {
        let cmd = format!("ap {} {}", ssid, password);
        self.send_control_command(&cmd, RESPONSE_TIMEOUT);
    }

    /// Can change the ports of the Tello drone's state and video packets
    /// Not supported in this library
    pub fn set_network_ports(&mut self, state_packet_port: i32, video_stream_port: i32) {
        let cmd = format!("port {} {}", state_packet_port, video_stream_port);
        self.send_control_command(&cmd, RESPONSE_TIMEOUT);
    }

    /// Reboots the drone
    pub fn reboot(&mut self) {
        self.send_command_without_return("reboot");
    }

    /// Sets the bitrate of the video stream\n
    /// Only supports MBPS as i32 in the range of [0, 5] where 0 is auto
    pub fn set_video_bitrate(&mut self, bitrate: i32) {
        let cmd = format!("setbitrate {}", bitrate);
        self.send_control_command(&cmd, RESPONSE_TIMEOUT);
    }

    /// Sets the resolution of the video stream\n
    /// 'low' for 480P\n
    /// 'high' for 720P\n
    pub fn set_video_resolution(&mut self, resolution: &str) {
        let cmd = format!("setresolution {}", resolution);
        self.send_control_command(&cmd, RESPONSE_TIMEOUT);
    }

    /// Set the frames per second of the video stream\n
    /// 'low' for 5 fps\n
    /// 'middle' for 15 fps\n
    /// 'high' for 30 fps\n
    pub fn set_video_fps(&mut self, fps: &str) {
        let cmd = format!("setfps {}", fps);
        self.send_control_command(&cmd, RESPONSE_TIMEOUT);
    }

    /// Selects one of 2 cameras for streaming\n
    /// Forward camera is the regular 1080x720 colour camera\n
    /// Downward camera is a grey-only 320x240 IR-sensitive camera\n
    pub fn set_video_direction(&mut self, direction: i32) {
        let cmd = format!("downvision {}", direction);
        self.send_control_command(&cmd, RESPONSE_TIMEOUT);
    }
}

// MISSION PAD METHODS
impl Drone {
    /// Fly to xyz relative to mission pad with id: mid\n
    /// Speed in cm/s\n
    /// Argument value ranges:\n
    /// x: -500 - 500\n
    /// y: -500 - 500\n
    /// z: -500 - 500\n
    /// mid: 1 - 8
    pub fn go_xyz_speed_mid(&mut self, x: i32, y: i32, z: i32, mid: i32) {
        let cmd = format!("go {} {} {} m{}", x, y, z, mid);
        self.send_control_command(&cmd, RESPONSE_TIMEOUT);
    }

    /// Fly to x2 y2 z2 in a curve via x1 y1 z1, speed is travelling speed in cm/s\n
    /// Both points relative to mission pad with id: mid\n
    /// Argument value ranges:\n
    /// x1: -500 - 500\n
    /// y1: -500 - 500\n
    /// z1: -500 - 500\n
    /// x2: -500 - 500\n
    /// y2: -500 - 500\n
    /// z2: -500 - 500\n
    /// speed: 10 - 60
    pub fn curve_xyz_speed_mid(
        &mut self,
        x1: i32,
        y1: i32,
        z1: i32,
        x2: i32,
        y2: i32,
        z2: i32,
        speed: i32,
        mid: i32,
    ) {
        let cmd = format!(
            "curve {} {} {} {} {} {} {} m{}",
            x1, y1, z1, x2, y2, z2, speed, mid
        );
        self.send_control_command(&cmd, RESPONSE_TIMEOUT);
    }

    pub fn enable_mission_pads(&mut self) {
        self.send_control_command("mon", RESPONSE_TIMEOUT);
    }

    pub fn disable_mission_pads(&mut self) {
        self.send_control_command("moff", RESPONSE_TIMEOUT);
    }

    pub fn set_mission_pad_detection_direction(&mut self, direction: i32) {
        let cmd = format!("mdirection {}", direction);
        self.send_control_command(&cmd, RESPONSE_TIMEOUT);
    }
}

// QUERY COMMANDS, USUALLY SLOWER THAN GETTERS
impl Drone {
    pub fn query_speed(&mut self) -> i32 {
        self.send_read_command_int("speed?")
    }

    pub fn query_battery(&mut self) -> i32 {
        self.send_read_command_int("battery?")
    }

    pub fn query_flight_time(&mut self) -> i32 {
        self.send_read_command_int("time?")
    }

    pub fn query_height(&mut self) -> i32 {
        self.send_read_command_int("height?")
    }

    pub fn query_temperature(&mut self) -> i32 {
        self.send_read_command_int("temp?")
    }

    /// Query IMU attitude data\n
    /// Using get_pitch, get_roll and get_yaw is usually faster\n
    /// Returns Map with {'pitch': i32, 'roll': int, 'yaw': i32}
    pub fn query_attitude(&mut self) -> HashMap<String, i32> {
        let response = self.send_read_command("attitude?");

        let mut attitude: HashMap<String, i32> = HashMap::new();

        for field in response.split(';') {
            let split: Vec<&str> = field.split(':').collect();

            if split.len() < 2 {
                continue;
            }

            let key = split[0].to_string();
            let value_str = split[1];
            let value: StateValue = match state_field_converter(&key, value_str) {
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

            let value_i32 = match value {
                StateValue::Int(i) => i,
                _ => panic!("'bat' state returned the incorrect Type"),
            };

            attitude.insert(key, value_i32);
        }

        return attitude;
    }

    pub fn query_barometer(&mut self) -> f64 {
        self.send_read_command_float("baro?") * 100.0
    }

    pub fn query_distance_tof(&mut self) -> f64 {
        let tof = self.send_read_command("tof?");
        tof.trim_end_matches("mm").parse::<f64>().unwrap_or(0.0) / 10.0
    }

    pub fn query_wifi_signal_noise_ratio(&mut self) -> String {
        self.send_read_command("wifi?")
    }

    pub fn query_sdk_version(&mut self) -> String {
        self.send_read_command("sdk?")
    }

    pub fn query_serial_number(&mut self) -> String {
        self.send_read_command("sn?")
    }

    pub fn query_active(&mut self) -> String {
        self.send_read_command("active?")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // #[test]
    // fn system_time_test() {
    //     let socket = UdpSocket::bind(TELLO_ADDR).expect("couldn't bind to address");
    //     let shared_response = Arc::new(Mutex::new(None::<String>));

    //     let state_response = Arc::new(Mutex::new(HashMap::new()));
    //     let shared_state = Arc::clone(&state_response);

    //     let mut d = Drone {
    //         socket,
    //         is_flying: false,
    //         stream_on: false,
    //         retry_count: 3,
    //         last_command_time: Instant::now(),
    //         shared_response,
    //         shared_state
    //         read_state: HashMap::new(),
    //     };

    //     d.send_command_with_return("a", 6);
    // }

    #[test]
    fn test_parse_state_valid_input() {
        let input = "mid:1;x:2;y:3;z:4;pitch:0;roll:0;yaw:0;vgx:0;vgy:0;vgz:0;templ:50;temph:60;tof:100;h:5;bat:85;time:300;";
        let result = parse_state(input).unwrap();

        assert_eq!(result.get("mid"), Some(&StateValue::Int(1)));
        assert_eq!(result.get("x"), Some(&StateValue::Int(2)));
        assert_eq!(result.get("y"), Some(&StateValue::Int(3)));
        assert_eq!(result.get("bat"), Some(&StateValue::Int(85)));
        assert_eq!(result.get("templ"), Some(&StateValue::Int(50)));
    }

    #[test]
    fn test_parse_state_empty_input() {
        let input = "";
        let result = parse_state(input);

        assert_eq!(result, None);
    }

    #[test]
    fn test_state_field_converter_int() {
        let key = "x";
        let value_str = "10";
        let result = state_field_converter(key, value_str).unwrap();

        assert_eq!(result, StateValue::Int(10));
    }

    #[test]
    fn test_state_field_converter_float() {
        let key = "baro";
        let value_str = "99.99";
        let result = state_field_converter(key, value_str).unwrap();

        assert_eq!(result, StateValue::Float(99.99));
    }

    #[test]
    fn test_state_field_converter_invalid_input() {
        let key = "x";
        let value_str = "invalid";
        let result = state_field_converter(key, value_str);

        assert!(result.is_err());
    }
}
