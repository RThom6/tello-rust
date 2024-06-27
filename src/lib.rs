use std::{
    net::UdpSocket, 
    time::Duration,
};

const TELLO_IP: &'static str = "0.0.0.0:8889";
const RESPONSE_TIMEOUT: u64 = 7;

pub struct Drone {
    socket: UdpSocket,
}

impl Drone {
    pub fn new() -> Drone {
        
        let socket = UdpSocket::bind(TELLO_IP)
            .expect("couldn't bind to address");
        
        socket.set_read_timeout(Some(Duration::from_secs(RESPONSE_TIMEOUT)))
            .expect("set_read_timeout call failed");
        socket.set_nonblocking(true).unwrap();
        socket.connect(TELLO_IP)
            .expect("connection failed");

        Drone {
            socket, 
        }
    } 
}

// pub fn connect() -> Result<(), Box<dyn Error>>{
//     let socket = UdpSocket::bind("192.168.10.1:8889")?;

//     socket.set_read_timeout(Some(Duration::from_secs(15))).expect("set_read_timeout call failed");

//     Ok(())
// }

#[cfg(test)]
mod tests {

}