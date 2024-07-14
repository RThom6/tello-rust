use tello_rust::Drone;

fn main() {
    let mut drone = Drone::new();

    drone.connect();
}
