use tello_rust::Drone;

fn main() {
    let mut drone = Drone::new();

    drone.connect();

    // drone.takeoff();

    // drone.rotate_clockwise(60);

    // drone.land();
}
