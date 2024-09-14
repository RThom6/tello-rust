# Ryze Tello Drone Rust Wrapper

## A Rust wrapper for interacting with the Ryze Tello drone, using the official Tello API. This library is inspired by the DJITelloPy Python library, offering similar functionality but in the Rust programming language.
### Features

    Control the Ryze Tello drone using Rust.
    Send commands to the drone, like takeoff, land, flip, and more.
    Receive state information from the drone in real time.
    Basic support for the Tello SDK 1.3 commands (no support for EDU-only commands as of now).

Note: This wrapper currently only supports controlling one drone at a time.
### Tello SDK Documentation

    Tello SDK 1.3
    Tello SDK 2.0 (including EDU-only commands)

### Installation

You can add this crate to your project by running the command:

    cargo add tell-rust

Or by adding

    tello-rust = "1.0.0"

to your cargo.toml file

### Usage

Here is a basic example of how to use the library to control the Tello drone:

### rust

    use ryze_tello::Tello;
    
    fn main() {
        let mut drone = Tello::new();
    
        drone.takeoff().expect("Failed to takeoff");
        drone.flip("l").expect("Failed to flip left");
        drone.land().expect("Failed to land");
    }

Supported Commands

    Takeoff: takeoff()
    Land: land()
    Move: move(x, y, z, speed)
    Flip: flip(direction)
    And more...

Limitations

    This wrapper does not support the EDU-only commands from SDK 2.0 yet, but this might be added in future updates.
    Only one drone can be controlled at a time.

Contributing

Contributions are welcome! If you find a bug or have a feature request, please open an issue or submit a pull request on GitHub.
License

This project is licensed under the MIT License - see the LICENSE file for details.
