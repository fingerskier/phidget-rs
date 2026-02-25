// phidget-rs/examples/dc_motor.rs
//
// Copyright (c) 2023-2025, Frank Pagliughi
//
// This file is an example application for the 'phidget-rs' library.
//
// Licensed under the MIT license:
//   <LICENSE or http://opensource.org/licenses/MIT>
// This file may not be copied, modified, or distributed except according
// to those terms.
//

//! Rust Phidget example application for DC Motor control (DCC1000_0).
//!
use clap::{arg, value_parser, ArgAction};
use phidget::Phidget;
use std::thread;

// The package version is used as the app version
const VERSION: &str = env!("CARGO_PKG_VERSION");

// --------------------------------------------------------------------------

fn main() -> anyhow::Result<()> {
    let opts = clap::Command::new("dc_motor")
        .version(VERSION)
        .author(env!("CARGO_PKG_AUTHORS"))
        .about("Phidget DC Motor Example")
        .disable_help_flag(true)
        .arg(
            arg!(--help "Print help information")
                .short('?')
                .action(ArgAction::Help),
        )
        .arg(
            arg!(-s --serial [serial] "Specify the serial number of the device to open")
                .value_parser(value_parser!(i32)),
        )
        .arg(
            arg!(-c --channel [channel] "Specify the channel number of the device to open")
                .value_parser(value_parser!(i32)),
        )
        .arg(
            arg!(-p --port [port] "Use a specific port on a VINT hub directly")
                .value_parser(value_parser!(i32)),
        )
        .arg(
            arg!(-v --velocity [velocity] "Set target velocity (-1.0 to 1.0)")
                .value_parser(value_parser!(f64)),
        )
        .arg(arg!(-h --hub "Use a hub VINT input port directly").action(ArgAction::SetTrue))
        .get_matches();

    let use_hub = opts.get_flag("hub");

    println!("Opening Phidget DC motor device...");
    let mut motor = phidget::DcMotor::new();

    // Whether we should use a hub port directly as the input,
    // and if so, which one?
    motor.set_is_hub_port_device(use_hub)?;
    if let Some(&port) = opts.get_one::<i32>("port") {
        motor.set_hub_port(port)?;
    }

    // Some other device selection filters...
    if let Some(&serial) = opts.get_one::<i32>("serial") {
        motor.set_serial_number(serial)?;
    }

    if let Some(&channel) = opts.get_one::<i32>("channel") {
        motor.set_channel(channel)?;
    }

    let target_velocity = opts.get_one::<f64>("velocity").copied().unwrap_or(0.0);

    motor.set_on_velocity_update_handler(|_, velocity: f64| {
        println!("Velocity: {:.4}", velocity);
    })?;

    motor.open_wait_default()?;

    let port = motor.hub_port()?;
    println!("Opened on hub port: {}", port);

    println!("Setting target velocity to: {:.4}", target_velocity);
    motor.set_target_velocity(target_velocity)?;

    // ^C handler wakes up the main thread
    ctrlc::set_handler({
        let thr = thread::current();
        move || {
            println!("\nExiting...");
            thr.unpark();
        }
    })
    .expect("Error setting Ctrl-C handler");

    // Block until a ^C wakes us up to exit.
    thread::park();
    Ok(())
}
