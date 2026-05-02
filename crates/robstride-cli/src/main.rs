//! Test CLI for the `robstride-driver` crate.
//!
//! Examples:
//! ```text
//! robstride-cli -i can0 scan
//! robstride-cli -i can0 -m 1 status
//! robstride-cli -i can0 -m 1 enable
//! robstride-cli -i can0 -m 1 move-to 1.57
//! robstride-cli -i can0 -m 1 spin 3.0 --duration 5
//! robstride-cli -i can0 -m 1 mit --pos 0.0 --vel 0.0 --kp 50 --kd 1.0
//! ```

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use clap::{Parser, Subcommand};

use robstride_driver::{Motor, MotorModel, RunMode, dump_bus, scan_bus, DEFAULT_HOST_ID};

#[derive(Parser, Debug)]
#[command(version, about = "Test CLI for the Robstride CAN servo motor driver")]
struct Cli {
    /// SocketCAN interface name (e.g. can0).
    #[arg(short, long, default_value = "can0")]
    interface: String,

    /// Motor CAN ID (1..=127). Required for per-motor commands.
    #[arg(short, long, default_value_t = 1)]
    motor_id: u8,

    /// Host CAN ID (must be greater than every motor id on the bus).
    #[arg(long, default_value_t = DEFAULT_HOST_ID)]
    host_id: u8,

    /// Motor model — accepts `RS-05`, `rs05`, `Edulite05`, etc.
    #[arg(long, default_value = "Edulite05")]
    model: String,

    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand, Debug)]
enum Command {
    /// Read a single status frame.
    Status,
    /// Enable the motor (must be sent before motion commands).
    Enable,
    /// Disable the motor (coast).
    Disable,
    /// Set the current position as the mechanical zero.
    SetZero,
    /// Move to an absolute position (rad) using the position-control mode.
    MoveTo {
        /// Target position in radians.
        position: f32,
        /// Speed limit (rad/s).
        #[arg(long, default_value_t = 5.0)]
        speed: f32,
        /// Position tolerance to declare arrival (rad).
        #[arg(long, default_value_t = 0.05)]
        tolerance: f32,
        /// Timeout (s).
        #[arg(long, default_value_t = 10.0)]
        timeout: f32,
    },
    /// Continuous velocity command (rad/s).
    Spin {
        /// Velocity in rad/s (negative reverses).
        velocity: f32,
        /// Stop after this many seconds (omit for Ctrl-C).
        #[arg(long)]
        duration: Option<f32>,
    },
    /// Continuous torque command (Nm).
    Torque {
        torque: f32,
        #[arg(long)]
        duration: Option<f32>,
    },
    /// One-shot MIT-mode command.
    Mit {
        #[arg(long, default_value_t = 0.0)]
        pos: f32,
        #[arg(long, default_value_t = 0.0)]
        vel: f32,
        #[arg(long, default_value_t = 0.0)]
        kp: f32,
        #[arg(long, default_value_t = 0.0)]
        kd: f32,
        #[arg(long, default_value_t = 0.0)]
        torque: f32,
    },
    /// Probe each motor id in the range and print responders.
    Scan {
        #[arg(long, default_value_t = 1)]
        from: u8,
        #[arg(long, default_value_t = 32)]
        to: u8,
        /// Per-id timeout (ms).
        #[arg(long, default_value_t = 50)]
        timeout: u64,
    },
    /// Passively listen on the bus and print every frame.
    Dump {
        /// Listen duration (s).
        #[arg(long, default_value_t = 5.0)]
        duration: f32,
    },
    /// Periodically print position / velocity / torque.
    Monitor {
        /// Sampling interval (ms).
        #[arg(long, default_value_t = 100)]
        interval: u64,
    },
}

fn parse_model(s: &str) -> Result<MotorModel> {
    MotorModel::from_name(s).with_context(|| format!("unknown motor model: {s}"))
}

fn open_motor(cli: &Cli) -> Result<Motor> {
    let model = parse_model(&cli.model)?;
    Motor::open_with_host(&cli.interface, cli.motor_id, cli.host_id, model)
        .with_context(|| format!("failed to open {} for motor {}", cli.interface, cli.motor_id))
}

fn print_feedback(label: &str, fb: &robstride_driver::MotorFeedback) {
    println!(
        "{label}: motor={:>3} pos={:+.3} rad  vel={:+.3} rad/s  τ={:+.3} Nm  T={:.1}°C  mode={}",
        fb.motor_id, fb.position, fb.velocity, fb.torque, fb.temperature, fb.status.mode,
    );
    let s = &fb.status;
    if s.uncalibrated || s.stall || s.magnetic_encoder_fault || s.overtemperature || s.overcurrent || s.undervoltage {
        println!(
            "  flags:{}{}{}{}{}{}",
            if s.uncalibrated { " UNCAL" } else { "" },
            if s.stall { " STALL" } else { "" },
            if s.magnetic_encoder_fault { " MAG_FAULT" } else { "" },
            if s.overtemperature { " OVER_TEMP" } else { "" },
            if s.overcurrent { " OVER_CUR" } else { "" },
            if s.undervoltage { " UNDER_V" } else { "" },
        );
    }
}

fn install_ctrl_c(flag: Arc<AtomicBool>) {
    let _ = ctrlc::set_handler(move || flag.store(true, Ordering::SeqCst));
}

fn run(cli: Cli) -> Result<()> {
    match &cli.command {
        Command::Status => {
            let motor = open_motor(&cli)?;
            let fb = motor.read_status()?;
            print_feedback("status", &fb);
        }
        Command::Enable => {
            let mut motor = open_motor(&cli)?;
            let fb = motor.enable()?;
            print_feedback("enabled", &fb);
            // Caller likely wants the motor to stay enabled — leak the handle
            // so Drop doesn't auto-disable.
            std::mem::forget(motor);
        }
        Command::Disable => {
            let mut motor = open_motor(&cli)?;
            let fb = motor.disable()?;
            print_feedback("disabled", &fb);
        }
        Command::SetZero => {
            let mut motor = open_motor(&cli)?;
            motor.set_zero()?;
            println!("zero set on motor {}", cli.motor_id);
        }
        Command::MoveTo {
            position,
            speed,
            tolerance,
            timeout,
        } => {
            let mut motor = open_motor(&cli)?;
            motor.enable()?;
            motor.set_run_mode(RunMode::Position)?;
            motor.set_position_speed_limit(*speed)?;
            motor.set_position(*position)?;

            let stop = Arc::new(AtomicBool::new(false));
            install_ctrl_c(stop.clone());
            let start = Instant::now();
            loop {
                if stop.load(Ordering::SeqCst) {
                    println!("interrupted");
                    break;
                }
                if start.elapsed() > Duration::from_secs_f32(*timeout) {
                    println!("timed out before reaching target");
                    break;
                }
                let fb = motor.read_status()?;
                print_feedback("  ", &fb);
                if (fb.position - *position).abs() < *tolerance {
                    println!("arrived at {:.3} rad", fb.position);
                    break;
                }
                std::thread::sleep(Duration::from_millis(100));
            }
            motor.disable()?;
        }
        Command::Spin { velocity, duration } => {
            let mut motor = open_motor(&cli)?;
            motor.enable()?;
            motor.set_run_mode(RunMode::Velocity)?;
            motor.set_velocity(*velocity)?;
            println!("spinning at {velocity} rad/s — Ctrl-C to stop");

            let stop = Arc::new(AtomicBool::new(false));
            install_ctrl_c(stop.clone());
            let start = Instant::now();
            let max = duration.map(Duration::from_secs_f32);
            while !stop.load(Ordering::SeqCst) {
                if max.is_some_and(|d| start.elapsed() > d) {
                    break;
                }
                let fb = motor.read_status()?;
                print_feedback("  ", &fb);
                std::thread::sleep(Duration::from_millis(100));
            }
            motor.disable()?;
        }
        Command::Torque { torque, duration } => {
            let mut motor = open_motor(&cli)?;
            motor.enable()?;
            motor.set_run_mode(RunMode::Torque)?;
            motor.set_torque(*torque)?;
            println!("applying {torque} Nm — Ctrl-C to stop");

            let stop = Arc::new(AtomicBool::new(false));
            install_ctrl_c(stop.clone());
            let start = Instant::now();
            let max = duration.map(Duration::from_secs_f32);
            while !stop.load(Ordering::SeqCst) {
                if max.is_some_and(|d| start.elapsed() > d) {
                    break;
                }
                let fb = motor.read_status()?;
                print_feedback("  ", &fb);
                std::thread::sleep(Duration::from_millis(100));
            }
            motor.disable()?;
        }
        Command::Mit {
            pos,
            vel,
            kp,
            kd,
            torque,
        } => {
            let mut motor = open_motor(&cli)?;
            motor.enable()?;
            motor.set_run_mode(RunMode::Mit)?;
            let fb = motor.mit_control(*pos, *vel, *kp, *kd, *torque)?;
            print_feedback("mit", &fb);
            motor.disable()?;
        }
        Command::Scan { from, to, timeout } => {
            let timeout_per_id = Duration::from_millis(*timeout);
            let mut last_id: u8 = 0;
            let results = {
                let mut cb = |idx: usize, total: usize, motor_id: u8| {
                    if motor_id != 0 && motor_id != last_id {
                        last_id = motor_id;
                        eprint!("\rprobing {idx}/{total} (id={motor_id})   ");
                    }
                };
                scan_bus(
                    &cli.interface,
                    cli.host_id,
                    *from..=*to,
                    timeout_per_id,
                    Some(&mut cb),
                )?
            };
            eprintln!();
            if results.is_empty() {
                println!("no motors found");
            } else {
                println!("found {} motor(s):", results.len());
                for r in results {
                    print!("  id={:<3} payload=", r.motor_id);
                    for b in &r.payload {
                        print!("{:02X} ", b);
                    }
                    println!();
                }
            }
        }
        Command::Dump { duration } => {
            let frames = dump_bus(&cli.interface, Duration::from_secs_f32(*duration))?;
            println!("captured {} frame(s)", frames.len());
            for (id, data) in frames {
                print!("  0x{:08X} ", id);
                for b in &data {
                    print!("{:02X} ", b);
                }
                println!();
            }
        }
        Command::Monitor { interval } => {
            let motor = open_motor(&cli)?;
            let stop = Arc::new(AtomicBool::new(false));
            install_ctrl_c(stop.clone());
            let dt = Duration::from_millis(*interval);
            while !stop.load(Ordering::SeqCst) {
                match motor.read_status() {
                    Ok(fb) => print_feedback("  ", &fb),
                    Err(e) => eprintln!("read error: {e}"),
                }
                std::thread::sleep(dt);
            }
        }
    }
    Ok(())
}

fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    run(Cli::parse())
}
