# robstride-rs

Rust driver for the **Robstride CAN servo motor family** (RS-00 / RS-01 / RS-02 / RS-03 / RS-04 / RS-05 / RS-06; the Edulite01/02/05 share the same protocol).

Initial development target is **Edulite05** (`MotorModel::Rs05`).

The workspace is split into three crates:

| Crate                | Purpose                                                                                  | `no_std` |
| -------------------- | ---------------------------------------------------------------------------------------- | -------- |
| `robstride-protocol` | Pure protocol layer — CAN ID build/parse, MIT codec, frame builders, response parsers.   | yes      |
| `robstride-driver`   | Synchronous driver wrapping `socketcan` for Linux. High-level `Motor` API + bus scanner. | no       |
| `robstride-cli`      | Test CLI binary (`status` / `enable` / `move-to` / `spin` / `mit` / `scan` / `dump`).    | no       |

The protocol crate performs no I/O and no allocation, so it can be reused on
embedded targets (STM32, RP2040, etc.) by pairing it with whatever CAN HAL the
project uses.

## Wire format

Every Robstride frame is a CAN extended (29-bit) frame with an 8-byte payload:

```
bits  28..24       23..8           7..0
     +---------+----------------+-----------+
     | comm(5) |  extra_data(16)| dev_id(8) |
     +---------+----------------+-----------+
```

For MIT-mode control the data payload is `[pos_u16, vel_u16, kp_u16, kd_u16]`
(big-endian) and the torque feedforward rides in `extra_data`.

Communication-type codes follow the published Robstride protocol — see
[`crates/robstride-protocol/src/comm_type.rs`](crates/robstride-protocol/src/comm_type.rs).

## Quick start (driver)

```rust
use robstride_driver::{Motor, MotorModel, RunMode};

let mut motor = Motor::open("can0", 1, MotorModel::Rs05)?;
motor.enable()?;
motor.set_run_mode(RunMode::Position)?;
motor.set_position_speed_limit(5.0)?;
motor.set_position(1.57)?;
let fb = motor.read_status()?;
println!("position = {:.3} rad", fb.position);
motor.disable()?;
# Ok::<(), robstride_driver::Error>(())
```

## Test CLI

Bring up SocketCAN (1 Mbps is the Robstride default), then run subcommands:

```bash
# scan ids 1..=32 on can0
cargo run -p robstride-cli -- -i can0 scan

# status of motor id 1, treating it as Edulite05
cargo run -p robstride-cli -- -i can0 -m 1 status

# move to 1.57 rad
cargo run -p robstride-cli -- -i can0 -m 1 move-to 1.57 --speed 2.0

# spin at 3 rad/s for 5 s
cargo run -p robstride-cli -- -i can0 -m 1 spin 3.0 --duration 5

# one-shot MIT command (returns to disabled afterwards)
cargo run -p robstride-cli -- -i can0 -m 1 mit --pos 0.0 --vel 0.0 --kp 50 --kd 1.0
```

`--model` accepts `RS-05`, `rs05`, `Edulite05`, etc. Default is `Edulite05`.

## Status

- [x] CAN ID encode/decode + MIT signed/unsigned codecs (with unit tests)
- [x] Frame builders (ping / enable / disable / set-zero / MIT / read-param / write-param-f32 / write-param-i8 / run-mode)
- [x] Status + parameter response parsers
- [x] Sync driver (`Motor`) with timeout-based receive loop
- [x] Bus scanner + passive bus dump
- [x] Test CLI binary (`robstride-cli`)
- [ ] async runtime adapter (tokio)
- [ ] Bus broadcast / multi-drop convenience layer
- [ ] Hardware soak test against real RS-05

The sandbox repo at `../robstride_sandbox/` remains the source of truth for
hardware-validated logic until that soak test runs against this crate.

## License

Copyright 2026 Takara Kasai

Licensed under the Apache License, Version 2.0 ([LICENSE](LICENSE) or
<http://www.apache.org/licenses/LICENSE-2.0>).
