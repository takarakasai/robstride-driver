//! Bus scanning and passive monitoring helpers.

use std::io;
use std::ops::RangeInclusive;
use std::time::{Duration, Instant};

use socketcan::{CanSocket, EmbeddedFrame, ExtendedId, Id, Socket, StandardId};

use robstride_protocol::{build_ping_frame, parse_can_id};

use crate::error::{Error, Result};

/// Result of probing a single motor id.
#[derive(Debug, Clone)]
pub struct ScanResult {
    pub motor_id: u8,
    /// 8-byte response payload (typically the MCU UUID for Robstride motors).
    pub payload: Vec<u8>,
}

/// Progress callback signature for [`scan_bus`]. Reports `(index, total, motor_id)`.
pub type ScanProgress<'a> = &'a mut dyn FnMut(usize, usize, u8);

/// Scan a CAN bus for motors that respond to `GET_DEVICE_ID`.
///
/// Sends a ping to each motor id in `id_range` and collects the responses.
/// `on_progress` (if provided) is called once per probe so a CLI/UI can show
/// progress.
pub fn scan_bus(
    interface: &str,
    host_id: u8,
    id_range: RangeInclusive<u8>,
    timeout_per_id: Duration,
    mut on_progress: Option<ScanProgress<'_>>,
) -> Result<Vec<ScanResult>> {
    let socket = CanSocket::open(interface)?;
    socket.set_read_timeout(timeout_per_id)?;

    let ids: Vec<u8> = id_range.collect();
    let total = ids.len();
    let mut found = Vec::new();

    for (idx, &motor_id) in ids.iter().enumerate() {
        if let Some(cb) = on_progress.as_mut() {
            cb(idx, total, motor_id);
        }

        let (can_id, data) = build_ping_frame(host_id, motor_id);
        let ext_id = match ExtendedId::new(can_id) {
            Some(id) => id,
            None => continue,
        };
        let frame = match socketcan::CanFrame::new(Id::Extended(ext_id), &data) {
            Some(f) => f,
            None => continue,
        };
        if socket.write_frame(&frame).is_err() {
            continue;
        }

        let start = Instant::now();
        while start.elapsed() < timeout_per_id {
            match socket.read_frame() {
                Ok(resp) => {
                    if !resp.is_extended() {
                        continue;
                    }
                    let raw_id = match resp.id() {
                        Id::Standard(s) => StandardId::as_raw(&s) as u32,
                        Id::Extended(e) => ExtendedId::as_raw(&e),
                    };
                    let (ct, extra, dev_id) = parse_can_id(raw_id);
                    let resp_motor_id = (extra & 0xFF) as u8;

                    // Skip our own TX echo (gs_usb ECHO flag re-emits the same frame).
                    if dev_id == motor_id && ct == 0 {
                        continue;
                    }
                    if resp_motor_id == motor_id || dev_id == motor_id {
                        found.push(ScanResult {
                            motor_id,
                            payload: resp.data().to_vec(),
                        });
                        break;
                    }
                }
                Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => break,
                Err(e) => return Err(Error::CanSocket(e)),
            }
        }
    }

    if let Some(cb) = on_progress.as_mut() {
        cb(total, total, 0);
    }
    Ok(found)
}

/// Passively listen on the bus and return every frame seen during `duration`.
pub fn dump_bus(interface: &str, duration: Duration) -> Result<Vec<(u32, Vec<u8>)>> {
    let socket = CanSocket::open(interface)?;
    socket.set_read_timeout(Duration::from_millis(100))?;

    let mut frames = Vec::new();
    let start = Instant::now();
    while start.elapsed() < duration {
        match socket.read_frame() {
            Ok(frame) => {
                let raw_id = match frame.id() {
                    Id::Standard(s) => StandardId::as_raw(&s) as u32,
                    Id::Extended(e) => ExtendedId::as_raw(&e),
                };
                frames.push((raw_id, frame.data().to_vec()));
            }
            Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => continue,
            Err(e) => return Err(Error::CanSocket(e)),
        }
    }
    Ok(frames)
}
