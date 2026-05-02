//! High-level [`Motor`] driver wrapping a Linux SocketCAN socket.

use std::io;
use std::time::{Duration, Instant};

use socketcan::{CanSocket, EmbeddedFrame, ExtendedId, Id, Socket, StandardId};

use robstride_protocol::{
    CommType, MitScales, MotorFeedback, MotorModel, ParamIndex, RunMode, build_can_id_raw,
    build_disable_frame, build_enable_frame, build_mit_frame, build_ping_frame,
    build_read_param_frame, build_run_mode_frame, build_set_zero_frame,
    build_write_param_f32_frame, parse_can_id, parse_param_response, parse_status_frame,
    DEFAULT_HOST_ID,
};

use crate::error::{Error, Result};

const DEFAULT_TIMEOUT: Duration = Duration::from_millis(100);

/// High-level controller for a single Robstride motor on a SocketCAN bus.
pub struct Motor {
    socket: CanSocket,
    motor_id: u8,
    host_id: u8,
    model: MotorModel,
    scales: MitScales,
    enabled: bool,
    run_mode: RunMode,
    timeout: Duration,
}

impl Motor {
    /// Open the given CAN interface and bind it to a single motor id.
    pub fn open(interface: &str, motor_id: u8, model: MotorModel) -> Result<Self> {
        Self::open_with_host(interface, motor_id, DEFAULT_HOST_ID, model)
    }

    /// Open with a custom host id (must be greater than every motor id on the
    /// bus for optimal scheduling on the controller side).
    pub fn open_with_host(
        interface: &str,
        motor_id: u8,
        host_id: u8,
        model: MotorModel,
    ) -> Result<Self> {
        let socket = CanSocket::open(interface)?;
        socket.set_read_timeout(DEFAULT_TIMEOUT)?;
        Ok(Self {
            socket,
            motor_id,
            host_id,
            model,
            scales: MitScales::for_model(model),
            enabled: false,
            run_mode: RunMode::Mit,
            timeout: DEFAULT_TIMEOUT,
        })
    }

    pub fn set_timeout(&mut self, timeout: Duration) -> Result<()> {
        self.socket.set_read_timeout(timeout)?;
        self.timeout = timeout;
        Ok(())
    }

    pub fn motor_id(&self) -> u8 {
        self.motor_id
    }

    pub fn host_id(&self) -> u8 {
        self.host_id
    }

    pub fn model(&self) -> MotorModel {
        self.model
    }

    pub fn scales(&self) -> &MitScales {
        &self.scales
    }

    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    pub fn current_run_mode(&self) -> RunMode {
        self.run_mode
    }

    // ---------------------------------------------------------------------
    // Low-level I/O
    // ---------------------------------------------------------------------

    fn send(&self, can_id: u32, data: &[u8]) -> Result<()> {
        let ext_id = ExtendedId::new(can_id).ok_or(Error::InvalidFrame("CAN ID exceeds 29 bits"))?;
        let frame = socketcan::CanFrame::new(Id::Extended(ext_id), data)
            .ok_or(Error::InvalidFrame("payload too long for CAN frame"))?;
        self.socket.write_frame(&frame)?;
        log::debug!("TX id=0x{:08X} data={:02X?}", can_id, data);
        Ok(())
    }

    /// Receive frames until one matches `accept`, dropping unmatched frames.
    /// Returns timeout error if no accepted frame arrives within `self.timeout`.
    fn recv_filtered<F>(&self, mut accept: F) -> Result<(u8, u16, u8, Vec<u8>)>
    where
        F: FnMut(u8, u16, u8) -> bool,
    {
        let start = Instant::now();
        loop {
            if start.elapsed() > self.timeout {
                return Err(Error::Timeout {
                    motor_id: self.motor_id,
                });
            }

            match self.socket.read_frame() {
                Ok(frame) => {
                    if !frame.is_extended() {
                        continue;
                    }
                    let raw_id = match frame.id() {
                        Id::Standard(s) => StandardId::as_raw(&s) as u32,
                        Id::Extended(e) => ExtendedId::as_raw(&e),
                    };
                    let data = frame.data().to_vec();
                    let (comm_type, extra_data, device_id) = parse_can_id(raw_id);
                    log::debug!(
                        "RX id=0x{:08X} comm={} extra=0x{:04X} dev={} data={:02X?}",
                        raw_id, comm_type, extra_data, device_id, &data,
                    );
                    if accept(comm_type, extra_data, device_id) {
                        return Ok((comm_type, extra_data, device_id, data));
                    }
                    // Otherwise drop and keep waiting.
                }
                Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => continue,
                Err(e) => return Err(Error::CanSocket(e)),
            }
        }
    }

    /// Backwards-compatible: receive the next frame regardless of type.
    fn recv(&self) -> Result<(u8, u16, u8, Vec<u8>)> {
        self.recv_filtered(|_, _, _| true)
    }

    fn recv_status(&self) -> Result<MotorFeedback> {
        let (comm_type, extra_data, device_id, data) = self.recv_filtered(|ct, _, _| {
            ct == CommType::OperationStatus as u8 || ct == CommType::FaultReport as u8
        })?;

        if comm_type == CommType::FaultReport as u8 {
            return Err(Error::MotorFault {
                motor_id: device_id,
                extra_data,
            });
        }

        parse_status_frame(
            build_can_id_raw(comm_type, extra_data, device_id),
            &data,
            &self.scales,
        )
        .ok_or_else(|| Error::InvalidResponse("failed to parse status frame".into()))
    }

    // ---------------------------------------------------------------------
    // Bus-level commands
    // ---------------------------------------------------------------------

    /// Send a `GET_DEVICE_ID` (ping). Returns `(device_id_field, payload)`.
    pub fn ping(&self) -> Result<(u16, Vec<u8>)> {
        let (id, data) = build_ping_frame(self.host_id, self.motor_id);
        self.send(id, &data)?;
        let (_ct, extra, _dev, payload) = self.recv()?;
        Ok((extra, payload))
    }

    pub fn enable(&mut self) -> Result<MotorFeedback> {
        let (id, data) = build_enable_frame(self.host_id, self.motor_id);
        self.send(id, &data)?;
        let fb = self.recv_status()?;
        self.enabled = true;
        Ok(fb)
    }

    pub fn disable(&mut self) -> Result<MotorFeedback> {
        let (id, data) = build_disable_frame(self.host_id, self.motor_id);
        self.send(id, &data)?;
        let fb = self.recv_status()?;
        self.enabled = false;
        Ok(fb)
    }

    pub fn set_zero(&mut self) -> Result<()> {
        let (id, data) = build_set_zero_frame(self.host_id, self.motor_id);
        self.send(id, &data)?;
        std::thread::sleep(Duration::from_millis(50));
        Ok(())
    }

    pub fn set_run_mode(&mut self, mode: RunMode) -> Result<()> {
        let (id, data) = build_run_mode_frame(self.host_id, self.motor_id, mode);
        self.send(id, &data)?;
        self.run_mode = mode;
        std::thread::sleep(Duration::from_millis(10));
        Ok(())
    }

    // ---------------------------------------------------------------------
    // MIT mode
    // ---------------------------------------------------------------------

    /// Send a MIT-mode control command. Requires the motor to be enabled.
    pub fn mit_control(
        &self,
        position: f32,
        velocity: f32,
        kp: f32,
        kd: f32,
        torque: f32,
    ) -> Result<MotorFeedback> {
        if !self.enabled {
            return Err(Error::NotEnabled {
                motor_id: self.motor_id,
            });
        }
        let (id, data) = build_mit_frame(
            self.motor_id,
            &self.scales,
            position,
            velocity,
            kp,
            kd,
            torque,
        );
        self.send(id, &data)?;
        self.recv_status()
    }

    // ---------------------------------------------------------------------
    // Position / velocity / torque parameter shortcuts
    // ---------------------------------------------------------------------

    pub fn set_position(&self, position: f32) -> Result<()> {
        self.write_param_f32(ParamIndex::LocRef, position)
    }

    pub fn set_position_speed_limit(&self, speed: f32) -> Result<()> {
        self.write_param_f32(ParamIndex::LimitSpd, speed)
    }

    pub fn set_torque_limit(&self, torque: f32) -> Result<()> {
        self.write_param_f32(ParamIndex::LimitTorque, torque)
    }

    pub fn set_current_limit(&self, current: f32) -> Result<()> {
        self.write_param_f32(ParamIndex::LimitCur, current)
    }

    pub fn set_velocity(&self, velocity: f32) -> Result<()> {
        self.write_param_f32(ParamIndex::SpdRef, velocity)
    }

    pub fn set_torque(&self, iq: f32) -> Result<()> {
        self.write_param_f32(ParamIndex::IqRef, iq)
    }

    // ---------------------------------------------------------------------
    // Parameter access
    // ---------------------------------------------------------------------

    pub fn read_param(&self, param: ParamIndex) -> Result<f32> {
        let (id, data) = build_read_param_frame(self.host_id, self.motor_id, param);
        self.send(id, &data)?;
        let target_idx = param as u16;
        // Drain ReadParameter responses for other indices (left over from
        // overlapping reads or buffered ACKs) until we get the one we asked for.
        let start = Instant::now();
        loop {
            if start.elapsed() > self.timeout {
                return Err(Error::Timeout {
                    motor_id: self.motor_id,
                });
            }
            let (_ct, _extra, _dev, payload) = self.recv_filtered(|ct, _, _| {
                ct == CommType::ReadParameter as u8
            })?;
            let (idx, val) = parse_param_response(&payload)
                .ok_or_else(|| Error::InvalidResponse("failed to parse param response".into()))?;
            if idx == target_idx {
                return Ok(val);
            }
            log::debug!(
                "read_param: discarding stale response idx=0x{:04X} (wanted 0x{:04X})",
                idx, target_idx
            );
        }
    }

    pub fn write_param_f32(&self, param: ParamIndex, value: f32) -> Result<()> {
        let (id, data) = build_write_param_f32_frame(self.host_id, self.motor_id, param, value);
        self.send(id, &data)?;
        std::thread::sleep(Duration::from_millis(5));
        Ok(())
    }

    // ---------------------------------------------------------------------
    // Status
    // ---------------------------------------------------------------------

    /// Send a zero-amplitude MIT command to elicit a status frame, even when
    /// the motor is not enabled.
    pub fn read_status(&self) -> Result<MotorFeedback> {
        let (id, data) = build_mit_frame(self.motor_id, &self.scales, 0.0, 0.0, 0.0, 0.0, 0.0);
        self.send(id, &data)?;
        self.recv_status()
    }

    pub fn read_position(&self) -> Result<f32> {
        self.read_param(ParamIndex::MechPos)
    }

    pub fn read_velocity(&self) -> Result<f32> {
        self.read_param(ParamIndex::MechVel)
    }

    pub fn read_current(&self) -> Result<f32> {
        self.read_param(ParamIndex::IqFilt)
    }

    pub fn read_vbus(&self) -> Result<f32> {
        self.read_param(ParamIndex::Vbus)
    }

    /// Read filtered measured torque (Nm) via parameter access. Safe to call
    /// in any run mode — does not send a control frame.
    pub fn read_torque(&self) -> Result<f32> {
        self.read_param(ParamIndex::MeasuredTorque)
    }

    /// Read motor temperature (°C). The protocol exposes temperature only via
    /// the status frame, so this calls [`Self::read_status`] internally — that
    /// sends a zero-amplitude MIT control frame, which can disrupt non-MIT
    /// run modes. Avoid calling while actively position/velocity/torque
    /// controlling; use it for one-shot snapshots when the motor is idle.
    pub fn read_temperature(&self) -> Result<f32> {
        Ok(self.read_status()?.temperature)
    }
}

impl Drop for Motor {
    fn drop(&mut self) {
        if self.enabled {
            let _ = self.disable();
        }
    }
}
