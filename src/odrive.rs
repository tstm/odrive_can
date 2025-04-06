use core::convert::TryInto;
use core::fmt;

use embedded_can::Frame;
use esp_hal::{
    twai::{EspTwaiError, EspTwaiFrame, StandardId, Twai},
    Async, Blocking, DriverMode,
};

use crate::odrive_errors::ODriveError;

// Command codes from ODrive CAN protocol documentation
// https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#can-msg-get-error

/// Command code for getting version information
const GET_VERSION: u8 = 0x00;
/// Command code for emergency stop
const ESTOP: u8 = 0x02;
/// Command code for getting error information
const GET_ERROR: u8 = 0x03;
/// Command code for setting axis state
const SET_AXIS_STATE: u8 = 0x07;
/// Command code for getting encoder estimates
const GET_ENCODER_ESTIMATES: u8 = 0x09;
/// Command code for setting controller mode
const SET_CONTROLLER_MODE: u8 = 0x0b;
/// Command code for setting input position
const SET_INPUT_POS: u8 = 0x0c;
/// Command code for getting bus voltage and current
const GET_BUS_VOLTAGE_CURRENT: u8 = 0x17;

// const GET_HEARTBEAT: u8 = 0x01;
const RXSDO: u8 = 0x04;
const TXSDO: u8 = 0x05;
// const ADDRESS: u8 = 0x06; // TODO: Implement address setting
// Only for SteadyWin GIM6010-8
// Use the MIT protocol for movement https://github.com/mit-biomimetics/Cheetah-Software
// const SET_INPUT_MIT: u8 = 0x08; // TODO: Implement MIT inputs
const SET_INPUT_VEL: u8 = 0x0d;
const SET_INPUT_TORQUE: u8 = 0x0e;
const SET_LIMITS: u8 = 0x0f;
const SET_TRAJ_VEL_LIMIT: u8 = 0x11;
const SET_TRAJ_ACCEL_LIMITS: u8 = 0x12;
const SET_TRAJ_INERTIA: u8 = 0x13;
const GET_IQ: u8 = 0x14;
const GET_TEMPERATURE: u8 = 0x15;
const REBOOT: u8 = 0x16;
const CLEAR_ERRORS: u8 = 0x18;
const SET_ABSOLUTE_POSITION: u8 = 0x19;
const SET_POS_GAIN: u8 = 0x1a;
const SET_VEL_GAINS: u8 = 0x1b;
const GET_TORQUES: u8 = 0x1c;
const GET_POWERS: u8 = 0x1d;
// Only for SteadyWin GIM6010-8
const DISABLE_CAN: u8 = 0x1e;
const ENTER_DFU_MODE: u8 = 0x1f;

/// Maximum number of retries when listening for CAN responses
const MAX_LISTEN_RETRY: u16 = 500;

/// Structure representing version information from ODrive
#[derive(Debug, Clone)]
pub struct Versions {
    /// Protocol version
    protocol_version: u8,
    /// Hardware major version
    hw_version_major: u8,
    /// Hardware minor version
    hw_version_minor: u8,
    /// Hardware variant
    hw_version_variant: u8,
    /// Firmware major version
    fw_version_major: u8,
    /// Firmware minor version
    fw_version_minor: u8,
    /// Firmware revision
    fw_version_revision: u8,
    /// Whether firmware is unreleased (development version)
    fw_version_unreleased: u8,
}

impl fmt::Display for Versions {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "P:{} HW:{}.{}.{} FW:{}.{}.{} U:{}",
            self.protocol_version,
            self.hw_version_major,
            self.hw_version_minor,
            self.hw_version_variant,
            self.fw_version_major,
            self.fw_version_minor,
            self.fw_version_revision,
            self.fw_version_unreleased,
        )
    }
}

/// Enum representing possible axis states of ODrive
#[derive(Debug, Clone, Copy)]
pub enum AxisState {
    /// Undefined state
    Undefined, // 0x0
    /// Idle state
    Idle, // 0x1
    /// Startup sequence state
    StartupSequence, // 0x2
    /// Full calibration sequence state
    FullCalibrationSequence, // 0x3
    /// Motor calibration state
    MotorCalibration, // 0x4
    /// Encoder index search state
    EncoderIndexSearch, // 0x6
    /// Encoder offset calibration state
    EncoderOffsetCalibration, // 0x7
    /// Closed loop control state
    ClosedLoopControl, // 0x8
    /// Lockin spin state
    LockinSpin, // 0x9
    /// Encoder direction find state
    EncoderDirFind, // 0xA
    /// Homing state
    Homing, // 0xB
    /// Encoder hall polarity calibration state
    EncoderHallPolarityCalibration, // 0xC
    /// Encoder hall phase calibration state
    EncoderHallPhaseCalibration, // 0xD
    /// Anticogging calibration state
    AnticoggingCalibration, // 0xE
}

impl AxisState {
    /// Gets the numeric value associated with the axis state
    pub fn get_value(&self) -> u32 {
        match self {
            AxisState::Undefined => 0x0,
            AxisState::Idle => 0x1,
            AxisState::StartupSequence => 0x2,
            AxisState::FullCalibrationSequence => 0x3,
            AxisState::MotorCalibration => 0x4,
            AxisState::EncoderIndexSearch => 0x6,
            AxisState::EncoderOffsetCalibration => 0x7,
            AxisState::ClosedLoopControl => 0x8,
            AxisState::LockinSpin => 0x9,
            AxisState::EncoderDirFind => 0xA,
            AxisState::Homing => 0xB,
            AxisState::EncoderHallPolarityCalibration => 0xC,
            AxisState::EncoderHallPhaseCalibration => 0xD,
            AxisState::AnticoggingCalibration => 0xE,
        }
    }

    /// Gets the AxisState from its numeric value
    pub fn get_state(state: u32) -> AxisState {
        match state {
            0x0 => AxisState::Undefined,
            0x1 => AxisState::Idle,
            0x2 => AxisState::StartupSequence,
            0x3 => AxisState::FullCalibrationSequence,
            0x4 => AxisState::MotorCalibration,
            0x6 => AxisState::EncoderIndexSearch,
            0x7 => AxisState::EncoderOffsetCalibration,
            0x8 => AxisState::ClosedLoopControl,
            0x9 => AxisState::LockinSpin,
            0xA => AxisState::EncoderDirFind,
            0xB => AxisState::Homing,
            0xC => AxisState::EncoderHallPolarityCalibration,
            0xD => AxisState::EncoderHallPhaseCalibration,
            0xE => AxisState::AnticoggingCalibration,
            _ => panic!("Unsupported Axis State"),
        }
    }
}

/// Enum representing control modes of ODrive
#[derive(Debug, Clone, Copy)]
pub enum ControlMode {
    /// Voltage control mode
    Voltage, // 0x0
    /// Torque control mode
    Torque, // 0x1
    /// Velocity control mode
    Velocity, // 0x2
    /// Position control mode
    Position, // 0x3
}

impl ControlMode {
    /// Gets the numeric value associated with the control mode
    pub fn get_value(&self) -> u32 {
        match self {
            ControlMode::Voltage => 0x0,
            ControlMode::Torque => 0x1,
            ControlMode::Velocity => 0x2,
            ControlMode::Position => 0x3,
        }
    }

    /// Gets the ControlMode from its numeric value
    pub fn get_mode(value: u32) -> ControlMode {
        match value {
            0x0 => ControlMode::Voltage,
            0x1 => ControlMode::Torque,
            0x2 => ControlMode::Velocity,
            0x3 => ControlMode::Position,
            _ => panic!("Unsupported control mode"),
        }
    }
}

/// Enum representing input modes of ODrive
#[derive(Debug, Clone, Copy)]
pub enum InputMode {
    /// Inactive input mode
    Inactive, // 0x0
    /// Passthrough input mode
    Passthrough, // 0x1
    /// Velocity ramp input mode
    VelocityRamp, // 0x2
    /// Position filter input mode
    PositionFilter, // 0x3
    /// Trapezoidal trajectory input mode
    TrapezoidalTrajectory, // 0x5
    /// Torque ramp input mode
    TorqueRamp, // 0x6
    /// Mirror input mode
    Mirror, // 0x7
    /// MIT motion control input mode (only on GIM6010-8)
    MitMotionControl, // 0x9
}

impl InputMode {
    /// Gets the numeric value associated with the input mode
    pub fn get_value(&self) -> u32 {
        match self {
            InputMode::Inactive => 0x0,
            InputMode::Passthrough => 0x1,
            InputMode::VelocityRamp => 0x2,
            InputMode::PositionFilter => 0x3,
            InputMode::TrapezoidalTrajectory => 0x5,
            InputMode::TorqueRamp => 0x6,
            InputMode::Mirror => 0x7,
            InputMode::MitMotionControl => 0x8,
        }
    }

    /// Gets the InputMode from its numeric value
    pub fn get_mode(value: u32) -> InputMode {
        match value {
            0x0 => InputMode::Inactive,
            0x1 => InputMode::Passthrough,
            0x2 => InputMode::VelocityRamp,
            0x3 => InputMode::PositionFilter,
            0x5 => InputMode::TrapezoidalTrajectory,
            0x6 => InputMode::TorqueRamp,
            0x7 => InputMode::Mirror,
            0x9 => InputMode::MitMotionControl,
            _ => panic!("Unsupported control mode"),
        }
    }
}

/// Controller for ODrive over CAN interface
///
/// This struct provides both asynchronous and blocking implementations
/// for communicating with ODrive motor controllers over CAN bus.
pub struct ODriveCanController<'a, T: DriverMode> {
    twai: Twai<'a, T>,
    node_id: u8,
}

impl<'a, T: DriverMode> ODriveCanController<'a, T> {
    /// Creates a new ODriveCanController instance
    ///
    /// # Arguments
    /// * `twai` - The TWAI (CAN) interface to use
    /// * `node_id` - The CAN node ID of the ODrive controller
    pub fn new(twai: Twai<'a, T>, node_id: u8) -> Self {
        Self { twai, node_id }
    }

    // Common helper methods

    /// Builds a CAN frame for the given command and data
    fn build_frame(&self, cmd_id: u8, data: &[u8]) -> EspTwaiFrame {
        let can_id = StandardId::new(((self.node_id as u16) << 5) | (cmd_id as u16)).unwrap();
        EspTwaiFrame::new(can_id, data).unwrap()
    }

    /// Parses two f32 values from CAN data
    fn parse_two_f32(data: &[u8; 8]) -> (f32, f32) {
        let a = f32::from_le_bytes(data[0..4].try_into().unwrap());
        let b = f32::from_le_bytes(data[4..8].try_into().unwrap());
        (a, b)
    }

    /// Parses two ODriveErrors from CAN data
    fn parse_two_odrive_errors(data: &[u8; 8]) -> (ODriveError, ODriveError) {
        let a = u32::from_le_bytes(data[0..4].try_into().unwrap());
        let b = u32::from_le_bytes(data[4..8].try_into().unwrap());
        (ODriveError::from_bits(a), ODriveError::from_bits(b))
    }

    /// Parses version information from CAN data
    fn parse_versions(data: &[u8; 8]) -> Versions {
        Versions {
            protocol_version: u8::from_le_bytes(data[0..1].try_into().unwrap()),
            hw_version_major: u8::from_le_bytes(data[1..2].try_into().unwrap()),
            hw_version_minor: u8::from_le_bytes(data[2..3].try_into().unwrap()),
            hw_version_variant: u8::from_le_bytes(data[3..4].try_into().unwrap()),
            fw_version_major: u8::from_le_bytes(data[4..5].try_into().unwrap()),
            fw_version_minor: u8::from_le_bytes(data[5..6].try_into().unwrap()),
            fw_version_revision: u8::from_le_bytes(data[6..7].try_into().unwrap()),
            fw_version_unreleased: u8::from_le_bytes(data[7..8].try_into().unwrap()),
        }
    }

    /// Parses a response frame from the ODrive
    fn parse_response_frame(&self, frame: &EspTwaiFrame, exp_cmd_id: u8) -> Option<(u8, [u8; 8])> {
        if frame.is_standard() {
            if let embedded_can::Id::Standard(id) = frame.id() {
                // let node_id = id.as_raw() >> 5;
                let cmd_id = (id.as_raw() & 0x1F) as u8;
                if cmd_id != exp_cmd_id {
                    return None;
                }
                let data: [u8; 8] = frame.data().try_into().unwrap_or([0; 8]);
                // log::debug!(
                //     "Parsed packet correctly from device Node: {node_id:?} Command: {cmd_id} and {data:?}"
                // );
                return Some((cmd_id, data));
            } else {
                // log::error!("Some weird packets in CAN..");
            }
        }
        None
    }

    // Common command builders

    /// Builds data for get/set parameter command
    fn build_parameter_data(opcode: u8, endpoint_id: u16, value: &[u8]) -> [u8; 8] {
        let mut data = [0u8; 8];
        data[0..1].copy_from_slice(&opcode.to_le_bytes());
        data[1..2].copy_from_slice(&endpoint_id.to_le_bytes());
        data[4..8].copy_from_slice(value);
        data
    }

    /// Builds data for set position command
    fn build_set_position_data(position: f32, vel_ff: u16, torque_ff: u16) -> [u8; 8] {
        let mut data = [0u8; 8];
        data[0..4].copy_from_slice(&position.to_le_bytes());
        data[4..6].copy_from_slice(&vel_ff.to_le_bytes());
        data[6..8].copy_from_slice(&torque_ff.to_le_bytes());
        data
    }

    /// Builds data for set velocity command
    fn build_set_velocity_data(vel_ff: f32, torque_ff: f32) -> [u8; 8] {
        let mut data = [0u8; 8];
        data[0..4].copy_from_slice(&vel_ff.to_le_bytes());
        data[4..8].copy_from_slice(&torque_ff.to_le_bytes());
        data
    }

    /// Builds data for set torque command
    fn build_set_torque_data(torque_ff: f32) -> [u8; 8] {
        let mut data = [0u8; 8];
        data[0..4].copy_from_slice(&torque_ff.to_le_bytes());
        data
    }

    /// Builds data for set limits command
    fn build_set_limits_data(vel_limit: f32, current_limit: f32) -> [u8; 8] {
        let mut data = [0u8; 8];
        data[0..4].copy_from_slice(&vel_limit.to_le_bytes());
        data[4..8].copy_from_slice(&current_limit.to_le_bytes());
        data
    }

    /// Builds data for set trajectory velocity command in rev/s
    fn build_set_traj_vel_limit_data(traj_vel_limit: f32) -> [u8; 8] {
        let mut data = [0u8; 8];
        data[0..4].copy_from_slice(&traj_vel_limit.to_le_bytes());
        data
    }

    /// Builds data for set trajectory acceleration command in rev/s^2
    fn build_set_traj_accel_limits_data(traj_accel_limit: f32, traj_decel_limit: f32) -> [u8; 8] {
        let mut data = [0u8; 8];
        data[0..4].copy_from_slice(&traj_accel_limit.to_le_bytes());
        data[4..8].copy_from_slice(&traj_decel_limit.to_le_bytes());
        data
    }

    /// Builds data for set trajectory acceleration command in rev/s^2
    fn build_set_traj_inertia_data(traj_inertia: f32) -> [u8; 8] {
        let mut data = [0u8; 8];
        data[0..4].copy_from_slice(&traj_inertia.to_le_bytes());
        data
    }

    /// Builds data for set absolute position command in rev
    fn build_set_absolute_position(pos_estimate: f32) -> [u8; 8] {
        let mut data = [0u8; 8];
        data[0..4].copy_from_slice(&pos_estimate.to_le_bytes());
        data
    }

    /// Builds data for set position gain in (rev/s)/rev
    fn build_set_pos_gain_data(pos_gain: f32) -> [u8; 8] {
        let mut data = [0u8; 8];
        data[0..4].copy_from_slice(&pos_gain.to_le_bytes());
        data
    }

    /// Builds data for set velocity gain command in Nm/(rev/s) and Nm/rev
    fn build_set_vel_gains_data(vel_gain: f32, vel_integrator_gain: f32) -> [u8; 8] {
        let mut data = [0u8; 8];
        data[0..4].copy_from_slice(&vel_gain.to_le_bytes());
        data[4..8].copy_from_slice(&vel_integrator_gain.to_le_bytes());
        data
    }

    /// Builds data for set controller mode command
    fn build_set_controller_mode_data(control_mode: ControlMode, input_mode: InputMode) -> [u8; 8] {
        let mut data = [0u8; 8];
        let cm = control_mode.get_value();
        let im = input_mode.get_value();
        data[0..4].copy_from_slice(&cm.to_le_bytes());
        data[4..8].copy_from_slice(&im.to_le_bytes());
        data
    }

    /// Builds data for set axis state command
    fn build_set_axis_state_data(axis_state: AxisState) -> [u8; 8] {
        let mut data = [0u8; 8];
        let state_value = axis_state.get_value();
        data[0..4].copy_from_slice(&state_value.to_le_bytes());
        data
    }
}

unsafe impl<'a, T: DriverMode> Send for ODriveCanController<'a, T> {}
unsafe impl<'a, T: DriverMode> Sync for ODriveCanController<'a, T> {}

// Async-specific implementation
impl<'a> ODriveCanController<'a, Async> {
    /// Sends a command to the ODrive asynchronously
    ///
    /// # Arguments
    /// * `cmd_id` - The command ID to send
    /// * `data` - The data to send with the command
    ///
    /// # Returns
    /// Result indicating success or failure
    pub async fn send_command(&mut self, cmd_id: u8, data: &[u8]) -> Result<(), EspTwaiError> {
        let frame = self.build_frame(cmd_id, data);
        self.twai.clear_receive_fifo();
        self.twai.transmit_async(&frame).await
    }

    /// Receives a response from the ODrive asynchronously
    ///
    /// # Arguments
    /// * `exp_cmd_id` - The expected command ID in the response
    ///
    /// # Returns
    /// Result containing the command ID and data, or an error
    pub async fn receive_response(
        &mut self,
        exp_cmd_id: u8,
    ) -> Result<(u8, [u8; 8]), EspTwaiError> {
        for _ in 0..MAX_LISTEN_RETRY {
            let frame = self.twai.receive_async().await?;
            if let Some(result) = self.parse_response_frame(&frame, exp_cmd_id) {
                return Ok(result);
            }
        }
        return Err(EspTwaiError::EmbeddedHAL(esp_hal::twai::ErrorKind::Other));
    }

    /// Sets the position of the ODrive asynchronously
    ///
    /// # Arguments
    /// * `position` - The target position in turns
    /// * `vel_ff` - Velocity feedforward in counts/s
    /// * `torque_ff` - Torque feedforward in counts
    ///
    /// # Returns
    /// Result indicating success or failure
    pub async fn set_position(
        &mut self,
        position: f32,
        vel_ff: u16,
        torque_ff: u16,
    ) -> Result<(), EspTwaiError> {
        let data = Self::build_set_position_data(position, vel_ff, torque_ff);
        self.send_command(SET_INPUT_POS, &data).await
    }

    /// Sets the controller mode of the ODrive asynchronously
    ///
    /// # Arguments
    /// * `control_mode` - The control mode to set
    /// * `input_mode` - The input mode to set
    ///
    /// # Returns
    /// Result indicating success or failure
    pub async fn set_controller_mode(
        &mut self,
        control_mode: ControlMode,
        input_mode: InputMode,
    ) -> Result<(), EspTwaiError> {
        let data = Self::build_set_controller_mode_data(control_mode, input_mode);
        self.send_command(SET_CONTROLLER_MODE, &data).await
    }

    /// Sets the axis state of the ODrive asynchronously
    ///
    /// # Arguments
    /// * `axis_state` - The axis state to set
    ///
    /// # Returns
    /// Result indicating success or failure
    pub async fn set_axis_state(&mut self, axis_state: AxisState) -> Result<(), EspTwaiError> {
        let data = Self::build_set_axis_state_data(axis_state);
        self.send_command(SET_AXIS_STATE, &data).await
    }

    /// Sets the velocity of the ODrive asynchronously
    ///
    /// # Arguments
    /// * `velocity` - The desired velocity in f32 in rev/s
    /// * `torque` - The desired torque in f32 in Nm
    ///
    /// # Returns
    /// Result indicating success or failure
    pub async fn set_velocity(&mut self, velocity: f32, torque: f32) -> Result<(), EspTwaiError> {
        let data = Self::build_set_velocity_data(velocity, torque);
        self.send_command(SET_INPUT_VEL, &data).await
    }

    /// Sets the torque of the ODrive asynchronously
    ///
    /// # Arguments
    /// * `torque` - The desired torque in f32 in Nm
    ///
    /// # Returns
    /// Result indicating success or failure
    pub async fn set_torque(&mut self, torque: f32) -> Result<(), EspTwaiError> {
        let data = Self::build_set_torque_data(torque);
        self.send_command(SET_INPUT_TORQUE, &data).await
    }

    /// Sets the limits of the ODrive asynchronously
    ///
    /// # Arguments
    /// * `velocity_limit` - The limit for velocity in rev/s
    /// * `current_limit` - The limit for current in A
    ///
    /// # Returns
    /// Result indicating success or failure
    pub async fn set_limits(
        &mut self,
        velocity_limit: f32,
        current_limit: f32,
    ) -> Result<(), EspTwaiError> {
        let data = Self::build_set_limits_data(velocity_limit, current_limit);
        self.send_command(SET_LIMITS, &data).await
    }

    /// Sets the trajectory velocity limit of the ODrive asynchronously
    ///
    /// # Arguments
    /// * `velocity_limit` - The limit for velocity in rev/s
    ///
    /// # Returns
    /// Result indicating success or failure
    pub async fn set_trajectory_velocity_limit(
        &mut self,
        velocity_limit: f32,
    ) -> Result<(), EspTwaiError> {
        let data = Self::build_set_traj_vel_limit_data(velocity_limit);
        self.send_command(SET_TRAJ_VEL_LIMIT, &data).await
    }

    /// Sets the trajectory acceleration limits of the ODrive asynchronously
    ///
    /// # Arguments
    /// * `accel_limit` - The limit for acceleration in rev/s^2
    /// * `decel_limit` - The limit for decelration in rev/s^2
    ///
    /// # Returns
    /// Result indicating success or failure
    pub async fn set_trajectory_accel_limits(
        &mut self,
        accel_limit: f32,
        decel_limit: f32,
    ) -> Result<(), EspTwaiError> {
        let data = Self::build_set_traj_accel_limits_data(accel_limit, decel_limit);
        self.send_command(SET_TRAJ_ACCEL_LIMITS, &data).await
    }

    /// Sets the absolute position of the ODrive asynchronously
    /// See: https://docs.odriverobotics.com/v/latest/manual/control.html#user-reference-frame
    ///
    /// # Arguments
    /// * `pos_estimate` - The absolute position in f32
    ///
    /// # Returns
    /// Result indicating success or failure
    pub async fn set_absolute_position(&mut self, pos_estimate: f32) -> Result<(), EspTwaiError> {
        let data = Self::build_set_absolute_position(pos_estimate);
        self.send_command(SET_ABSOLUTE_POSITION, &data).await
    }

    /// Sets the position gain of the ODrive asynchronously
    ///
    /// # Arguments
    /// * `pos_gain` - The position gain in f32 (rev/s)/rev
    ///
    /// # Returns
    /// Result indicating success or failure
    pub async fn set_pos_gain(&mut self, pos_gain: f32) -> Result<(), EspTwaiError> {
        let data = Self::build_set_pos_gain_data(pos_gain);
        self.send_command(SET_POS_GAIN, &data).await
    }

    /// Sets the velocity gains of the ODrive asynchronously
    ///
    /// # Arguments
    /// * `vel_gain` - The position gain in f32 Nm/(rev/s)
    /// * `vel_integrator_gain` - The position gain in f32 Nm/rev
    ///
    /// # Returns
    /// Result indicating success or failure
    pub async fn set_vel_gains(
        &mut self,
        vel_gain: f32,
        vel_integrator_gain: f32,
    ) -> Result<(), EspTwaiError> {
        let data = Self::build_set_vel_gains_data(vel_gain, vel_integrator_gain);
        self.send_command(SET_VEL_GAINS, &data).await
    }

    /// Sets the trajectory inertia of the ODrive asynchronously
    ///
    /// # Arguments
    /// * `traj_inertia` - The limit for velocity in Nm/(rev/s^2)
    ///
    /// # Returns
    /// Result indicating success or failure
    pub async fn set_trajectory_inertia(&mut self, traj_inertia: f32) -> Result<(), EspTwaiError> {
        let data = Self::build_set_traj_inertia_data(traj_inertia);
        self.send_command(SET_TRAJ_INERTIA, &data).await
    }

    /// Gets a parameter according to ODrive endpoints.json
    ///
    /// # Arguments
    /// * `endpoint_id` - The endpoint id from an endpoints.json
    ///
    /// # Returns
    /// The raw data (4 bytes), left to the client to parse according to the endpoint information
    pub async fn get_parameter(&mut self, endpoint_id: u16) -> Result<[u8; 4], EspTwaiError> {
        let mut response = [0u8; 4];
        let data = Self::build_parameter_data(0, endpoint_id, &[]);
        self.send_command(RXSDO, &data).await?;
        response.copy_from_slice(&self.receive_response(TXSDO).await?.1[4..8]);
        Ok(response)
    }

    /// Sets a parameter according to ODrive endpoints.json
    ///
    /// # Arguments
    /// * `endpoint_id` - The endpoint id from an endpoints.json
    /// * `data` - up to 4 bytes of data to set for the endpoint according to endpoints.json
    ///
    /// # Returns
    /// Result indicating success or failure
    pub async fn set_parameter(
        &mut self,
        endpoint_id: u16,
        data: &[u8],
    ) -> Result<(), EspTwaiError> {
        let parameters = Self::build_parameter_data(1, endpoint_id, data);
        self.send_command(RXSDO, &parameters).await
    }

    /// Triggers an emergency stop on the ODrive asynchronously
    ///
    /// # Returns
    /// Result indicating success or failure
    pub async fn estop(&mut self) -> Result<(), EspTwaiError> {
        self.send_command(ESTOP, &[]).await
    }

    /// Triggers a reboot on the ODrive asynchronously
    ///
    /// # Returns
    /// Result indicating success or failure
    pub async fn reboot(&mut self) -> Result<(), EspTwaiError> {
        self.send_command(REBOOT, &[]).await
    }

    /// Enters DFU Mode on the ODrive asynchronously
    /// Equivalent to calling enter_dfu_mode2()
    ///
    /// # Returns
    /// Result indicating success or failure
    pub async fn enter_dfu_mode(&mut self) -> Result<(), EspTwaiError> {
        self.send_command(ENTER_DFU_MODE, &[]).await
    }

    /// Disables CAN bus on the ODrive asynchronously
    /// Only available on GIM6010-8 - could work on others, no guarantees
    ///
    /// # Returns
    /// Result indicating success or failure
    pub async fn disable_can(&mut self) -> Result<(), EspTwaiError> {
        self.send_command(DISABLE_CAN, &[]).await
    }

    /// Clears errors from the ODrive
    ///
    /// # Returns
    /// Result containing (active_errors, disarm_reason) or an error
    pub async fn clear_errors(&mut self) -> Result<(), EspTwaiError> {
        self.send_command(CLEAR_ERRORS, &[]).await
    }

    // Response parsing methods

    /// Helper method to send a request and parse the response
    async fn request_and_parse<T, F>(&mut self, cmd: u8, parser: F) -> Result<T, EspTwaiError>
    where
        F: FnOnce(&[u8; 8]) -> T,
    {
        self.send_command(cmd, &[]).await?;
        let (_, data) = self.receive_response(cmd).await?;
        Ok(parser(&data))
    }

    /// Requests encoder estimates from the ODrive asynchronously
    ///
    /// # Returns
    /// Result containing (position, velocity) or an error
    pub async fn get_encoder_estimates(&mut self) -> Result<(f32, f32), EspTwaiError> {
        let (pos, vel) = self
            .request_and_parse(GET_ENCODER_ESTIMATES, Self::parse_two_f32)
            .await?;

        Ok((pos, vel))
    }

    /// Requests Iq from the ODrive asynchronously
    ///
    /// # Returns
    /// Result containing (iq_setpoint, iq_measured) or an error
    pub async fn get_iq(&mut self) -> Result<(f32, f32), EspTwaiError> {
        let (iq_setpoint, iq_measured) =
            self.request_and_parse(GET_IQ, Self::parse_two_f32).await?;

        Ok((iq_setpoint, iq_measured))
    }

    /// Requests temperatures from the ODrive asynchronously
    ///
    /// # Returns
    /// Result containing (fet_temperature, motor_temperature) or an error
    pub async fn get_temperature(&mut self) -> Result<(f32, f32), EspTwaiError> {
        let (fet_temperature, motor_temperature) = self
            .request_and_parse(GET_TEMPERATURE, Self::parse_two_f32)
            .await?;

        Ok((fet_temperature, motor_temperature))
    }

    /// Requests bus voltage and current measurements from the ODrive asynchronously
    ///
    /// # Returns
    /// Result containing (voltage, current) or an error
    pub async fn get_voltages(&mut self) -> Result<(f32, f32), EspTwaiError> {
        let (voltage, current) = self
            .request_and_parse(GET_BUS_VOLTAGE_CURRENT, Self::parse_two_f32)
            .await?;

        Ok((voltage, current))
    }

    /// Requests error information from the ODrive asynchronously
    ///
    /// # Returns
    /// Result containing (active_errors, disarm_reason) or an error
    pub async fn get_errors(&mut self) -> Result<(ODriveError, ODriveError), EspTwaiError> {
        let (active_errors, disarm_reason) = self
            .request_and_parse(GET_ERROR, Self::parse_two_odrive_errors)
            .await?;

        Ok((active_errors, disarm_reason))
    }

    /// Requests version information from the ODrive asynchronously
    ///
    /// # Returns
    /// Result containing version information or an error
    pub async fn get_versions(&mut self) -> Result<Versions, EspTwaiError> {
        let versions = self
            .request_and_parse(GET_VERSION, Self::parse_versions)
            .await?;

        Ok(versions)
    }

    /// Requests torque information from the ODrive asynchronously
    ///
    /// # Returns
    /// Result containing (torque_target, torque_estimate) in Nm
    pub async fn get_torques(&mut self) -> Result<(f32, f32), EspTwaiError> {
        let (torque_target, torque_estimate) = self
            .request_and_parse(GET_TORQUES, Self::parse_two_f32)
            .await?;

        Ok((torque_target, torque_estimate))
    }

    /// Requests power information from the ODrive asynchronously
    ///
    /// # Returns
    /// Result containing (electrical_power, mechanical_power) in W
    pub async fn get_powers(&mut self) -> Result<(f32, f32), EspTwaiError> {
        let (electrical_power, mechanical_power) = self
            .request_and_parse(GET_POWERS, Self::parse_two_f32)
            .await?;

        Ok((electrical_power, mechanical_power))
    }
}

// Synchronous implementation
impl<'a> ODriveCanController<'a, Blocking> {
    /// Sends a command to the ODrive
    ///
    /// # Arguments
    /// * `cmd_id` - The command ID to send
    /// * `data` - The data to send with the command
    ///
    /// # Returns
    /// Result indicating success or would-block
    pub fn send_command(&mut self, cmd_id: u8, data: &[u8]) -> nb::Result<(), EspTwaiError> {
        let frame = self.build_frame(cmd_id, data);
        self.twai.clear_receive_fifo();
        self.twai.transmit(&frame)
    }

    /// Receives a response from the ODrive
    ///
    /// # Arguments
    /// * `exp_cmd_id` - The expected command ID in the response
    ///
    /// # Returns
    /// Result containing the command ID and data, or would-block/error
    pub fn receive_response(&mut self, exp_cmd_id: u8) -> nb::Result<(u8, [u8; 8]), EspTwaiError> {
        for _ in 0..MAX_LISTEN_RETRY {
            let frame = self.twai.receive()?;
            if let Some(result) = self.parse_response_frame(&frame, exp_cmd_id) {
                return Ok(result);
            }
        }
        return nb::Result::Err(nb::Error::Other(EspTwaiError::EmbeddedHAL(
            esp_hal::twai::ErrorKind::Other,
        )));
    }

    /// Sets the position of the ODrive
    ///
    /// # Arguments
    /// * `position` - The target position in turns
    /// * `vel_ff` - Velocity feedforward in counts/s
    /// * `torque_ff` - Torque feedforward in counts
    ///
    /// # Returns
    /// Result indicating success or would-block
    pub fn set_position(
        &mut self,
        position: f32,
        vel_ff: u16,
        torque_ff: u16,
    ) -> nb::Result<(), EspTwaiError> {
        let data = Self::build_set_position_data(position, vel_ff, torque_ff);
        self.send_command(SET_INPUT_POS, &data)
    }

    /// Sets the controller mode of the ODrive
    ///
    /// # Arguments
    /// * `control_mode` - The control mode to set
    /// * `input_mode` - The input mode to set
    ///
    /// # Returns
    /// Result indicating success or would-block
    pub fn set_controller_mode(
        &mut self,
        control_mode: ControlMode,
        input_mode: InputMode,
    ) -> nb::Result<(), EspTwaiError> {
        let data = Self::build_set_controller_mode_data(control_mode, input_mode);
        self.send_command(SET_CONTROLLER_MODE, &data)
    }

    /// Sets the axis state of the ODrive
    ///
    /// # Arguments
    /// * `axis_state` - The axis state to set
    ///
    /// # Returns
    /// Result indicating success or would-block
    pub fn set_axis_state(&mut self, axis_state: AxisState) -> nb::Result<(), EspTwaiError> {
        let data = Self::build_set_axis_state_data(axis_state);
        self.send_command(SET_AXIS_STATE, &data)
    }

    /// Sets the velocity of the ODrive
    ///
    /// # Arguments
    /// * `velocity` - The desired velocity in f32 in rev/s
    /// * `torque` - The desired torque in f32 in Nm
    ///
    /// # Returns
    /// Result indicating success or failure
    pub fn set_velocity(&mut self, velocity: f32, torque: f32) -> nb::Result<(), EspTwaiError> {
        let data = Self::build_set_velocity_data(velocity, torque);
        self.send_command(SET_INPUT_VEL, &data)
    }

    /// Sets the torque of the ODrive
    ///
    /// # Arguments
    /// * `torque` - The desired torque in f32 in Nm
    ///
    /// # Returns
    /// Result indicating success or failure
    pub fn set_torque(&mut self, torque: f32) -> nb::Result<(), EspTwaiError> {
        let data = Self::build_set_torque_data(torque);
        self.send_command(SET_INPUT_TORQUE, &data)
    }

    /// Sets the limits of the ODrive asynchronously
    ///
    /// # Arguments
    /// * `velocity_limit` - The limit for velocity in rev/s
    /// * `current_limit` - The limit for current in A
    ///
    /// # Returns
    /// Result indicating success or failure
    pub fn set_limits(
        &mut self,
        velocity_limit: f32,
        current_limit: f32,
    ) -> nb::Result<(), EspTwaiError> {
        let data = Self::build_set_limits_data(velocity_limit, current_limit);
        self.send_command(SET_LIMITS, &data)
    }

    /// Sets the trajectory velocity limit of the ODrive
    ///
    /// # Arguments
    /// * `velocity_limit` - The limit for velocity in rev/s
    ///
    /// # Returns
    /// Result indicating success or failure
    pub fn set_trajectory_velocity_limit(
        &mut self,
        velocity_limit: f32,
    ) -> nb::Result<(), EspTwaiError> {
        let data = Self::build_set_traj_vel_limit_data(velocity_limit);
        self.send_command(SET_TRAJ_VEL_LIMIT, &data)
    }

    /// Sets the trajectory acceleration limits of the ODrive
    ///
    /// # Arguments
    /// * `accel_limit` - The limit for acceleration in rev/s^2
    /// * `decel_limit` - The limit for decelration in rev/s^2
    ///
    /// # Returns
    /// Result indicating success or failure
    pub fn set_trajectory_accel_limits(
        &mut self,
        accel_limit: f32,
        decel_limit: f32,
    ) -> nb::Result<(), EspTwaiError> {
        let data = Self::build_set_traj_accel_limits_data(accel_limit, decel_limit);
        self.send_command(SET_TRAJ_ACCEL_LIMITS, &data)
    }

    /// Sets the absolute position of the ODrive
    /// See: https://docs.odriverobotics.com/v/latest/manual/control.html#user-reference-frame
    ///
    /// # Arguments
    /// * `pos_estimate` - The absolute position in f32
    ///
    /// # Returns
    /// Result indicating success or failure
    pub fn set_absolute_position(&mut self, pos_estimate: f32) -> nb::Result<(), EspTwaiError> {
        let data = Self::build_set_absolute_position(pos_estimate);
        self.send_command(SET_ABSOLUTE_POSITION, &data)
    }

    /// Sets the position gain of the ODrive
    ///
    /// # Arguments
    /// * `pos_gain` - The position gain in f32 (rev/s)/rev
    ///
    /// # Returns
    /// Result indicating success or failure
    pub fn set_pos_gain(&mut self, pos_gain: f32) -> nb::Result<(), EspTwaiError> {
        let data = Self::build_set_pos_gain_data(pos_gain);
        self.send_command(SET_POS_GAIN, &data)
    }

    /// Sets the velocity gains of the ODrive
    ///
    /// # Arguments
    /// * `vel_gain` - The position gain in f32 Nm/(rev/s)
    /// * `vel_integrator_gain` - The position gain in f32 Nm/rev
    ///
    /// # Returns
    /// Result indicating success or failure
    pub fn set_vel_gains(
        &mut self,
        vel_gain: f32,
        vel_integrator_gain: f32,
    ) -> nb::Result<(), EspTwaiError> {
        let data = Self::build_set_vel_gains_data(vel_gain, vel_integrator_gain);
        self.send_command(SET_VEL_GAINS, &data)
    }

    /// Sets the trajectory inertia of the ODrive asynchronously
    ///
    /// # Arguments
    /// * `traj_inertia` - The limit for velocity in Nm/(rev/s^2)
    ///
    /// # Returns
    /// Result indicating success or failure
    pub fn set_trajectory_inertia(&mut self, traj_inertia: f32) -> nb::Result<(), EspTwaiError> {
        let data = Self::build_set_traj_inertia_data(traj_inertia);
        self.send_command(SET_TRAJ_INERTIA, &data)
    }

    /// Gets a parameter according to ODrive endpoints.json
    ///
    /// # Arguments
    /// * `endpoint_id` - The endpoint id from an endpoints.json
    ///
    /// # Returns
    /// The raw data (4 bytes), left to the client to parse according to the endpoint information
    pub fn get_parameter(&mut self, endpoint_id: u16) -> nb::Result<[u8; 4], EspTwaiError> {
        let mut response = [0u8; 4];
        let data = Self::build_parameter_data(0, endpoint_id, &[]);
        self.send_command(RXSDO, &data)?;
        response.copy_from_slice(&self.receive_response(TXSDO)?.1[4..8]);
        Ok(response)
    }

    /// Sets a parameter according to ODrive endpoints.json
    ///
    /// # Arguments
    /// * `endpoint_id` - The endpoint id from an endpoints.json
    /// * `data` - up to 4 bytes of data to set for the endpoint according to endpoints.json
    ///
    /// # Returns
    /// Result indicating success or failure
    pub fn set_parameter(&mut self, endpoint_id: u16, data: &[u8]) -> nb::Result<(), EspTwaiError> {
        let parameters = Self::build_parameter_data(1, endpoint_id, data);
        self.send_command(RXSDO, &parameters)
    }

    /// Triggers an emergency stop on the ODrive
    ///
    /// # Returns
    /// Result indicating success or would-block
    pub fn estop(&mut self) -> nb::Result<(), EspTwaiError> {
        self.send_command(ESTOP, &[])
    }

    /// Triggers a reboot on the ODrive
    ///
    /// # Returns
    /// Result indicating success or failure
    pub fn reboot(&mut self) -> nb::Result<(), EspTwaiError> {
        self.send_command(REBOOT, &[])
    }

    /// Enters DFU Mode on the ODrive
    /// Equivalent to calling enter_dfu_mode2()
    ///
    /// # Returns
    /// Result indicating success or failure
    pub fn enter_dfu_mode(&mut self) -> nb::Result<(), EspTwaiError> {
        self.send_command(ENTER_DFU_MODE, &[])
    }

    /// Disables CAN bus on the ODrive asynchronously
    /// Only available on GIM6010-8 - could work on others, no guarantees
    ///
    /// # Returns
    /// Result indicating success or failure
    pub fn disable_can(&mut self) -> nb::Result<(), EspTwaiError> {
        self.send_command(DISABLE_CAN, &[])
    }

    /// Clears errors from the ODrive
    ///
    /// # Returns
    /// Result containing (active_errors, disarm_reason) or an error
    pub fn clear_errors(&mut self) -> nb::Result<(), EspTwaiError> {
        self.send_command(CLEAR_ERRORS, &[])
    }

    // Response parsing methods

    /// Helper method to send a request and parse the response
    fn request_and_parse<T, F>(&mut self, cmd: u8, parser: F) -> nb::Result<T, EspTwaiError>
    where
        F: FnOnce(&[u8; 8]) -> T,
    {
        self.send_command(cmd, &[])?;
        let (_, data) = self.receive_response(cmd)?;
        Ok(parser(&data))
    }

    /// Requests encoder estimates from the ODrive
    ///
    /// # Returns
    /// Result containing (position, velocity) or would-block/error
    pub fn get_encoder_estimates(&mut self) -> nb::Result<(f32, f32), EspTwaiError> {
        let (pos, vel) = self.request_and_parse(GET_ENCODER_ESTIMATES, Self::parse_two_f32)?;
        Ok((pos, vel))
    }

    /// Requests bus voltage and current measurements from the ODrive
    ///
    /// # Returns
    /// Result containing (voltage, current) or would-block/error
    pub fn get_voltages(&mut self) -> nb::Result<(f32, f32), EspTwaiError> {
        let (voltage, current) =
            self.request_and_parse(GET_BUS_VOLTAGE_CURRENT, Self::parse_two_f32)?;
        Ok((voltage, current))
    }

    /// Requests error information from the ODrive
    ///
    /// # Returns
    /// Result containing (active_errors, disarm_reason) or would-block/error
    pub fn get_errors(&mut self) -> nb::Result<(ODriveError, ODriveError), EspTwaiError> {
        let (active_errors, disarm_reason) =
            self.request_and_parse(GET_ERROR, Self::parse_two_odrive_errors)?;
        Ok((active_errors, disarm_reason))
    }

    /// Requests version information from the ODrive
    ///
    /// # Returns
    /// Result containing version information or would-block/error
    pub fn get_versions(&mut self) -> nb::Result<Versions, EspTwaiError> {
        let versions = self.request_and_parse(GET_VERSION, Self::parse_versions)?;
        Ok(versions)
    }

    /// Requests torque information from the ODrive
    ///
    /// # Returns
    /// Result containing (torque_target, torque_estimate) in Nm
    pub fn get_torques(&mut self) -> nb::Result<(f32, f32), EspTwaiError> {
        let (torque_target, torque_estimate) =
            self.request_and_parse(GET_TORQUES, Self::parse_two_f32)?;

        Ok((torque_target, torque_estimate))
    }

    /// Requests power information from the ODrive
    ///
    /// # Returns
    /// Result containing (electrical_power, mechanical_power) in W
    pub fn get_powers(&mut self) -> nb::Result<(f32, f32), EspTwaiError> {
        let (electrical_power, mechanical_power) =
            self.request_and_parse(GET_POWERS, Self::parse_two_f32)?;

        Ok((electrical_power, mechanical_power))
    }
}
