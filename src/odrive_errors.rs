use core::fmt;
/// ODrive error codes as bitflags
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ODriveError {
    bits: u32,
}

impl ODriveError {
    /// Documentation from
    /// <https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.Error>
    pub const INITIALIZING: Self = Self { bits: 0x1 };
    pub const SYSTEM_LEVEL: Self = Self { bits: 0x2 };
    pub const TIMING_ERROR: Self = Self { bits: 0x4 };
    pub const MISSING_ESTIMATE: Self = Self { bits: 0x8 };
    pub const BAD_CONFIG: Self = Self { bits: 0x10 };
    pub const DRV_FAULT: Self = Self { bits: 0x20 };
    pub const MISSING_INPUT: Self = Self { bits: 0x40 };
    pub const DC_BUS_OVER_VOLTAGE: Self = Self { bits: 0x100 };
    pub const DC_BUS_UNDER_VOLTAGE: Self = Self { bits: 0x200 };
    pub const DC_BUS_OVER_CURRENT: Self = Self { bits: 0x400 };
    pub const DC_BUS_OVER_REGEN_CURRENT: Self = Self { bits: 0x800 };
    pub const CURRENT_LIMIT_VIOLATION: Self = Self { bits: 0x1000 };
    pub const MOTOR_OVER_TEMP: Self = Self { bits: 0x2000 };
    pub const INVERTER_OVER_TEMP: Self = Self { bits: 0x4000 };
    pub const VELOCITY_LIMIT_VIOLATION: Self = Self { bits: 0x8000 };
    pub const POSITION_LIMIT_VIOLATION: Self = Self { bits: 0x10000 };
    pub const WATCHDOG_TIMER_EXPIRED: Self = Self { bits: 0x1000000 };
    pub const ESTOP_REQUESTED: Self = Self { bits: 0x2000000 };
    pub const SPINOUT_DETECTED: Self = Self { bits: 0x4000000 };
    pub const BRAKE_RESISTOR_DISARMED: Self = Self { bits: 0x8000000 };
    pub const THERMISTOR_DISCONNECTED: Self = Self { bits: 0x10000000 };
    pub const CALIBRATION_ERROR: Self = Self { bits: 0x40000000 };

    /// Create a new ODriveError from raw bits
    pub fn from_bits(bits: u32) -> Self {
        Self { bits }
    }

    /// Get the raw bits
    pub fn bits(self) -> u32 {
        self.bits
    }

    /// Check if a specific error is set
    pub fn contains(self, other: Self) -> bool {
        (self.bits & other.bits) == other.bits
    }

    /// Check if any error is set
    pub fn is_error(self) -> bool {
        self.bits != 0
    }

    /// Iterate over all set error flags
    pub fn iter(self) -> ODriveErrorIter {
        ODriveErrorIter {
            bits: self.bits,
            index: 0,
        }
    }
}

/// Iterator over set error flags
pub struct ODriveErrorIter {
    bits: u32,
    index: u8,
}

impl Iterator for ODriveErrorIter {
    type Item = ODriveError;

    fn next(&mut self) -> Option<Self::Item> {
        while self.index < 32 {
            let mask = 1 << self.index;
            self.index += 1;

            if (self.bits & mask) != 0 {
                return Some(ODriveError { bits: mask });
            }
        }
        None
    }
}

impl fmt::Display for ODriveError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.bits == 0 {
            return write!(f, "No error");
        }

        let mut first = true;

        for error in self.iter() {
            if !first {
                write!(f, ", ")?;
            }
            first = false;

            match error {
                ODriveError::INITIALIZING => write!(f, "Initializing"),
                ODriveError::SYSTEM_LEVEL => write!(f, "System level error"),
                ODriveError::TIMING_ERROR => write!(f, "Timing error"),
                ODriveError::MISSING_ESTIMATE => write!(f, "Missing estimate"),
                ODriveError::BAD_CONFIG => write!(f, "Bad config"),
                ODriveError::DRV_FAULT => write!(f, "Driver fault"),
                ODriveError::MISSING_INPUT => write!(f, "Missing input"),
                ODriveError::DC_BUS_OVER_VOLTAGE => write!(f, "DC bus over voltage"),
                ODriveError::DC_BUS_UNDER_VOLTAGE => write!(f, "DC bus under voltage"),
                ODriveError::DC_BUS_OVER_CURRENT => write!(f, "DC bus over current"),
                ODriveError::DC_BUS_OVER_REGEN_CURRENT => write!(f, "DC bus over regen current"),
                ODriveError::CURRENT_LIMIT_VIOLATION => write!(f, "Current limit violation"),
                ODriveError::MOTOR_OVER_TEMP => write!(f, "Motor over temperature"),
                ODriveError::INVERTER_OVER_TEMP => write!(f, "Inverter over temperature"),
                ODriveError::VELOCITY_LIMIT_VIOLATION => write!(f, "Velocity limit violation"),
                ODriveError::POSITION_LIMIT_VIOLATION => write!(f, "Position limit violation"),
                ODriveError::WATCHDOG_TIMER_EXPIRED => write!(f, "Watchdog timer expired"),
                ODriveError::ESTOP_REQUESTED => write!(f, "E-stop requested"),
                ODriveError::SPINOUT_DETECTED => write!(f, "Spinout detected"),
                ODriveError::BRAKE_RESISTOR_DISARMED => write!(f, "Brake resistor disarmed"),
                ODriveError::THERMISTOR_DISCONNECTED => write!(f, "Thermistor disconnected"),
                ODriveError::CALIBRATION_ERROR => write!(f, "Calibration error"),
                _ => write!(f, "Unknown error (0x{:x})", error.bits),
            }?;
        }

        Ok(())
    }
}
