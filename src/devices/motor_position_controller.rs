// phidget-rs/src/devices/motor_position_controller.rs
//
// Copyright (c) 2023-2025, Frank Pagliughi
//
// This file is part of the 'phidget-rs' library.
//
// Licensed under the MIT license:
//   <LICENSE or http://opensource.org/licenses/MIT>
// This file may not be copied, modified, or distributed except according
// to those terms.
//

use crate::devices::dc_motor::FanMode;
use crate::devices::encoder::EncoderIoMode;
use crate::{Error, Phidget, Result, ReturnCode};
use phidget_sys::{
    self as ffi,
    PhidgetHandle,
    PhidgetMotorPositionControllerHandle as MotorPositionControllerHandle,
};
use std::{
    ffi::{c_int, c_uint, c_void},
    mem, ptr,
    time::Duration,
};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Position type for the motor position controller
#[derive(Copy, Clone, Debug, Eq, Ord, PartialEq, PartialOrd, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[repr(u32)]
pub enum PositionType {
    /// Encoder-based position sensing.
    Encoder = ffi::Phidget_PositionType_POSITION_TYPE_ENCODER,
    /// Hall sensor-based position sensing.
    HallSensor = ffi::Phidget_PositionType_POSITION_TYPE_HALL_SENSOR,
}

impl TryFrom<u32> for PositionType {
    type Error = Error;

    fn try_from(val: u32) -> Result<Self> {
        use PositionType::*;
        match val {
            ffi::Phidget_PositionType_POSITION_TYPE_ENCODER => Ok(Encoder),
            ffi::Phidget_PositionType_POSITION_TYPE_HALL_SENSOR => Ok(HallSensor),
            _ => Err(ReturnCode::UnknownVal),
        }
    }
}

/////////////////////////////////////////////////////////////////////////////

/// The function type for the safe Rust attach callback.
pub type AttachCallback = dyn Fn(&mut MotorPositionController) + Send + 'static;

/// The function type for the safe Rust detach callback.
pub type DetachCallback = dyn Fn(&mut MotorPositionController) + Send + 'static;

/// The function type for the safe Rust duty cycle update callback.
pub type DutyCycleUpdateCallback = dyn Fn(&MotorPositionController, f64) + Send + 'static;

/// The function type for the safe Rust expected position change callback.
pub type ExpectedPositionChangeCallback = dyn Fn(&MotorPositionController, f64) + Send + 'static;

/// The function type for the safe Rust position change callback.
pub type PositionChangeCallback = dyn Fn(&MotorPositionController, f64) + Send + 'static;

/// Phidget Motor Position Controller channel
pub struct MotorPositionController {
    // Handle to the channel for the phidget22 library
    chan: MotorPositionControllerHandle,
    // Double-boxed callback, if registered
    cb: Option<*mut c_void>,
    // Double-boxed attach callback, if registered
    attach_cb: Option<*mut c_void>,
    // Double-boxed detach callback, if registered
    detach_cb: Option<*mut c_void>,
}

impl MotorPositionController {
    /// Create a new Motor Position Controller channel.
    pub fn new() -> Self {
        let mut chan: MotorPositionControllerHandle = ptr::null_mut();
        unsafe {
            ffi::PhidgetMotorPositionController_create(&mut chan);
        }
        Self::from(chan)
    }

    // Low-level, unsafe callback for device attach events
    unsafe extern "C" fn on_attach(phid: PhidgetHandle, ctx: *mut c_void) {
        if !ctx.is_null() {
            let cb: &mut Box<AttachCallback> = &mut *(ctx as *mut _);
            let mut dev = Self::from(phid as MotorPositionControllerHandle);
            cb(&mut dev);
            mem::forget(dev);
        }
    }

    // Low-level, unsafe callback for device detach events
    unsafe extern "C" fn on_detach(phid: PhidgetHandle, ctx: *mut c_void) {
        if !ctx.is_null() {
            let cb: &mut Box<DetachCallback> = &mut *(ctx as *mut _);
            let mut dev = Self::from(phid as MotorPositionControllerHandle);
            cb(&mut dev);
            mem::forget(dev);
        }
    }

    /// Get a reference to the underlying channel handle
    pub fn as_channel(&self) -> &MotorPositionControllerHandle {
        &self.chan
    }

    // ----- Failsafe -----

    /// Enable failsafe for the channel with the specified failsafe time.
    pub fn enable_failsafe(&self, failsafe_time: Duration) -> Result<()> {
        let ms = u32::try_from(failsafe_time.as_millis()).map_err(|_| ReturnCode::InvalidArg)?;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_enableFailsafe(self.chan, ms)
        })?;
        Ok(())
    }

    /// Reset failsafe.
    pub fn reset_failsafe(&self) -> Result<()> {
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_resetFailsafe(self.chan)
        })?;
        Ok(())
    }

    /// Get minimum failsafe time.
    pub fn min_failsafe_time(&self) -> Result<Duration> {
        let mut val: u32 = 0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getMinFailsafeTime(self.chan, &mut val)
        })?;
        Ok(Duration::from_millis(val.into()))
    }

    /// Get maximum failsafe time.
    pub fn max_failsafe_time(&self) -> Result<Duration> {
        let mut val: u32 = 0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getMaxFailsafeTime(self.chan, &mut val)
        })?;
        Ok(Duration::from_millis(val.into()))
    }

    // ----- Position offset -----

    /// Add position offset.
    pub fn add_position_offset(&self, position_offset: f64) -> Result<()> {
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_addPositionOffset(self.chan, position_offset)
        })?;
        Ok(())
    }

    // ----- Acceleration -----

    /// Set acceleration.
    pub fn set_acceleration(&self, acceleration: f64) -> Result<()> {
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_setAcceleration(self.chan, acceleration)
        })?;
        Ok(())
    }

    /// Get acceleration.
    pub fn acceleration(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getAcceleration(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Get minimum acceleration.
    pub fn min_acceleration(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getMinAcceleration(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Get maximum acceleration.
    pub fn max_acceleration(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getMaxAcceleration(self.chan, &mut value)
        })?;
        Ok(value)
    }

    // ----- Current limit -----

    /// Get active current limit.
    pub fn active_current_limit(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getActiveCurrentLimit(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Set current limit.
    pub fn set_current_limit(&self, current_limit: f64) -> Result<()> {
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_setCurrentLimit(self.chan, current_limit)
        })?;
        Ok(())
    }

    /// Get current limit.
    pub fn current_limit(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getCurrentLimit(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Get minimum current limit.
    pub fn min_current_limit(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getMinCurrentLimit(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Get maximum current limit.
    pub fn max_current_limit(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getMaxCurrentLimit(self.chan, &mut value)
        })?;
        Ok(value)
    }

    // ----- Current regulator gain -----

    /// Set current regulator gain.
    pub fn set_current_regulator_gain(&self, gain: f64) -> Result<()> {
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_setCurrentRegulatorGain(self.chan, gain)
        })?;
        Ok(())
    }

    /// Get current regulator gain.
    pub fn current_regulator_gain(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getCurrentRegulatorGain(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Get minimum current regulator gain.
    pub fn min_current_regulator_gain(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getMinCurrentRegulatorGain(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Get maximum current regulator gain.
    pub fn max_current_regulator_gain(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getMaxCurrentRegulatorGain(self.chan, &mut value)
        })?;
        Ok(value)
    }

    // ----- Data interval / rate -----

    /// Set data interval in milliseconds.
    pub fn set_data_interval(&self, data_interval: u32) -> Result<()> {
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_setDataInterval(self.chan, data_interval)
        })?;
        Ok(())
    }

    /// Get data interval in milliseconds.
    pub fn data_interval(&self) -> Result<u32> {
        let mut value = 0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getDataInterval(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Get minimum data interval.
    pub fn min_data_interval(&self) -> Result<u32> {
        let mut value = 0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getMinDataInterval(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Get maximum data interval.
    pub fn max_data_interval(&self) -> Result<u32> {
        let mut value = 0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getMaxDataInterval(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Set data rate in Hz.
    pub fn set_data_rate(&self, data_rate: f64) -> Result<()> {
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_setDataRate(self.chan, data_rate)
        })?;
        Ok(())
    }

    /// Get data rate in Hz.
    pub fn data_rate(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getDataRate(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Get minimum data rate.
    pub fn min_data_rate(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getMinDataRate(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Get maximum data rate.
    pub fn max_data_rate(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getMaxDataRate(self.chan, &mut value)
        })?;
        Ok(value)
    }

    // ----- Dead band -----

    /// Set dead band.
    pub fn set_dead_band(&self, dead_band: f64) -> Result<()> {
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_setDeadBand(self.chan, dead_band)
        })?;
        Ok(())
    }

    /// Get dead band.
    pub fn dead_band(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getDeadBand(self.chan, &mut value)
        })?;
        Ok(value)
    }

    // ----- Duty cycle -----

    /// Get duty cycle.
    pub fn duty_cycle(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getDutyCycle(self.chan, &mut value)
        })?;
        Ok(value)
    }

    // ----- Engaged -----

    /// Set engaged.
    pub fn set_engaged(&self, engaged: bool) -> Result<()> {
        let value = engaged as c_int;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_setEngaged(self.chan, value)
        })?;
        Ok(())
    }

    /// Get engaged.
    pub fn engaged(&self) -> Result<bool> {
        let mut value: c_int = 0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getEngaged(self.chan, &mut value)
        })?;
        Ok(value != 0)
    }

    // ----- Expected position -----

    /// Get expected position.
    pub fn expected_position(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getExpectedPosition(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Set enable expected position.
    pub fn set_enable_expected_position(&self, enabled: bool) -> Result<()> {
        let value = enabled as c_int;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_setEnableExpectedPosition(self.chan, value)
        })?;
        Ok(())
    }

    /// Get enable expected position.
    pub fn enable_expected_position(&self) -> Result<bool> {
        let mut value: c_int = 0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getEnableExpectedPosition(self.chan, &mut value)
        })?;
        Ok(value != 0)
    }

    // ----- Failsafe braking / current -----

    /// Set failsafe braking enabled.
    pub fn set_failsafe_braking_enabled(&self, enabled: bool) -> Result<()> {
        let value = enabled as c_int;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_setFailsafeBrakingEnabled(self.chan, value)
        })?;
        Ok(())
    }

    /// Get failsafe braking enabled.
    pub fn failsafe_braking_enabled(&self) -> Result<bool> {
        let mut value: c_int = 0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getFailsafeBrakingEnabled(self.chan, &mut value)
        })?;
        Ok(value != 0)
    }

    /// Set failsafe current limit.
    pub fn set_failsafe_current_limit(&self, limit: f64) -> Result<()> {
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_setFailsafeCurrentLimit(self.chan, limit)
        })?;
        Ok(())
    }

    /// Get failsafe current limit.
    pub fn failsafe_current_limit(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getFailsafeCurrentLimit(self.chan, &mut value)
        })?;
        Ok(value)
    }

    // ----- Fan mode -----

    /// Set fan mode.
    pub fn set_fan_mode(&self, fan_mode: FanMode) -> Result<()> {
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_setFanMode(self.chan, fan_mode as c_uint)
        })?;
        Ok(())
    }

    /// Get fan mode.
    pub fn fan_mode(&self) -> Result<FanMode> {
        let mut fm: ffi::Phidget_FanMode = 0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getFanMode(self.chan, &mut fm)
        })?;
        FanMode::try_from(fm)
    }

    // ----- Inductance -----

    /// Set inductance.
    pub fn set_inductance(&self, inductance: f64) -> Result<()> {
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_setInductance(self.chan, inductance)
        })?;
        Ok(())
    }

    /// Get inductance.
    pub fn inductance(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getInductance(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Get minimum inductance.
    pub fn min_inductance(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getMinInductance(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Get maximum inductance.
    pub fn max_inductance(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getMaxInductance(self.chan, &mut value)
        })?;
        Ok(value)
    }

    // ----- IO mode -----

    /// Set encoder I/O mode.
    pub fn set_io_mode(&self, io_mode: EncoderIoMode) -> Result<()> {
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_setIOMode(self.chan, io_mode as c_uint)
        })?;
        Ok(())
    }

    /// Get encoder I/O mode.
    pub fn io_mode(&self) -> Result<EncoderIoMode> {
        let mut mode: ffi::Phidget_EncoderIOMode = 0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getIOMode(self.chan, &mut mode)
        })?;
        EncoderIoMode::try_from(mode)
    }

    // ----- PID -----

    /// Set derivative gain (Kd).
    pub fn set_kd(&self, kd: f64) -> Result<()> {
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_setKd(self.chan, kd)
        })?;
        Ok(())
    }

    /// Get derivative gain (Kd).
    pub fn kd(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getKd(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Set integral gain (Ki).
    pub fn set_ki(&self, ki: f64) -> Result<()> {
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_setKi(self.chan, ki)
        })?;
        Ok(())
    }

    /// Get integral gain (Ki).
    pub fn ki(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getKi(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Set proportional gain (Kp).
    pub fn set_kp(&self, kp: f64) -> Result<()> {
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_setKp(self.chan, kp)
        })?;
        Ok(())
    }

    /// Get proportional gain (Kp).
    pub fn kp(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getKp(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Set normalize PID.
    pub fn set_normalize_pid(&self, normalize: bool) -> Result<()> {
        let value = normalize as c_int;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_setNormalizePID(self.chan, value)
        })?;
        Ok(())
    }

    /// Get normalize PID.
    pub fn normalize_pid(&self) -> Result<bool> {
        let mut value: c_int = 0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getNormalizePID(self.chan, &mut value)
        })?;
        Ok(value != 0)
    }

    // ----- Position -----

    /// Get position.
    pub fn position(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getPosition(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Get minimum position.
    pub fn min_position(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getMinPosition(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Get maximum position.
    pub fn max_position(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getMaxPosition(self.chan, &mut value)
        })?;
        Ok(value)
    }

    // ----- Position type -----

    /// Set position type.
    pub fn set_position_type(&self, position_type: PositionType) -> Result<()> {
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_setPositionType(
                self.chan,
                position_type as c_uint,
            )
        })?;
        Ok(())
    }

    /// Get position type.
    pub fn position_type(&self) -> Result<PositionType> {
        let mut pt: ffi::Phidget_PositionType = 0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getPositionType(self.chan, &mut pt)
        })?;
        PositionType::try_from(pt)
    }

    // ----- Rescale factor -----

    /// Set rescale factor.
    pub fn set_rescale_factor(&self, rescale_factor: f64) -> Result<()> {
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_setRescaleFactor(self.chan, rescale_factor)
        })?;
        Ok(())
    }

    /// Get rescale factor.
    pub fn rescale_factor(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getRescaleFactor(self.chan, &mut value)
        })?;
        Ok(value)
    }

    // ----- Stall velocity -----

    /// Set stall velocity.
    pub fn set_stall_velocity(&self, stall_velocity: f64) -> Result<()> {
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_setStallVelocity(self.chan, stall_velocity)
        })?;
        Ok(())
    }

    /// Get stall velocity.
    pub fn stall_velocity(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getStallVelocity(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Get minimum stall velocity.
    pub fn min_stall_velocity(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getMinStallVelocity(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Get maximum stall velocity.
    pub fn max_stall_velocity(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getMaxStallVelocity(self.chan, &mut value)
        })?;
        Ok(value)
    }

    // ----- Surge current limit -----

    /// Set surge current limit.
    pub fn set_surge_current_limit(&self, limit: f64) -> Result<()> {
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_setSurgeCurrentLimit(self.chan, limit)
        })?;
        Ok(())
    }

    /// Get surge current limit.
    pub fn surge_current_limit(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getSurgeCurrentLimit(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Get minimum surge current limit.
    pub fn min_surge_current_limit(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getMinSurgeCurrentLimit(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Get maximum surge current limit.
    pub fn max_surge_current_limit(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getMaxSurgeCurrentLimit(self.chan, &mut value)
        })?;
        Ok(value)
    }

    // ----- Target position -----

    /// Set target position.
    pub fn set_target_position(&self, target_position: f64) -> Result<()> {
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_setTargetPosition(self.chan, target_position)
        })?;
        Ok(())
    }

    /// Get target position.
    pub fn target_position(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getTargetPosition(self.chan, &mut value)
        })?;
        Ok(value)
    }

    // ----- Velocity limit -----

    /// Set velocity limit.
    pub fn set_velocity_limit(&self, velocity_limit: f64) -> Result<()> {
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_setVelocityLimit(self.chan, velocity_limit)
        })?;
        Ok(())
    }

    /// Get velocity limit.
    pub fn velocity_limit(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getVelocityLimit(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Get minimum velocity limit.
    pub fn min_velocity_limit(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getMinVelocityLimit(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Get maximum velocity limit.
    pub fn max_velocity_limit(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_getMaxVelocityLimit(self.chan, &mut value)
        })?;
        Ok(value)
    }

    // ----- Callbacks -----

    // Low-level, unsafe, callback for duty cycle update events.
    unsafe extern "C" fn on_duty_cycle_update(
        chan: MotorPositionControllerHandle,
        ctx: *mut c_void,
        duty_cycle: f64,
    ) {
        if !ctx.is_null() {
            let cb: &mut Box<DutyCycleUpdateCallback> = &mut *(ctx as *mut _);
            let dev = Self::from(chan);
            cb(&dev, duty_cycle);
            mem::forget(dev);
        }
    }

    /// Set a handler to receive duty cycle update callbacks.
    pub fn set_on_duty_cycle_update_handler<F>(&mut self, cb: F) -> Result<()>
    where
        F: Fn(&MotorPositionController, f64) + Send + 'static,
    {
        let cb: Box<Box<DutyCycleUpdateCallback>> = Box::new(Box::new(cb));
        let ctx = Box::into_raw(cb) as *mut c_void;
        self.cb = Some(ctx);

        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_setOnDutyCycleUpdateHandler(
                self.chan,
                Some(Self::on_duty_cycle_update),
                ctx,
            )
        })
    }

    // Low-level, unsafe, callback for expected position change events.
    unsafe extern "C" fn on_expected_position_change(
        chan: MotorPositionControllerHandle,
        ctx: *mut c_void,
        expected_position: f64,
    ) {
        if !ctx.is_null() {
            let cb: &mut Box<ExpectedPositionChangeCallback> = &mut *(ctx as *mut _);
            let dev = Self::from(chan);
            cb(&dev, expected_position);
            mem::forget(dev);
        }
    }

    /// Set a handler to receive expected position change callbacks.
    pub fn set_on_expected_position_change_handler<F>(&mut self, cb: F) -> Result<()>
    where
        F: Fn(&MotorPositionController, f64) + Send + 'static,
    {
        let cb: Box<Box<ExpectedPositionChangeCallback>> = Box::new(Box::new(cb));
        let ctx = Box::into_raw(cb) as *mut c_void;
        self.cb = Some(ctx);

        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_setOnExpectedPositionChangeHandler(
                self.chan,
                Some(Self::on_expected_position_change),
                ctx,
            )
        })
    }

    // Low-level, unsafe, callback for position change events.
    unsafe extern "C" fn on_position_change(
        chan: MotorPositionControllerHandle,
        ctx: *mut c_void,
        position: f64,
    ) {
        if !ctx.is_null() {
            let cb: &mut Box<PositionChangeCallback> = &mut *(ctx as *mut _);
            let dev = Self::from(chan);
            cb(&dev, position);
            mem::forget(dev);
        }
    }

    /// Set a handler to receive position change callbacks.
    pub fn set_on_position_change_handler<F>(&mut self, cb: F) -> Result<()>
    where
        F: Fn(&MotorPositionController, f64) + Send + 'static,
    {
        let cb: Box<Box<PositionChangeCallback>> = Box::new(Box::new(cb));
        let ctx = Box::into_raw(cb) as *mut c_void;
        self.cb = Some(ctx);

        ReturnCode::result(unsafe {
            ffi::PhidgetMotorPositionController_setOnPositionChangeHandler(
                self.chan,
                Some(Self::on_position_change),
                ctx,
            )
        })
    }

    // ----- Attach / Detach -----

    /// Sets a handler to receive attach callbacks.
    pub fn set_on_attach_handler<F>(&mut self, cb: F) -> Result<()>
    where
        F: Fn(&mut MotorPositionController) + Send + 'static,
    {
        let cb: Box<Box<AttachCallback>> = Box::new(Box::new(cb));
        let ctx = Box::into_raw(cb) as *mut c_void;

        ReturnCode::result(unsafe {
            ffi::Phidget_setOnAttachHandler(self.as_mut_handle(), Some(Self::on_attach), ctx)
        })?;
        self.attach_cb = Some(ctx);
        Ok(())
    }

    /// Sets a handler to receive detach callbacks.
    pub fn set_on_detach_handler<F>(&mut self, cb: F) -> Result<()>
    where
        F: Fn(&mut MotorPositionController) + Send + 'static,
    {
        let cb: Box<Box<DetachCallback>> = Box::new(Box::new(cb));
        let ctx = Box::into_raw(cb) as *mut c_void;

        ReturnCode::result(unsafe {
            ffi::Phidget_setOnDetachHandler(self.as_mut_handle(), Some(Self::on_detach), ctx)
        })?;
        self.detach_cb = Some(ctx);
        Ok(())
    }
}

impl Phidget for MotorPositionController {
    fn as_mut_handle(&mut self) -> PhidgetHandle {
        self.chan as PhidgetHandle
    }

    fn as_handle(&self) -> PhidgetHandle {
        self.chan as PhidgetHandle
    }
}

unsafe impl Send for MotorPositionController {}

impl Default for MotorPositionController {
    fn default() -> Self {
        Self::new()
    }
}

impl From<MotorPositionControllerHandle> for MotorPositionController {
    fn from(chan: MotorPositionControllerHandle) -> Self {
        Self {
            chan,
            cb: None,
            attach_cb: None,
            detach_cb: None,
        }
    }
}

impl Drop for MotorPositionController {
    fn drop(&mut self) {
        if let Ok(true) = self.is_open() {
            let _ = self.close();
        }
        unsafe {
            ffi::PhidgetMotorPositionController_delete(&mut self.chan);
            crate::drop_cb::<DutyCycleUpdateCallback>(self.cb.take());
            crate::drop_cb::<ExpectedPositionChangeCallback>(self.cb.take());
            crate::drop_cb::<PositionChangeCallback>(self.cb.take());
            crate::drop_cb::<AttachCallback>(self.attach_cb.take());
            crate::drop_cb::<DetachCallback>(self.detach_cb.take());
        }
    }
}
