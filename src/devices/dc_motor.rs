// phidget-rs/src/devices/dc_motor.rs
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

use crate::{Error, Phidget, Result, ReturnCode};
use phidget_sys::{self as ffi, PhidgetDCMotorHandle as DcMotorHandle, PhidgetHandle};
use std::{
    ffi::{c_int, c_uint, c_void},
    mem, ptr,
    time::Duration,
};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Drive mode for DC motor
#[derive(Copy, Clone, Debug, Eq, Ord, PartialEq, PartialOrd, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[repr(u32)]
pub enum DriveMode {
    /// Motor will coast when stopped.
    Coast = ffi::Phidget_DriveMode_DRIVE_MODE_COAST,
    /// Motor will be held (braked) when stopped.
    Forced = ffi::Phidget_DriveMode_DRIVE_MODE_FORCED,
}

impl TryFrom<u32> for DriveMode {
    type Error = Error;

    fn try_from(val: u32) -> Result<Self> {
        use DriveMode::*;
        match val {
            ffi::Phidget_DriveMode_DRIVE_MODE_COAST => Ok(Coast),
            ffi::Phidget_DriveMode_DRIVE_MODE_FORCED => Ok(Forced),
            _ => Err(ReturnCode::UnknownVal),
        }
    }
}

/// Fan mode for motor controllers with integrated fans
#[derive(Copy, Clone, Debug, Eq, Ord, PartialEq, PartialOrd, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[repr(u32)]
pub enum FanMode {
    /// Fan is off.
    Off = ffi::Phidget_FanMode_FAN_MODE_OFF,
    /// Fan is always on.
    On = ffi::Phidget_FanMode_FAN_MODE_ON,
    /// Fan is automatically controlled based on temperature.
    Auto = ffi::Phidget_FanMode_FAN_MODE_AUTO,
}

impl TryFrom<u32> for FanMode {
    type Error = Error;

    fn try_from(val: u32) -> Result<Self> {
        use FanMode::*;
        match val {
            ffi::Phidget_FanMode_FAN_MODE_OFF => Ok(Off),
            ffi::Phidget_FanMode_FAN_MODE_ON => Ok(On),
            ffi::Phidget_FanMode_FAN_MODE_AUTO => Ok(Auto),
            _ => Err(ReturnCode::UnknownVal),
        }
    }
}

/////////////////////////////////////////////////////////////////////////////

/// The function type for the safe Rust attach callback.
pub type AttachCallback = dyn Fn(&mut DcMotor) + Send + 'static;

/// The function type for the safe Rust detach callback.
pub type DetachCallback = dyn Fn(&mut DcMotor) + Send + 'static;

/// The function type for the safe Rust back EMF change callback.
pub type BackEmfChangeCallback = dyn Fn(&DcMotor, f64) + Send + 'static;

/// The function type for the safe Rust braking strength change callback.
pub type BrakingStrengthChangeCallback = dyn Fn(&DcMotor, f64) + Send + 'static;

/// The function type for the safe Rust velocity update callback.
pub type VelocityUpdateCallback = dyn Fn(&DcMotor, f64) + Send + 'static;

/// Phidget DC Motor channel
pub struct DcMotor {
    // Handle to the channel for the phidget22 library
    chan: DcMotorHandle,
    // Double-boxed callback, if registered
    cb: Option<*mut c_void>,
    // Double-boxed attach callback, if registered
    attach_cb: Option<*mut c_void>,
    // Double-boxed detach callback, if registered
    detach_cb: Option<*mut c_void>,
}

impl DcMotor {
    /// Create a new DC Motor channel.
    pub fn new() -> Self {
        let mut chan: DcMotorHandle = ptr::null_mut();
        unsafe {
            ffi::PhidgetDCMotor_create(&mut chan);
        }
        Self::from(chan)
    }

    // Low-level, unsafe callback for device attach events
    unsafe extern "C" fn on_attach(phid: PhidgetHandle, ctx: *mut c_void) {
        if !ctx.is_null() {
            let cb: &mut Box<AttachCallback> = &mut *(ctx as *mut _);
            let mut dev = Self::from(phid as DcMotorHandle);
            cb(&mut dev);
            mem::forget(dev);
        }
    }

    // Low-level, unsafe callback for device detach events
    unsafe extern "C" fn on_detach(phid: PhidgetHandle, ctx: *mut c_void) {
        if !ctx.is_null() {
            let cb: &mut Box<DetachCallback> = &mut *(ctx as *mut _);
            let mut dev = Self::from(phid as DcMotorHandle);
            cb(&mut dev);
            mem::forget(dev);
        }
    }

    /// Get a reference to the underlying channel handle
    pub fn as_channel(&self) -> &DcMotorHandle {
        &self.chan
    }

    // ----- Failsafe -----

    /// Enable failsafe for the channel with the specified failsafe time.
    pub fn enable_failsafe(&self, failsafe_time: Duration) -> Result<()> {
        let ms = u32::try_from(failsafe_time.as_millis()).map_err(|_| ReturnCode::InvalidArg)?;
        ReturnCode::result(unsafe { ffi::PhidgetDCMotor_enableFailsafe(self.chan, ms) })?;
        Ok(())
    }

    /// Reset failsafe.
    pub fn reset_failsafe(&self) -> Result<()> {
        ReturnCode::result(unsafe { ffi::PhidgetDCMotor_resetFailsafe(self.chan) })?;
        Ok(())
    }

    /// Get minimum failsafe time.
    pub fn min_failsafe_time(&self) -> Result<Duration> {
        let mut val: u32 = 0;
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_getMinFailsafeTime(self.chan, &mut val)
        })?;
        Ok(Duration::from_millis(val.into()))
    }

    /// Get maximum failsafe time.
    pub fn max_failsafe_time(&self) -> Result<Duration> {
        let mut val: u32 = 0;
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_getMaxFailsafeTime(self.chan, &mut val)
        })?;
        Ok(Duration::from_millis(val.into()))
    }

    // ----- Acceleration -----

    /// Set acceleration.
    pub fn set_acceleration(&self, acceleration: f64) -> Result<()> {
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_setAcceleration(self.chan, acceleration)
        })?;
        Ok(())
    }

    /// Get acceleration.
    pub fn acceleration(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_getAcceleration(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Get minimum acceleration.
    pub fn min_acceleration(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_getMinAcceleration(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Get maximum acceleration.
    pub fn max_acceleration(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_getMaxAcceleration(self.chan, &mut value)
        })?;
        Ok(value)
    }

    // ----- Back EMF -----

    /// Get back EMF value.
    pub fn back_emf(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe { ffi::PhidgetDCMotor_getBackEMF(self.chan, &mut value) })?;
        Ok(value)
    }

    /// Set back EMF sensing state.
    pub fn set_back_emf_sensing_state(&self, state: bool) -> Result<()> {
        let value = state as c_int;
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_setBackEMFSensingState(self.chan, value)
        })?;
        Ok(())
    }

    /// Get back EMF sensing state.
    pub fn back_emf_sensing_state(&self) -> Result<bool> {
        let mut value: c_int = 0;
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_getBackEMFSensingState(self.chan, &mut value)
        })?;
        Ok(value != 0)
    }

    // ----- Braking -----

    /// Set braking enabled.
    pub fn set_braking_enabled(&self, enabled: bool) -> Result<()> {
        let value = enabled as c_int;
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_setBrakingEnabled(self.chan, value)
        })?;
        Ok(())
    }

    /// Get braking enabled.
    pub fn braking_enabled(&self) -> Result<bool> {
        let mut value: c_int = 0;
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_getBrakingEnabled(self.chan, &mut value)
        })?;
        Ok(value != 0)
    }

    /// Get braking strength.
    pub fn braking_strength(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_getBrakingStrength(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Get minimum braking strength.
    pub fn min_braking_strength(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_getMinBrakingStrength(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Get maximum braking strength.
    pub fn max_braking_strength(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_getMaxBrakingStrength(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Set target braking strength.
    pub fn set_target_braking_strength(&self, target_braking_strength: f64) -> Result<()> {
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_setTargetBrakingStrength(self.chan, target_braking_strength)
        })?;
        Ok(())
    }

    /// Get target braking strength.
    pub fn target_braking_strength(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_getTargetBrakingStrength(self.chan, &mut value)
        })?;
        Ok(value)
    }

    // ----- Current -----

    /// Get active current limit.
    pub fn active_current_limit(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_getActiveCurrentLimit(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Set current limit.
    pub fn set_current_limit(&self, current_limit: f64) -> Result<()> {
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_setCurrentLimit(self.chan, current_limit)
        })?;
        Ok(())
    }

    /// Get current limit.
    pub fn current_limit(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_getCurrentLimit(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Get minimum current limit.
    pub fn min_current_limit(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_getMinCurrentLimit(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Get maximum current limit.
    pub fn max_current_limit(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_getMaxCurrentLimit(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Set current regulator gain.
    pub fn set_current_regulator_gain(&self, gain: f64) -> Result<()> {
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_setCurrentRegulatorGain(self.chan, gain)
        })?;
        Ok(())
    }

    /// Get current regulator gain.
    pub fn current_regulator_gain(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_getCurrentRegulatorGain(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Get minimum current regulator gain.
    pub fn min_current_regulator_gain(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_getMinCurrentRegulatorGain(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Get maximum current regulator gain.
    pub fn max_current_regulator_gain(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_getMaxCurrentRegulatorGain(self.chan, &mut value)
        })?;
        Ok(value)
    }

    // ----- Data interval / rate -----

    /// Set data interval in milliseconds.
    pub fn set_data_interval(&self, data_interval: u32) -> Result<()> {
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_setDataInterval(self.chan, data_interval)
        })?;
        Ok(())
    }

    /// Get data interval in milliseconds.
    pub fn data_interval(&self) -> Result<u32> {
        let mut value = 0;
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_getDataInterval(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Get minimum data interval.
    pub fn min_data_interval(&self) -> Result<u32> {
        let mut value = 0;
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_getMinDataInterval(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Get maximum data interval.
    pub fn max_data_interval(&self) -> Result<u32> {
        let mut value = 0;
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_getMaxDataInterval(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Set data rate in Hz.
    pub fn set_data_rate(&self, data_rate: f64) -> Result<()> {
        ReturnCode::result(unsafe { ffi::PhidgetDCMotor_setDataRate(self.chan, data_rate) })?;
        Ok(())
    }

    /// Get data rate in Hz.
    pub fn data_rate(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe { ffi::PhidgetDCMotor_getDataRate(self.chan, &mut value) })?;
        Ok(value)
    }

    /// Get minimum data rate.
    pub fn min_data_rate(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_getMinDataRate(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Get maximum data rate.
    pub fn max_data_rate(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_getMaxDataRate(self.chan, &mut value)
        })?;
        Ok(value)
    }

    // ----- Drive mode -----

    /// Set drive mode.
    pub fn set_drive_mode(&self, drive_mode: DriveMode) -> Result<()> {
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_setDriveMode(self.chan, drive_mode as c_uint)
        })?;
        Ok(())
    }

    /// Get drive mode.
    pub fn drive_mode(&self) -> Result<DriveMode> {
        let mut dm: ffi::Phidget_DriveMode = 0;
        ReturnCode::result(unsafe { ffi::PhidgetDCMotor_getDriveMode(self.chan, &mut dm) })?;
        DriveMode::try_from(dm)
    }

    // ----- Fan mode -----

    /// Set fan mode.
    pub fn set_fan_mode(&self, fan_mode: FanMode) -> Result<()> {
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_setFanMode(self.chan, fan_mode as c_uint)
        })?;
        Ok(())
    }

    /// Get fan mode.
    pub fn fan_mode(&self) -> Result<FanMode> {
        let mut fm: ffi::Phidget_FanMode = 0;
        ReturnCode::result(unsafe { ffi::PhidgetDCMotor_getFanMode(self.chan, &mut fm) })?;
        FanMode::try_from(fm)
    }

    // ----- Failsafe braking / current -----

    /// Set failsafe braking enabled.
    pub fn set_failsafe_braking_enabled(&self, enabled: bool) -> Result<()> {
        let value = enabled as c_int;
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_setFailsafeBrakingEnabled(self.chan, value)
        })?;
        Ok(())
    }

    /// Get failsafe braking enabled.
    pub fn failsafe_braking_enabled(&self) -> Result<bool> {
        let mut value: c_int = 0;
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_getFailsafeBrakingEnabled(self.chan, &mut value)
        })?;
        Ok(value != 0)
    }

    /// Set failsafe current limit.
    pub fn set_failsafe_current_limit(&self, limit: f64) -> Result<()> {
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_setFailsafeCurrentLimit(self.chan, limit)
        })?;
        Ok(())
    }

    /// Get failsafe current limit.
    pub fn failsafe_current_limit(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_getFailsafeCurrentLimit(self.chan, &mut value)
        })?;
        Ok(value)
    }

    // ----- Inductance -----

    /// Set inductance.
    pub fn set_inductance(&self, inductance: f64) -> Result<()> {
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_setInductance(self.chan, inductance)
        })?;
        Ok(())
    }

    /// Get inductance.
    pub fn inductance(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_getInductance(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Get minimum inductance.
    pub fn min_inductance(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_getMinInductance(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Get maximum inductance.
    pub fn max_inductance(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_getMaxInductance(self.chan, &mut value)
        })?;
        Ok(value)
    }

    // ----- Surge current -----

    /// Set surge current limit.
    pub fn set_surge_current_limit(&self, limit: f64) -> Result<()> {
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_setSurgeCurrentLimit(self.chan, limit)
        })?;
        Ok(())
    }

    /// Get surge current limit.
    pub fn surge_current_limit(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_getSurgeCurrentLimit(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Get minimum surge current limit.
    pub fn min_surge_current_limit(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_getMinSurgeCurrentLimit(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Get maximum surge current limit.
    pub fn max_surge_current_limit(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_getMaxSurgeCurrentLimit(self.chan, &mut value)
        })?;
        Ok(value)
    }

    // ----- Velocity -----

    /// Set target velocity (-1.0 to 1.0).
    pub fn set_target_velocity(&self, target_velocity: f64) -> Result<()> {
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_setTargetVelocity(self.chan, target_velocity)
        })?;
        Ok(())
    }

    /// Get target velocity.
    pub fn target_velocity(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_getTargetVelocity(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Get current velocity.
    pub fn velocity(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe { ffi::PhidgetDCMotor_getVelocity(self.chan, &mut value) })?;
        Ok(value)
    }

    /// Get minimum velocity.
    pub fn min_velocity(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_getMinVelocity(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Get maximum velocity.
    pub fn max_velocity(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_getMaxVelocity(self.chan, &mut value)
        })?;
        Ok(value)
    }

    // ----- Callbacks -----

    // Low-level, unsafe, callback for back EMF change events.
    unsafe extern "C" fn on_back_emf_change(
        chan: DcMotorHandle,
        ctx: *mut c_void,
        back_emf: f64,
    ) {
        if !ctx.is_null() {
            let cb: &mut Box<BackEmfChangeCallback> = &mut *(ctx as *mut _);
            let dev = Self::from(chan);
            cb(&dev, back_emf);
            mem::forget(dev);
        }
    }

    /// Set a handler to receive back EMF change callbacks.
    pub fn set_on_back_emf_change_handler<F>(&mut self, cb: F) -> Result<()>
    where
        F: Fn(&DcMotor, f64) + Send + 'static,
    {
        let cb: Box<Box<BackEmfChangeCallback>> = Box::new(Box::new(cb));
        let ctx = Box::into_raw(cb) as *mut c_void;
        self.cb = Some(ctx);

        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_setOnBackEMFChangeHandler(
                self.chan,
                Some(Self::on_back_emf_change),
                ctx,
            )
        })
    }

    // Low-level, unsafe, callback for braking strength change events.
    unsafe extern "C" fn on_braking_strength_change(
        chan: DcMotorHandle,
        ctx: *mut c_void,
        braking_strength: f64,
    ) {
        if !ctx.is_null() {
            let cb: &mut Box<BrakingStrengthChangeCallback> = &mut *(ctx as *mut _);
            let dev = Self::from(chan);
            cb(&dev, braking_strength);
            mem::forget(dev);
        }
    }

    /// Set a handler to receive braking strength change callbacks.
    pub fn set_on_braking_strength_change_handler<F>(&mut self, cb: F) -> Result<()>
    where
        F: Fn(&DcMotor, f64) + Send + 'static,
    {
        let cb: Box<Box<BrakingStrengthChangeCallback>> = Box::new(Box::new(cb));
        let ctx = Box::into_raw(cb) as *mut c_void;
        self.cb = Some(ctx);

        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_setOnBrakingStrengthChangeHandler(
                self.chan,
                Some(Self::on_braking_strength_change),
                ctx,
            )
        })
    }

    // Low-level, unsafe, callback for velocity update events.
    unsafe extern "C" fn on_velocity_update(
        chan: DcMotorHandle,
        ctx: *mut c_void,
        velocity: f64,
    ) {
        if !ctx.is_null() {
            let cb: &mut Box<VelocityUpdateCallback> = &mut *(ctx as *mut _);
            let dev = Self::from(chan);
            cb(&dev, velocity);
            mem::forget(dev);
        }
    }

    /// Set a handler to receive velocity update callbacks.
    pub fn set_on_velocity_update_handler<F>(&mut self, cb: F) -> Result<()>
    where
        F: Fn(&DcMotor, f64) + Send + 'static,
    {
        let cb: Box<Box<VelocityUpdateCallback>> = Box::new(Box::new(cb));
        let ctx = Box::into_raw(cb) as *mut c_void;
        self.cb = Some(ctx);

        ReturnCode::result(unsafe {
            ffi::PhidgetDCMotor_setOnVelocityUpdateHandler(
                self.chan,
                Some(Self::on_velocity_update),
                ctx,
            )
        })
    }

    // ----- Attach / Detach -----

    /// Sets a handler to receive attach callbacks.
    pub fn set_on_attach_handler<F>(&mut self, cb: F) -> Result<()>
    where
        F: Fn(&mut DcMotor) + Send + 'static,
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
        F: Fn(&mut DcMotor) + Send + 'static,
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

impl Phidget for DcMotor {
    fn as_mut_handle(&mut self) -> PhidgetHandle {
        self.chan as PhidgetHandle
    }

    fn as_handle(&self) -> PhidgetHandle {
        self.chan as PhidgetHandle
    }
}

unsafe impl Send for DcMotor {}

impl Default for DcMotor {
    fn default() -> Self {
        Self::new()
    }
}

impl From<DcMotorHandle> for DcMotor {
    fn from(chan: DcMotorHandle) -> Self {
        Self {
            chan,
            cb: None,
            attach_cb: None,
            detach_cb: None,
        }
    }
}

impl Drop for DcMotor {
    fn drop(&mut self) {
        if let Ok(true) = self.is_open() {
            let _ = self.close();
        }
        unsafe {
            ffi::PhidgetDCMotor_delete(&mut self.chan);
            crate::drop_cb::<BackEmfChangeCallback>(self.cb.take());
            crate::drop_cb::<BrakingStrengthChangeCallback>(self.cb.take());
            crate::drop_cb::<VelocityUpdateCallback>(self.cb.take());
            crate::drop_cb::<AttachCallback>(self.attach_cb.take());
            crate::drop_cb::<DetachCallback>(self.detach_cb.take());
        }
    }
}
