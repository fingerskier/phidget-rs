// phidget-rs/src/devices/encoder.rs
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
use phidget_sys::{self as ffi, PhidgetEncoderHandle as EncoderHandle, PhidgetHandle};
use std::{
    ffi::{c_int, c_uint, c_void},
    mem, ptr,
};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Encoder I/O mode
#[derive(Copy, Clone, Debug, Eq, Ord, PartialEq, PartialOrd, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[repr(u32)]
pub enum EncoderIoMode {
    /// Push-pull mode.
    PushPull = ffi::Phidget_EncoderIOMode_ENCODER_IO_MODE_PUSH_PULL,
    /// Line driver 2.2K mode.
    LineDriver2K2 = ffi::Phidget_EncoderIOMode_ENCODER_IO_MODE_LINE_DRIVER_2K2,
    /// Line driver 10K mode.
    LineDriver10K = ffi::Phidget_EncoderIOMode_ENCODER_IO_MODE_LINE_DRIVER_10K,
    /// Open collector 2.2K mode.
    OpenCollector2K2 = ffi::Phidget_EncoderIOMode_ENCODER_IO_MODE_OPEN_COLLECTOR_2K2,
    /// Open collector 10K mode.
    OpenCollector10K = ffi::Phidget_EncoderIOMode_ENCODER_IO_MODE_OPEN_COLLECTOR_10K,
}

impl TryFrom<u32> for EncoderIoMode {
    type Error = Error;

    fn try_from(val: u32) -> Result<Self> {
        use EncoderIoMode::*;
        match val {
            ffi::Phidget_EncoderIOMode_ENCODER_IO_MODE_PUSH_PULL => Ok(PushPull),
            ffi::Phidget_EncoderIOMode_ENCODER_IO_MODE_LINE_DRIVER_2K2 => Ok(LineDriver2K2),
            ffi::Phidget_EncoderIOMode_ENCODER_IO_MODE_LINE_DRIVER_10K => Ok(LineDriver10K),
            ffi::Phidget_EncoderIOMode_ENCODER_IO_MODE_OPEN_COLLECTOR_2K2 => Ok(OpenCollector2K2),
            ffi::Phidget_EncoderIOMode_ENCODER_IO_MODE_OPEN_COLLECTOR_10K => Ok(OpenCollector10K),
            _ => Err(ReturnCode::UnknownVal),
        }
    }
}

/////////////////////////////////////////////////////////////////////////////

/// The function type for the safe Rust attach callback.
pub type AttachCallback = dyn Fn(&mut Encoder) + Send + 'static;

/// The function type for the safe Rust detach callback.
pub type DetachCallback = dyn Fn(&mut Encoder) + Send + 'static;

/// The function type for the safe Rust position change callback.
/// Parameters: position change (i32), time change (f64), index triggered (bool)
pub type PositionChangeCallback = dyn Fn(&Encoder, i32, f64, bool) + Send + 'static;

/// Phidget Encoder channel
pub struct Encoder {
    // Handle to the channel for the phidget22 library
    chan: EncoderHandle,
    // Double-boxed callback, if registered
    cb: Option<*mut c_void>,
    // Double-boxed attach callback, if registered
    attach_cb: Option<*mut c_void>,
    // Double-boxed detach callback, if registered
    detach_cb: Option<*mut c_void>,
}

impl Encoder {
    /// Create a new Encoder channel.
    pub fn new() -> Self {
        let mut chan: EncoderHandle = ptr::null_mut();
        unsafe {
            ffi::PhidgetEncoder_create(&mut chan);
        }
        Self::from(chan)
    }

    // Low-level, unsafe callback for device attach events
    unsafe extern "C" fn on_attach(phid: PhidgetHandle, ctx: *mut c_void) {
        if !ctx.is_null() {
            let cb: &mut Box<AttachCallback> = &mut *(ctx as *mut _);
            let mut dev = Self::from(phid as EncoderHandle);
            cb(&mut dev);
            mem::forget(dev);
        }
    }

    // Low-level, unsafe callback for device detach events
    unsafe extern "C" fn on_detach(phid: PhidgetHandle, ctx: *mut c_void) {
        if !ctx.is_null() {
            let cb: &mut Box<DetachCallback> = &mut *(ctx as *mut _);
            let mut dev = Self::from(phid as EncoderHandle);
            cb(&mut dev);
            mem::forget(dev);
        }
    }

    /// Get a reference to the underlying channel handle
    pub fn as_channel(&self) -> &EncoderHandle {
        &self.chan
    }

    // ----- Enable -----

    /// Set enabled state.
    pub fn set_enabled(&self, enabled: bool) -> Result<()> {
        let value = enabled as c_int;
        ReturnCode::result(unsafe { ffi::PhidgetEncoder_setEnabled(self.chan, value) })?;
        Ok(())
    }

    /// Get enabled state.
    pub fn enabled(&self) -> Result<bool> {
        let mut value: c_int = 0;
        ReturnCode::result(unsafe { ffi::PhidgetEncoder_getEnabled(self.chan, &mut value) })?;
        Ok(value != 0)
    }

    // ----- Data interval / rate -----

    /// Set data interval in milliseconds.
    pub fn set_data_interval(&self, data_interval: u32) -> Result<()> {
        ReturnCode::result(unsafe {
            ffi::PhidgetEncoder_setDataInterval(self.chan, data_interval)
        })?;
        Ok(())
    }

    /// Get data interval in milliseconds.
    pub fn data_interval(&self) -> Result<u32> {
        let mut value = 0;
        ReturnCode::result(unsafe {
            ffi::PhidgetEncoder_getDataInterval(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Get minimum data interval.
    pub fn min_data_interval(&self) -> Result<u32> {
        let mut value = 0;
        ReturnCode::result(unsafe {
            ffi::PhidgetEncoder_getMinDataInterval(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Get maximum data interval.
    pub fn max_data_interval(&self) -> Result<u32> {
        let mut value = 0;
        ReturnCode::result(unsafe {
            ffi::PhidgetEncoder_getMaxDataInterval(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Set data rate in Hz.
    pub fn set_data_rate(&self, data_rate: f64) -> Result<()> {
        ReturnCode::result(unsafe { ffi::PhidgetEncoder_setDataRate(self.chan, data_rate) })?;
        Ok(())
    }

    /// Get data rate in Hz.
    pub fn data_rate(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe { ffi::PhidgetEncoder_getDataRate(self.chan, &mut value) })?;
        Ok(value)
    }

    /// Get minimum data rate.
    pub fn min_data_rate(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetEncoder_getMinDataRate(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Get maximum data rate.
    pub fn max_data_rate(&self) -> Result<f64> {
        let mut value = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetEncoder_getMaxDataRate(self.chan, &mut value)
        })?;
        Ok(value)
    }

    // ----- Index position -----

    /// Get the index position of the encoder.
    pub fn index_position(&self) -> Result<i64> {
        let mut value: i64 = 0;
        ReturnCode::result(unsafe {
            ffi::PhidgetEncoder_getIndexPosition(self.chan, &mut value)
        })?;
        Ok(value)
    }

    // ----- IO mode -----

    /// Set encoder I/O mode.
    pub fn set_io_mode(&self, io_mode: EncoderIoMode) -> Result<()> {
        ReturnCode::result(unsafe {
            ffi::PhidgetEncoder_setIOMode(self.chan, io_mode as c_uint)
        })?;
        Ok(())
    }

    /// Get encoder I/O mode.
    pub fn io_mode(&self) -> Result<EncoderIoMode> {
        let mut mode: ffi::Phidget_EncoderIOMode = 0;
        ReturnCode::result(unsafe { ffi::PhidgetEncoder_getIOMode(self.chan, &mut mode) })?;
        EncoderIoMode::try_from(mode)
    }

    // ----- Position -----

    /// Set position.
    pub fn set_position(&self, position: i64) -> Result<()> {
        ReturnCode::result(unsafe { ffi::PhidgetEncoder_setPosition(self.chan, position) })?;
        Ok(())
    }

    /// Get position.
    pub fn position(&self) -> Result<i64> {
        let mut value: i64 = 0;
        ReturnCode::result(unsafe { ffi::PhidgetEncoder_getPosition(self.chan, &mut value) })?;
        Ok(value)
    }

    // ----- Position change trigger -----

    /// Set position change trigger.
    pub fn set_position_change_trigger(&self, trigger: u32) -> Result<()> {
        ReturnCode::result(unsafe {
            ffi::PhidgetEncoder_setPositionChangeTrigger(self.chan, trigger)
        })?;
        Ok(())
    }

    /// Get position change trigger.
    pub fn position_change_trigger(&self) -> Result<u32> {
        let mut value = 0;
        ReturnCode::result(unsafe {
            ffi::PhidgetEncoder_getPositionChangeTrigger(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Get minimum position change trigger.
    pub fn min_position_change_trigger(&self) -> Result<u32> {
        let mut value = 0;
        ReturnCode::result(unsafe {
            ffi::PhidgetEncoder_getMinPositionChangeTrigger(self.chan, &mut value)
        })?;
        Ok(value)
    }

    /// Get maximum position change trigger.
    pub fn max_position_change_trigger(&self) -> Result<u32> {
        let mut value = 0;
        ReturnCode::result(unsafe {
            ffi::PhidgetEncoder_getMaxPositionChangeTrigger(self.chan, &mut value)
        })?;
        Ok(value)
    }

    // ----- Callbacks -----

    // Low-level, unsafe, callback for position change events.
    unsafe extern "C" fn on_position_change(
        chan: EncoderHandle,
        ctx: *mut c_void,
        position_change: c_int,
        time_change: f64,
        index_triggered: c_int,
    ) {
        if !ctx.is_null() {
            let cb: &mut Box<PositionChangeCallback> = &mut *(ctx as *mut _);
            let dev = Self::from(chan);
            cb(&dev, position_change, time_change, index_triggered != 0);
            mem::forget(dev);
        }
    }

    /// Set a handler to receive position change callbacks.
    pub fn set_on_position_change_handler<F>(&mut self, cb: F) -> Result<()>
    where
        F: Fn(&Encoder, i32, f64, bool) + Send + 'static,
    {
        let cb: Box<Box<PositionChangeCallback>> = Box::new(Box::new(cb));
        let ctx = Box::into_raw(cb) as *mut c_void;
        self.cb = Some(ctx);

        ReturnCode::result(unsafe {
            ffi::PhidgetEncoder_setOnPositionChangeHandler(
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
        F: Fn(&mut Encoder) + Send + 'static,
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
        F: Fn(&mut Encoder) + Send + 'static,
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

impl Phidget for Encoder {
    fn as_mut_handle(&mut self) -> PhidgetHandle {
        self.chan as PhidgetHandle
    }

    fn as_handle(&self) -> PhidgetHandle {
        self.chan as PhidgetHandle
    }
}

unsafe impl Send for Encoder {}

impl Default for Encoder {
    fn default() -> Self {
        Self::new()
    }
}

impl From<EncoderHandle> for Encoder {
    fn from(chan: EncoderHandle) -> Self {
        Self {
            chan,
            cb: None,
            attach_cb: None,
            detach_cb: None,
        }
    }
}

impl Drop for Encoder {
    fn drop(&mut self) {
        if let Ok(true) = self.is_open() {
            let _ = self.close();
        }
        unsafe {
            ffi::PhidgetEncoder_delete(&mut self.chan);
            crate::drop_cb::<PositionChangeCallback>(self.cb.take());
            crate::drop_cb::<AttachCallback>(self.attach_cb.take());
            crate::drop_cb::<DetachCallback>(self.detach_cb.take());
        }
    }
}
