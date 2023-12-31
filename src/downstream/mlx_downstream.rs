use core::convert::Infallible;

use cortex_m::delay;
use defmt::{debug, info, Format};
use embedded_hal::digital::v2::OutputPin;
use rp2040_hal::{
    spi::{Enabled, SpiDevice, ValidSpiPinout},
    Spi,
};
use ux::u7;

use crate::negicon_event::{NegiconEvent, NegiconEventType};

use super::{
    mlx90363::{Mlx90363, MlxReply},
    spi_downstream::{DownstreamDevice, DownstreamError},
};

#[derive(PartialEq, Clone, Copy, Format)]
enum InputMode {
    Absolute,
    Relative,
}

#[derive(PartialEq, Copy, Clone, Format)]
enum ParameterState<T: Copy + Format> {
    Uninitialized(T),
    Requested(T),
    Initialized(T),
}

impl<T: Copy + Format> ParameterState<T> {
    fn get_value(&self) -> T {
        match self {
            ParameterState::Uninitialized(value) => *value,
            ParameterState::Requested(value) => *value,
            ParameterState::Initialized(value) => *value,
        }
    }
}

#[derive(PartialEq, Copy, Clone, Format)]
enum ButtonState {
    Up,
    Down,
}

#[derive(Format)]
pub(crate) struct MlxDownstream {
    id: ParameterState<u16>,
    min: ParameterState<u16>,
    max: ParameterState<u16>,
    mode: InputMode,
    last: u16,
    button_state: ButtonState,
    lock_countdown: i16,
}

const ADDR_ID: u16 = 0x1018;
const ADDR_MIN: u16 = 0x103A;
const ADDR_MAX: u16 = 0x103C;

impl MlxDownstream {
    pub(crate) fn new() -> Self {
        Self {
            id: ParameterState::Uninitialized(0),
            min: ParameterState::Uninitialized(0),
            max: ParameterState::Uninitialized(0),
            mode: InputMode::Relative,
            last: 0,
            button_state: ButtonState::Up,
            lock_countdown: 100,
        }
    }
    fn init_param<R: Copy + Format, D: SpiDevice, T: ValidSpiPinout<D>>(
        spi: &mut Spi<Enabled, D, T, 8>,
        cs: &mut dyn OutputPin<Error = Infallible>,
        param: ParameterState<R>,
        addresses: [u16; 2],
        transform: fn([u16; 2]) -> R,
    ) -> Result<ParameterState<R>, DownstreamError> {
        debug!("Querying param {:x}", addresses[0]);
        match param {
            ParameterState::Uninitialized(default) => {
                match Mlx90363::read_memory(spi, cs, addresses[0], addresses[1]) {
                    Ok(_) => Ok(ParameterState::Requested(default)),
                    Err(e) => return Err(DownstreamError::MlxError(e)),
                }
            }
            ParameterState::Requested(_) => {
                match Mlx90363::read_memory(spi, cs, addresses[0], addresses[1]) {
                    Ok(res) => match res {
                        MlxReply::MlxMemReadResponse(msg) => {
                            Ok(ParameterState::Initialized(transform([
                                msg.data0, msg.data1,
                            ])))
                        }
                        _ => {
                            debug!("MLX init got {}", res);
                            return Err(DownstreamError::UnexpectedReply);
                        }
                    },
                    Err(e) => return Err(DownstreamError::MlxError(e)),
                }
            }
            ParameterState::Initialized(_) => Ok(param),
        }
    }
    fn check_deadzone(&mut self, input: u16) -> bool {
        let deadzone = 64;
        let diff = input as i32 - self.last as i32;
        if diff.abs() > deadzone {
            true
        } else {
            false
        }
    }
    fn calculate_output(&mut self, input: u16) -> i16 {
        match self.mode {
            InputMode::Absolute => {
                self.last = input;
                let mut output = input as i32;
                output -= self.min.get_value() as i32;
                output *= 16383;
                output /= (self.max.get_value() - self.min.get_value()) as i32;
                output as i16
            }
            InputMode::Relative => {
                let last = self.last as i32;
                let input = input as i32;
                self.last = input as u16;

                let mut diff = input - last;

                if diff > 16384 / 2 {
                    diff -= 16384;
                } else if diff < -16384 / 2 {
                    diff += 16384;
                }
                diff as i16
            }
        }
    }

    fn check_button(&mut self, vg: u8) -> Option<NegiconEvent> {
        if self.button_state == ButtonState::Up && vg < 35 {
            self.lock_countdown = -1;
            self.button_state = ButtonState::Down;
            Some(NegiconEvent::new(
                NegiconEventType::Input,
                self.id.get_value(),
                u7::new(1),
                1,
                0,
                0,
            ))
        } else if self.button_state == ButtonState::Down && vg > 35 {
            self.button_state = ButtonState::Up;
            self.lock_countdown = 100;
            Some(NegiconEvent::new(
                NegiconEventType::Input,
                self.id.get_value(),
                u7::new(1),
                -1,
                0,
                0,
            ))
        } else {
            None
        }
    }
}
impl<D, T> DownstreamDevice<D, T> for MlxDownstream
where
    D: SpiDevice,
    T: ValidSpiPinout<D>,
{
    fn poll(
        &mut self,
        spi: &mut Spi<Enabled, D, T, 8>,
        cs: &mut dyn OutputPin<Error = Infallible>,
    ) -> Result<Option<NegiconEvent>, DownstreamError> {
        match self.id {
            ParameterState::Initialized(_) => {}
            _ => {
                self.id =
                    MlxDownstream::init_param(spi, cs, self.id, [ADDR_ID, ADDR_ID], |x| -> u16 {
                        x[1]
                    })?;
                return Ok(None);
            }
        }
        match self.min {
            ParameterState::Initialized(_) => {}
            _ => {
                self.min = MlxDownstream::init_param(
                    spi,
                    cs,
                    self.min,
                    [ADDR_MIN, ADDR_MIN],
                    |x| -> u16 { x[1] },
                )?;
                return Ok(None);
            }
        }
        match self.max {
            ParameterState::Initialized(_) => {}
            _ => {
                self.max = MlxDownstream::init_param(
                    spi,
                    cs,
                    self.max,
                    [ADDR_MAX, ADDR_MAX],
                    |x| -> u16 { x[1] },
                )?;
                if let ParameterState::Initialized(_) = self.max {
                    info!("Initialized MLX Downstream {}", self)
                }
                return Ok(None);
            }
        }
        if self.min.get_value() != 0 || self.max.get_value() != 0 {
            self.mode = InputMode::Absolute;
        } else {
            self.mode = InputMode::Relative;
        }
        match Mlx90363::get_alpha(spi, cs) {
            Ok(res) => match res {
                MlxReply::MlxAlpha(a) => {
                    match self.check_button(a.vg) {
                        Some(event) => return Ok(Some(event)),
                        None => {}
                    }

                    match self.lock_countdown {
                        -1 => {
                            self.last = a.data;
                            return Ok(None);
                        }
                        0 => {}
                        _ => {
                            self.last = a.data;
                            self.lock_countdown -= 1;
                            return Ok(None);
                        }
                    }
                    if self.check_deadzone(a.data) {
                        Ok(Some(NegiconEvent::new(
                            NegiconEventType::Input,
                            self.id.get_value() as u16,
                            u7::new(0),
                            self.calculate_output(a.data),
                            0,
                            0,
                        )))
                    } else {
                        return Ok(None);
                    }
                }
                _ => Ok(None),
            },
            Err(e) => Err(DownstreamError::MlxError(e)),
        }
    }

    fn write_memory(
        &mut self,
        spi: &mut Spi<Enabled, D, T, 8>,
        cs: &mut dyn OutputPin<Error = Infallible>,
        delay: &mut delay::Delay,
        write_event: &NegiconEvent,
    ) {
        Mlx90363::write_memory(spi, cs, delay, write_event.value, write_event.sequence);
    }
}

//impl<R: MlxReply> MlxDownstream<R> {}
