use core::convert::Infallible;

use cortex_m::delay;
use defmt::{debug, info};
use embedded_hal::digital::v2::OutputPin;
use rp2040_hal::{
    spi::{Enabled, SpiDevice, ValidSpiPinout},
    Spi,
};

use crate::negicon_event::{NegiconEvent, NegiconEventType};

use super::{
    mlx90363::{Mlx90363, MlxReply},
    spi_downstream::{DownstreamDevice, DownstreamError},
};

#[derive(PartialEq, Clone, Copy)]
enum InputMode {
    Absolute,
    Relative,
}

#[derive(PartialEq, Copy, Clone)]
enum ParameterState<T: Copy> {
    Uninitialized(T),
    Requested(T),
    Initialized(T),
}

impl<T: Copy> ParameterState<T> {
    fn get_value(&self) -> T {
        match self {
            ParameterState::Uninitialized(value) => *value,
            ParameterState::Requested(value) => *value,
            ParameterState::Initialized(value) => *value,
        }
    }
}

pub(crate) struct MlxDownstream {
    id: ParameterState<u16>,
    min: ParameterState<u16>,
    max: ParameterState<u16>,
    mode: ParameterState<InputMode>,
    last: u16,
}

impl MlxDownstream {
    pub(crate) fn new() -> Self {
        Self {
            id: ParameterState::Uninitialized(0),
            min: ParameterState::Initialized(0),
            max: ParameterState::Initialized(16383),
            mode: ParameterState::Initialized(InputMode::Absolute),
            last: 0,
        }
    }
    fn init_param<R: Copy, D: SpiDevice, T: ValidSpiPinout<D>>(
        spi: &mut Spi<Enabled, D, T, 8>,
        cs: &mut dyn OutputPin<Error = Infallible>,
        param: ParameterState<R>,
        addresses: [u16; 2],
        transform: fn([u16; 2]) -> R,
    ) -> Result<ParameterState<R>, DownstreamError> {
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
        let deadzone = 100;
        let diff = input as i32 - self.last as i32;
        if diff.abs() > deadzone {
            self.last = input;
            true
        } else {
            false
        }
    }
    fn calculate_output(&self, input: i16) -> i16 {
        match self.mode {
            ParameterState::Initialized(InputMode::Absolute) => {
                let mut output = input as i32;
                output -= self.min.get_value() as i32;
                output *= 16383;
                output /= (self.max.get_value() - self.min.get_value()) as i32;
                output as i16
            }
            ParameterState::Initialized(InputMode::Relative) => {
                todo!()
            }
            _ => panic!("Mlx not initialized"),
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
                    MlxDownstream::init_param(spi, cs, self.id, [0x1018, 0x1018], |x| -> u16 {
                        info!("Initialized MLX with ID {:x}", x[1]);
                        x[1]
                    })?;
            }
        }
        match Mlx90363::get_alpha(spi, cs) {
            Ok(res) => match res {
                MlxReply::MlxAlpha(a) => {
                    if self.check_deadzone(a.data) {
                        Ok(Some(NegiconEvent::new(
                            NegiconEventType::Input,
                            self.id.get_value() as u16,
                            self.calculate_output(a.data as i16),
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
        /*match self.min {
                    ParameterState::Initialized(_) => {}
                    _ => {
                        self.min = MlxDownstream::init_param(spi, cs, self.min, [], make_u16)?;
                    }
                }
                match self.max {
                    ParameterState::Uninitialized => todo!(),
                    ParameterState::Requested => todo!(),
                    ParameterState::Initialized(_) => todo!(),
                }
                match self.mode {
                    ParameterState::Uninitialized => todo!(),
                    ParameterState::Requested => todo!(),
                    ParameterState::Initialized(_) => todo!(),
                }
        */

        //self.mlx.nop(spi, cs, 0x3939)
        /*match self.mode {
            Uninitialized => {
                //Mlx90363::read_memory(spi, cs, MLXID_ADDR_HI, MLXID_ADDR_MID);
            }
        }*/
        /*match self.sensor.get_alpha(spi, cs) {
            Ok(res) => Ok(Some(NegiconEvent::new(0, res.data as i16, 0, res.counter))),
            Err(e) => Err(DownstreamError::MlxError(e)),
        }*/
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
