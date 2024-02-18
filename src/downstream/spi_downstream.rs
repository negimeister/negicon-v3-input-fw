extern crate alloc;
use core::convert::Infallible;

use alloc::boxed::Box;
use cortex_m::delay::Delay;
use defmt::{error, info, Format};
use embedded_hal::digital::v2::OutputPin;
use rp2040_hal::{
    spi::{Enabled, SpiDevice as HalSpiDevice, ValidSpiPinout},
    Spi,
};

use negicon_protocol::negicon_event::NegiconEvent;
#[derive(Format)]
pub(crate) enum DownstreamError {
    UnknownDevice(u8),
    UnexpectedReply,
}

pub(crate) struct SpiDownstream<'a, D, T>
where
    D: HalSpiDevice,
    T: ValidSpiPinout<D>,
{
    cs: &'a mut dyn OutputPin<Error = Infallible>,
    pub(crate) device: Option<Box<dyn DownstreamDevice<D, T>>>,
}

pub(crate) trait DownstreamDevice<D, T>
where
    D: HalSpiDevice,
    T: ValidSpiPinout<D>,
{
    fn poll(
        &mut self,
        spi: &mut Spi<Enabled, D, T, 8>,
        cs: &mut dyn OutputPin<Error = Infallible>,
    ) -> Result<Option<NegiconEvent>, DownstreamError>;

    fn write_memory(
        &mut self,
        _spi: &mut Spi<Enabled, D, T, 8>,
        _cs: &mut dyn OutputPin<Error = Infallible>,
        _delay: &mut Delay,
        _write_event: &NegiconEvent,
    ) {
        error!("Memory write target not implemented");
    }
}

impl<'a, D, T> SpiDownstream<'a, D, T>
where
    D: HalSpiDevice,
    T: ValidSpiPinout<D>,
{
    pub(crate) fn new(cs: &'a mut dyn OutputPin<Error = Infallible>) -> Self {
        Self { cs, device: None }
    }

    pub fn poll(
        &mut self,
        _delay: &mut Delay,
        _spi: &mut Spi<Enabled, D, T, 8>,
    ) -> Result<Option<NegiconEvent>, DownstreamError> {
        Ok(None)
    }
}
