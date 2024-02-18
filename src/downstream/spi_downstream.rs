extern crate alloc;
use core::convert::Infallible;

use alloc::boxed::Box;
use cortex_m::delay::Delay;
use defmt::{error, info, warn, Format};
use embedded_hal::digital::v2::OutputPin;
use rp2040_hal::{
    spi::{Enabled, SpiDevice as HalSpiDevice, ValidSpiPinout},
    Spi,
};

use negicon_protocol::{negicon_event::NegiconEvent, spi_protocol::verified_transmit_cs};
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
    pub(crate) device: DownstreamState<D, T>,
}

pub(crate) enum DownstreamState<D, T>
where
    D: HalSpiDevice,
    T: ValidSpiPinout<D>,
{
    Uninitialized,
    Initialized(Box<dyn DownstreamDevice<D, T>>),
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
        Self {
            cs,
            device: DownstreamState::Uninitialized,
        }
    }

    pub fn poll(
        &mut self,
        delay: &mut Delay,
        spi: &mut Spi<Enabled, D, T, 8>,
    ) -> Result<Option<NegiconEvent>, DownstreamError> {
        Ok(None)
    }

    pub(crate) fn write_memory(
        &mut self,
        write_event: &NegiconEvent,
        spi: &mut Spi<Enabled, D, T, 8>,
        delay: &mut Delay,
    ) {
        info!(
            "Downstream memory write request. Id: {}, Address: {:x}, Value: {:x}",
            write_event.id, write_event.sequence, write_event.value
        );
        match &mut self.device {
            DownstreamState::Uninitialized => {
                error!("Memory write target not inialized")
            }
            DownstreamState::Initialized(dev) => {
                dev.as_mut().write_memory(spi, self.cs, delay, write_event);
            }
        }
    }

    fn detect(
        &mut self,
        _delay: &mut Delay,
        spi: &mut Spi<Enabled, D, T, 8>,
    ) -> Result<Option<NegiconEvent>, DownstreamError>
    where
        D: HalSpiDevice,
        T: ValidSpiPinout<D>,
    {
        Ok(None)
    }
}
