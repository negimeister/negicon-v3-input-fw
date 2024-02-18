extern crate alloc;
use core::{convert::Infallible, marker::PhantomData};

use cortex_m::delay::Delay;
use defmt::{error, Format};
use embedded_hal::{blocking::spi::Transfer, digital::v2::OutputPin};
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
    spi: PhantomData<D>,
    pinout: PhantomData<T>,
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
            spi: PhantomData,
            pinout: PhantomData,
        }
    }

    pub fn poll(
        &mut self,
        _delay: &mut Delay,
        spi: &mut Spi<Enabled, D, T, 8>,
    ) -> Result<NegiconEvent, DownstreamError> {
        let packet = &mut [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
        self.cs.set_low().unwrap();
        spi.transfer(packet)
            .map_err(|_| DownstreamError::UnknownDevice(0))?;
        self.cs.set_high().unwrap();
        NegiconEvent::deserialize(&packet).map_err(|_| DownstreamError::UnexpectedReply)
    }
}
