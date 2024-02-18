use defmt::debug;

use negicon_protocol::spi_protocol::verified_transmit;
use rp2040_hal::{
    spi::{Enabled, SpiDevice, ValidSpiPinout},
    Spi,
};

pub(crate) struct SPIUpstream<D, P>
where
    D: SpiDevice,
    P: ValidSpiPinout<D>,
{
    spi: Spi<Enabled, D, P, 8>,
}

impl<D, P> SPIUpstream<D, P>
where
    D: SpiDevice,
    P: ValidSpiPinout<D>,
{
    pub(crate) fn new(spi: Spi<Enabled, D, P, 8>) -> Self {
        Self { spi }
    }

    pub(crate) fn transmit_event(&mut self, event: &mut [u8; 8]) -> Result<(), &'static str> {
        debug!("SPI TX: {:?}", event);
        match verified_transmit(&mut self.spi, event) {
            Ok(_) => {
                debug!("SPI OK. rx: {:?}", event);
                Ok(())
            }
            Err(_) => {
                debug!("SPI Error");
                Err("SPI Upstream Error")
            }
        }
    }
}
