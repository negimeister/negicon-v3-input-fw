use defmt::debug;

use embedded_hal::{
    blocking::spi::{Transfer, Write},
    spi::FullDuplex,
};
use rp2040_hal::{
    dma::ReadTarget,
    spi::{Enabled, SpiDevice, ValidSpiPinout},
    Spi,
};

pub(crate) struct SPIUpstream<D, P>
where
    D: SpiDevice,
    P: ValidSpiPinout<D>,
{
    spi: Spi<Enabled, D, P, 8>,
    rx_buffer: [u8; 8],
    rx_index: usize,
    tx_buffer: [u8; 8],
    tx_idx: usize,
}

impl<D, P> SPIUpstream<D, P>
where
    D: SpiDevice,
    P: ValidSpiPinout<D>,
{
    pub(crate) fn new(spi: Spi<Enabled, D, P, 8>) -> Self {
        Self {
            spi,
            rx_buffer: [0u8; 8],
            rx_index: 0,
            tx_buffer: [0u8; 8],
            tx_idx: 0,
        }
    }

    pub(crate) fn read(&mut self) -> Option<[u8; 8]> {
        while self.rx_index < self.rx_buffer.len() {
            match self.spi.read() {
                Ok(word) => {
                    self.rx_buffer[self.rx_index] = word;
                    self.rx_index += 1;
                }
                Err(nb::Error::WouldBlock) => {
                    return None;
                }
                Err(_) => {
                    return None;
                }
            }
        }
        let res = self.rx_buffer.clone();
        self.rx_index = 0;
        Some(res)
    }

    pub(crate) fn send(&mut self, event: &mut [u8; 8]) -> Result<(), nb::Error<()>> {
        if self.spi.is_busy() {
            return Err(nb::Error::WouldBlock);
        }
        match self.spi.send(event[0]) {
            Ok(_) => self.spi.write(&event[1..]),
            Err(_) => return Err(nb::Error::WouldBlock),
        };
        Ok(())
    }
}
