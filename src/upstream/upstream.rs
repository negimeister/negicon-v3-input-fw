use super::spi::SPIUpstream;
use negicon_protocol::{negicon_event::NegiconEvent, ringbuf::RingBuffer, InvalidMessage};

use defmt::Format;
use frunk::{HCons, HNil};

use rp2040_hal::spi::{SpiDevice, ValidSpiPinout};
use usb_device::{class_prelude::UsbBus, device::UsbDevice, UsbError};
use usbd_human_interface_device::{
    interface::{InBytes8, Interface, OutBytes8, ReportSingle},
    usb_class::UsbHidClass,
};
type HID<'a, B> =
    UsbHidClass<'a, B, HCons<Interface<'a, B, InBytes8, OutBytes8, ReportSingle>, HNil>>;
pub(crate) struct Upstream<'a> {
    tx_buffer: RingBuffer<[u8; 8], 64>,
    rx_buffer: RingBuffer<[u8; 8], 64>,
    interface: &'a mut dyn UpstreamInterface<64>,
}

impl<'a> Upstream<'a> {
    pub(crate) fn new(interface: &'a mut dyn UpstreamInterface<64>) -> Self {
        Self {
            tx_buffer: RingBuffer::new(),
            rx_buffer: RingBuffer::new(),
            interface,
        }
    }

    pub(crate) fn poll(&mut self) -> Result<(), UpstreamError> {
        self.interface
            .poll(&mut self.tx_buffer, &mut self.rx_buffer)
    }

    pub(crate) fn send(&mut self, event: &NegiconEvent) -> Result<(), UpstreamError> {
        self.tx_buffer
            .push(event.serialize())
            .map_err(|_| UpstreamError::BufferOverflow)
    }

    pub(crate) fn receive(&mut self) -> Result<Option<NegiconEvent>, UpstreamError> {
        let deserialized = match self.tx_buffer.pop() {
            Some(event) => NegiconEvent::deserialize(&event),
            None => return Ok(None),
        };
        match deserialized {
            Ok(event) => Ok(Some(event)),
            Err(e) => Err(UpstreamError::InvalidMessage(e)),
        }
    }
}

pub(crate) struct UsbUpstream<'a, B: UsbBus + 'a> {
    hid: HID<'a, B>,
    dev: UsbDevice<'a, B>,
}

impl<'a, B> UsbUpstream<'a, B>
where
    B: UsbBus,
{
    pub(crate) fn new(hid: HID<'a, B>, dev: UsbDevice<'a, B>) -> Self {
        Self { hid, dev }
    }
}

impl<B, const SIZE: usize> UpstreamInterface<SIZE> for UsbUpstream<'_, B>
where
    B: UsbBus,
{
    fn poll(
        &mut self,
        tx_buffer: &mut RingBuffer<[u8; 8], SIZE>,
        rx_buffer: &mut RingBuffer<[u8; 8], SIZE>,
    ) -> Result<(), UpstreamError> {
        self.dev.poll(&mut [&mut self.hid]);
        let mut rx = [0u8; 8];
        loop {
            match self.hid.device().read_report(&mut rx) {
                Ok(_) => {
                    let _ = rx_buffer.push(rx);
                }
                Err(UsbError::WouldBlock) => {
                    break;
                }
                Err(e) => return Err(UpstreamError::UsbError(e)),
            }
        }
        loop {
            match tx_buffer.peek() {
                Some(event) => match self.hid.device().write_report(event) {
                    Ok(_) => {
                        let _ = tx_buffer.discard();
                    }
                    Err(UsbError::WouldBlock) => {
                        break;
                    }
                    Err(e) => return Err(UpstreamError::UsbError(e)),
                },
                None => {
                    break;
                }
            }
        }
        Ok(())
    }
}

pub(crate) trait UpstreamInterface<const SIZE: usize> {
    fn poll(
        &mut self,
        tx_buffer: &mut RingBuffer<[u8; 8], SIZE>,
        rx_buffer: &mut RingBuffer<[u8; 8], SIZE>,
    ) -> Result<(), UpstreamError>;
}

#[derive(Format)]
pub(crate) enum UpstreamError {
    SpiError,
    UsbError(UsbError),
    BufferOverflow,
    InvalidMessage(InvalidMessage),
}

impl<'a, D, P, const SIZE: usize> UpstreamInterface<SIZE> for SPIUpstream<D, P>
where
    D: SpiDevice,
    P: ValidSpiPinout<D>,
{
    fn poll(
        &mut self,
        tx_buffer: &mut RingBuffer<[u8; 8], SIZE>,
        rx_buffer: &mut RingBuffer<[u8; 8], SIZE>,
    ) -> Result<(), UpstreamError> {
        self.read().map(|data| {
            for _byte in data.iter() {
                let _ = rx_buffer.push(data);
            }
        });
        if let Some(event) = tx_buffer.peek() {
            match self.send(event) {
                Ok(_) => {
                    tx_buffer.discard();
                }
                Err(_e) => return Err(UpstreamError::SpiError),
            }
        }
        Ok(())
    }
}
