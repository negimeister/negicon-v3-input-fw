use super::{ringbuf::RingBuffer, spi::SPIUpstream};
use negicon_protocol::negicon_event::NegiconEvent;

use defmt::{warn, Format};
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
    buffer: RingBuffer<[u8; 8]>,
    interface: &'a mut dyn UpstreamInterface,
}

impl<'a> Upstream<'a> {
    pub(crate) fn new(interface: &'a mut dyn UpstreamInterface) -> Self {
        Self {
            buffer: RingBuffer::new(),
            interface,
        }
    }

    pub(crate) fn poll(&mut self) -> Result<Option<NegiconEvent>, UpstreamError> {
        match self.send() {
            Ok(_) => {}
            Err(e) => {} //warn!("Failed to send event to upstream {:?}", e),
        }
        self.interface.receive()
    }

    pub(crate) fn enqueue(&mut self, event: NegiconEvent) -> Result<(), UpstreamError> {
        match self.buffer.push(event.serialize()) {
            Ok(_) => Ok(()),
            Err(_) => panic!("Upstream buffer overflow"), //TODO probably shouldn't panic
        }
    }

    pub(crate) fn enqueue_raw(&mut self, event: &[u8; 8]) -> Result<(), UpstreamError> {
        match self.buffer.push(*event) {
            Ok(_) => Ok(()),
            Err(_) => Ok(()), //TODO error handling
        }
    }

    pub(crate) fn send(&mut self) -> Result<(), UpstreamError> {
        if let Some(event) = self.buffer.peek() {
            match self.interface.send(event) {
                Ok(_) => Ok(self.buffer.discard()),
                Err(e) => return Err(e),
            }
        } else {
            Ok(())
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

impl<B> UpstreamInterface for UsbUpstream<'_, B>
where
    B: UsbBus,
{
    fn receive(&mut self) -> Result<Option<NegiconEvent>, UpstreamError> {
        self.dev.poll(&mut [&mut self.hid]);
        let mut data = [0u8; 8];
        match self.hid.device().read_report(&mut data) {
            Ok(_report) => Ok(Some(NegiconEvent::deserialize(data))),
            Err(e) => match e {
                UsbError::WouldBlock => Ok(None),
                _ => Err(UpstreamError::UsbError(e)),
            },
        }
    }

    fn send(&mut self, event: &mut [u8; 8]) -> Result<(), UpstreamError> {
        match self.hid.device().write_report(event) {
            Ok(_) => Ok(()),
            Err(e) => Err(UpstreamError::UsbError(e)),
        }
    }
}

pub(crate) trait UpstreamInterface {
    fn receive(&mut self) -> Result<Option<NegiconEvent>, UpstreamError>;
    fn send(&mut self, event: &mut [u8; 8]) -> Result<(), UpstreamError>;
}

#[derive(Format)]
pub(crate) enum UpstreamError {
    SpiError,
    UsbError(UsbError),
}

impl<D, P> UpstreamInterface for SPIUpstream<D, P>
where
    D: SpiDevice,
    P: ValidSpiPinout<D>,
{
    fn send(&mut self, event: &mut [u8; 8]) -> Result<(), UpstreamError> {
        match self.transmit_event(event) {
            Ok(_) => Ok(()),
            Err(_) => Err(UpstreamError::SpiError),
        }
    }

    fn receive(&mut self) -> Result<Option<NegiconEvent>, UpstreamError> {
        todo!()
    }
}
