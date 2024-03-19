//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

extern crate panic_usb_boot;
use defmt::{debug, info, warn, Debug2Format};
use defmt_rtt as _;

use embedded_alloc::Heap;
use embedded_hal::{digital::v2::PinState, spi::MODE_1, timer::CountDown};
use fugit::{ExtU32, RateExtU32};
use negicon_protocol::negicon_event::{NegiconEvent, NegiconEventType};

use usb_device::{
    class_prelude::UsbBusAllocator,
    prelude::{UsbDeviceBuilder, UsbVidPid},
};
// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp2040_hal as hal;
// use sparkfun_pro_micro_rp2040 as bsp;
use hal::{
    clocks::{init_clocks_and_plls, Clock},
    entry,
    gpio::{FunctionPio0, FunctionSpi, Pins},
    pac,
    pio::PIOExt,
    rom_data::reset_to_usb_boot,
    spi::FrameFormat,
    usb::UsbBus,
    watchdog::Watchdog,
    Sio, Timer,
};

use usbd_human_interface_device::{
    interface::{InBytes8, InterfaceBuilder, OutBytes8, ReportSingle},
    usb_class::UsbHidClassBuilder,
};

pub mod downstream;
pub mod upstream;

use crate::{
    downstream::spi_downstream::DownstreamDevice,
    upstream::{
        spi::SPIUpstream,
        upstream::{Upstream, UsbUpstream},
    },
};

#[global_allocator]
static HEAP: Heap = Heap::empty();

const USB_HID_DESCRIPTOR: [u8; 38] = [
    0x05, 0x01, // USAGE_PAGE (Generic Desktop)
    0x09, 0x00, // USAGE (Undefined)
    0xa1, 0x01, // COLLECTION (Application)
    0x09, 0x01, //   USAGE (Pointer)
    0xa1, 0x00, //   COLLECTION (Physical)
    // Input report
    0x09, 0x02, //     USAGE (Undefined)
    0x15, 0x00, //     LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00, //     LOGICAL_MAXIMUM (255)
    0x75, 0x08, //     REPORT_SIZE (8) - 8 bits
    0x95, 0x08, //     REPORT_COUNT (8) - 8 fields
    0x81, 0x02, //     INPUT (Data,Var,Abs)
    // Output report
    0x09, 0x03, //     USAGE (Undefined)
    0x15, 0x00, //     LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00, //     LOGICAL_MAXIMUM (255)
    0x75, 0x08, //     REPORT_SIZE (8)
    0x95, 0x08, //     REPORT_COUNT (8)
    0x91, 0x02, //     OUTPUT (Data,Var,Abs)
    0xc0, //   END_COLLECTION
    0xc0, // END_COLLECTION
];
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

#[entry]
fn main() -> ! {
    info!("Program start");
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 1024 * 64;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }
    let mut pac = pac::Peripherals::take().unwrap();
    let _core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();
    let mut delay = cortex_m::delay::Delay::new(_core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let hid = UsbHidClassBuilder::new()
        .add_device(
            InterfaceBuilder::<InBytes8, OutBytes8, ReportSingle>::new(&USB_HID_DESCRIPTOR)
                .unwrap()
                .description("Negicon v3")
                .idle_default(500.millis())
                .unwrap()
                .in_endpoint(1.millis())
                .unwrap()
                .with_out_endpoint(50.millis())
                .unwrap()
                .build(),
        )
        .build(&usb_bus);

    let (mut pio0, mut sm0, mut sm1, mut sm2, mut sm3) = pac.PIO0.split(&mut pac.RESETS);
    pins.gpio18.into_function::<FunctionPio0>();
    pins.gpio19.into_function::<FunctionPio0>();
    pins.gpio20.into_function::<FunctionPio0>();
    pins.gpio21.into_function::<FunctionPio0>();
    pins.gpio22.into_function::<FunctionPio0>();
    pins.gpio23.into_function::<FunctionPio0>();
    pins.gpio24.into_function::<FunctionPio0>();
    pins.gpio25.into_function::<FunctionPio0>();
    pins.gpio26.into_function::<FunctionPio0>();
    pins.gpio27.into_function::<FunctionPio0>();
    pins.gpio28.into_function::<FunctionPio0>();
    pins.gpio29.into_function::<FunctionPio0>();

    let mut downstreamInterface =
        downstream::spi_downstream::PioSpiDownstream::new(pio0, sm0, sm1, sm2);
    let mut tick_timer = timer.count_down();
    let mut ping_timer = timer.count_down();
    tick_timer.start(5.millis());
    ping_timer.start(5.secs());

    let usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x1209, 0x3939))
        .manufacturer("LeekLabs International")
        .product("Negicon v3")
        .serial_number("3939")
        .build();
    let mut usb_upstream = UsbUpstream::new(hid, usb_dev);
    let _i = 0u8;

    let spi_sclk = pins.gpio10.into_function::<FunctionSpi>();
    let spi_mosi = pins.gpio11.into_function::<FunctionSpi>();
    let spi_miso = pins.gpio12.into_function::<FunctionSpi>();
    let mut _spi1_cs = pins.gpio13.into_function::<FunctionSpi>();
    /*let upward_spi = hal::Spi::new(pac.SPI1, (spi_mosi, spi_miso, spi_sclk))
        .init_slave(&mut pac.RESETS, FrameFormat::MotorolaSpi(MODE_1));

    let mut _spi_upstream = SPIUpstream::new(upward_spi);*/

    let mut downstreams = [
        DownstreamDevice::new(0),
        DownstreamDevice::new(1),
        DownstreamDevice::new(2),
        DownstreamDevice::new(3),
        DownstreamDevice::new(4),
        DownstreamDevice::new(5),
        DownstreamDevice::new(6),
        DownstreamDevice::new(7),
        DownstreamDevice::new(8),
        DownstreamDevice::new(9),
        DownstreamDevice::new(10),
        DownstreamDevice::new(11),
        DownstreamDevice::new(12),
        DownstreamDevice::new(13),
        DownstreamDevice::new(14),
        DownstreamDevice::new(15),
        DownstreamDevice::new(16),
        DownstreamDevice::new(17),
        DownstreamDevice::new(18),
        DownstreamDevice::new(19),
        DownstreamDevice::new(20),
        DownstreamDevice::new(21),
        DownstreamDevice::new(22),
        DownstreamDevice::new(23),
        DownstreamDevice::new(24),
        DownstreamDevice::new(25),
        DownstreamDevice::new(26),
        DownstreamDevice::new(27),
        DownstreamDevice::new(28),
        DownstreamDevice::new(29),
        DownstreamDevice::new(30),
        DownstreamDevice::new(31),
    ];
    let controller_id = 0u8;
    let mut upstreams = [
        Upstream::new(&mut usb_upstream),
        //Upstream::new(&mut _spi_upstream),
    ];
    let mut ping = 0u8;

    loop {
        for up in upstreams.iter_mut() {
            match up.poll() {
                Ok(_) => loop {
                    match up.receive() {
                        Ok(None) => break,
                        Ok(Some(e)) => {
                            match e.event_type {
                                NegiconEventType::Reboot => {
                                    debug!("Rebooting to USB boot");
                                    reset_to_usb_boot(0, 0);
                                }
                                _ => {}
                            };
                            debug!("Received event from upstream {:?}", Debug2Format(&e))
                        }
                        Err(_e) => warn!("Error while receiving event from upstream"),
                    }
                },
                Err(e) => {
                    warn!("Error while polling upstream: {:?}", e);
                }
            }
        }

        match tick_timer.wait() {
            Ok(_) => {
                tick_timer.start(5.millis());
                for ds in downstreams.iter_mut() {
                    match ds.poll(&mut delay, &mut downstreamInterface) {
                        Ok(_) => {}
                        Err(e) => match e {
                            downstream::spi_downstream::DownstreamError::InvalidMessage => {}
                            _ => {
                                warn!("Error while polling downstream: {:?}", e);
                            }
                        },
                    }
                    match ds.receive() {
                        Ok(None) => {}
                        Ok(Some(e)) => {
                            debug!("Received event from downstream {:?}", Debug2Format(&e));
                            if !e.is_ping() {
                                for up in upstreams.iter_mut() {
                                    match up.send(&e) {
                                        Ok(_) => {}
                                        Err(e) => {
                                            warn!(
                                                "Error while enqueueing event for upstream: {:?}",
                                                e
                                            );
                                        }
                                    }
                                }
                            }
                        }
                        Err(_e) => {
                            //debug!("Error while polling downstream: {:?}", _e);
                        }
                    };
                }
            }
            Err(_) => {}
        }
        match ping_timer.wait() {
            Ok(_) => {
                ping_timer.start(500.millis());
                let _packet =
                    &mut NegiconEvent::new(NegiconEventType::Input, 0, ux::u7::new(0), 39, 1, ping)
                        .serialize();

                for up in upstreams.iter_mut() {
                    match up.send(&NegiconEvent::new(
                        NegiconEventType::Input,
                        0,
                        ux::u7::new(0),
                        39,
                        controller_id,
                        ping,
                    )) {
                        Ok(_) => {}
                        Err(e) => {
                            warn!("Error while sending event to upstream: {:?}", e);
                        }
                    }
                }
                ping = ping.wrapping_add(1);
            }
            Err(_) => {}
        }
    }
}
