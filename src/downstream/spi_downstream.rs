extern crate alloc;
use core::{convert::Infallible, marker::PhantomData};

use cortex_m::delay::Delay;
use defmt::{error, Format};
use embedded_hal::{blocking::spi::Transfer, digital::v2::OutputPin};
use pio::{Label, SideSet};
use rp2040_hal::{
    gpio::PinId,
    pac::adc::cs,
    pio::{
        Buffers, PIOExt, PinDir, ShiftDirection, StateMachine, StateMachineIndex,
        UninitStateMachine, ValidStateMachine, PIO, SM0,
    },
    pll::State,
    spi::{Enabled, SpiDevice as HalSpiDevice, ValidSpiPinout},
    Spi,
};

use negicon_protocol::{
    make_u32,
    negicon_event::{NegiconEvent, NegiconEventType},
};
use ux::u7;
#[derive(Format)]
pub(crate) enum DownstreamError {
    UnknownDevice(u8),
    UnexpectedReply,
}

pub trait DownstreamInterface {
    fn transfer(&mut self, cs: u8, packet: &mut [u8; 8]) -> Result<(), DownstreamError>;
}

pub(crate) struct PioSpiDownstream<P: PIOExt, SM0: StateMachineIndex, SM1: StateMachineIndex> {
    pio: PIO<P>,
    cs_tx: rp2040_hal::pio::Tx<(P, SM0)>,
    data_tx: rp2040_hal::pio::Tx<(P, SM1)>,
    data_rx: rp2040_hal::pio::Rx<(P, SM1)>,
}

impl<P: PIOExt, SM0: StateMachineIndex, SM1: StateMachineIndex> PioSpiDownstream<P, SM0, SM1> {
    pub fn new(
        mut pio: PIO<P>,
        sm0: UninitStateMachine<(P, SM0)>,
        sm1: UninitStateMachine<(P, SM1)>,
    ) -> Self {
        let sck_pin_id = 18;
        let miso_pin_id = 20;
        let mosi_pin_id = 19;
        let cs_pins_base = 21;

        let (program, wrap_source, wrap_target) = spi_program();
        let mut program = program.assemble_with_wrap(wrap_source, wrap_target);
        program.side_set = SideSet::new(true, 1, false);
        let mut program = pio.install(&program).unwrap();

        let (int, frac) = (10000, 0); //TODO adjust, maybe calculate with a given clock speed
        let (mut sm, mut data_rx, mut data_tx) = rp2040_hal::pio::PIOBuilder::from_program(program)
            .buffers(Buffers::RxTx)
            .in_pin_base(miso_pin_id)
            .out_pins(mosi_pin_id, 1)
            .out_shift_direction(ShiftDirection::Left)
            .side_set_pin_base(sck_pin_id)
            .build(sm1);
        sm.set_pindirs([
            (mosi_pin_id, PinDir::Output),
            (sck_pin_id, PinDir::Output),
            (miso_pin_id, PinDir::Input),
        ]);
        sm.start();

        let (program, wrap_source, wrap_target) = cs_program();
        let program = program.assemble_with_wrap(wrap_source, wrap_target);
        let program = pio.install(&program).unwrap();
        let (mut sm, _, cs_tx) = rp2040_hal::pio::PIOBuilder::from_program(program)
            .buffers(Buffers::OnlyTx)
            .out_pins(cs_pins_base, 5)
            .set_pins(cs_pins_base, 5)
            .out_shift_direction(ShiftDirection::Left)
            .build(sm0);

        sm.set_pindirs([
            (cs_pins_base, PinDir::Output),
            (cs_pins_base + 1, PinDir::Output),
            (cs_pins_base + 2, PinDir::Output),
            (cs_pins_base + 3, PinDir::Output),
            (cs_pins_base + 4, PinDir::Output),
        ]);
        sm.start();
        //      program.side_set = SideSet::new(true, 5, false);
        Self {
            pio,
            cs_tx,
            data_tx,
            data_rx,
        }
    }
}

impl<P: PIOExt, SM0: StateMachineIndex, SM1: StateMachineIndex> DownstreamInterface
    for PioSpiDownstream<P, SM0, SM1>
{
    fn transfer(&mut self, cs: u8, packet: &mut [u8; 8]) -> Result<(), DownstreamError> {
        let first = make_u32(packet[0], packet[1], packet[2], packet[3]);
        let second = make_u32(packet[4], packet[5], packet[6], packet[7]);
        self.cs_tx.write(cs.into());
        self.data_tx.write(first);
        self.data_tx.write(second);

        Ok(())
    }
}

pub(crate) struct DownstreamDevice {
    cs: u8,
}

impl DownstreamDevice {
    pub(crate) fn new(cs: u8) -> Self {
        Self { cs }
    }

    pub fn poll(
        &mut self,
        _delay: &mut Delay,
        interface: &mut dyn DownstreamInterface,
    ) -> Result<NegiconEvent, DownstreamError> {
        let mut packet =
            NegiconEvent::new(NegiconEventType::Output, 0, u7::new(0), 0x39, 39, 0).serialize();

        interface
            .transfer(self.cs, &mut packet)
            .map_err(|_| DownstreamError::UnknownDevice(0))?;
        NegiconEvent::deserialize(&packet).map_err(|_| DownstreamError::UnexpectedReply)
    }
}

fn spi_program() -> (pio::Assembler<32>, Label, Label) {
    let mut program = pio::Assembler::<32>::new();
    let mut wrap_target = program.label();
    let mut wrap_source = program.label();
    let mut next_bit = program.label();
    program.bind(&mut wrap_target);
    program.pull(false, true);
    program.bind(&mut next_bit);
    // cs delay
    program.out_with_side_set(pio::OutDestination::PINS, 1, 1);
    program.in_with_side_set(pio::InSource::PINS, 1, 0);
    program.jmp(
        pio::JmpCondition::OutputShiftRegisterNotEmpty,
        &mut next_bit,
    );
    // pull another word
    program.pull(false, false);
    // if there is one, continue transmitting
    program.jmp(
        pio::JmpCondition::OutputShiftRegisterNotEmpty,
        &mut next_bit,
    );
    // else set the irq flag and loop
    program.irq(false, false, 4, false);

    // push data bit and set clock high
    // (wait)
    // shift in data and set clock low
    // loop until work is out
    // somehow grab 2nd word
    program.irq(false, false, 4, false);
    (program, wrap_source, wrap_target)
}

fn cs_program() -> (pio::Assembler<32>, Label, Label) {
    let mut program = pio::Assembler::<32>::new();
    let mut wrap_target = program.label();
    let mut wrap_source = program.label();
    program.bind(&mut wrap_target);
    program.pull(false, true);
    program.out(pio::OutDestination::PINS, 5);
    //wait for transfer to finish
    program.wait(1, pio::WaitSource::IRQ, 4, false);
    program.set(pio::SetDestination::PINS, 0);
    program.bind(&mut wrap_source);

    (program, wrap_source, wrap_target)
}

fn in_program(sck_gpio: u8) -> (pio::Assembler<32>, Label, Label) {
    let mut program = pio::Assembler::<32>::new();
    let mut wrap_target = program.label();
    let mut wrap_source = program.label();
    let mut next_bit = program.label();
    program.bind(&mut wrap_target);
    program.pull(false, true);
    program.set(pio::SetDestination::PINS, 0);
    //program.set_with_delay_and_side_set(destination, data, delay, side_set);
    // pull chip select from fifo
    // set chip address
    // pull data word 1
    // wait
    // push data bit and set clock high
    // (wait)
    // shift in data and set clock low
    // loop until work is out
    // somehow grab 2nd word

    // Set pin as Out
    /*program.set(pio::SetDestination::PINDIRS, 1);
    // Define begin of program loop
    program.bind(&mut wrap_target);
    //Pull a new word
    program.pull(false, true);
    //Wait for cs to go low
    program.wait(0, pio::WaitSource::PIN, 0, false);

    program.bind(&mut next_bit);

    program.out_with_side_set(pio::OutDestination::PINS, 1, 1);

    program.out(pio::OutDestination::PINS, 1);

    program.wait(0, pio::WaitSource::PIN, 1, false);

    program.jmp(pio::JmpCondition::PinHigh, &mut wrap_target);

    program.jmp(
        pio::JmpCondition::OutputShiftRegisterNotEmpty,
        &mut next_bit,
    );
    // Pull next word to transfer*/
    program.bind(&mut wrap_source);
    (program, wrap_source, wrap_target)
}
