extern crate alloc;
use core::{convert::Infallible, marker::PhantomData, ops::BitXor};

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
    ringbuf::RingBuffer,
};
use ux::u7;
#[derive(Format)]
pub(crate) enum DownstreamError {
    InvalidMessage,
    UnexpectedReply,
    TxOverflow,
    RxOverflow,
}

pub trait DownstreamInterface {
    fn transfer(&mut self, cs: u8, packet: &mut [u8; 8]) -> Result<[u8; 8], DownstreamError>;
}

pub(crate) struct PioSpiDownstream<
    P: PIOExt,
    SM0: StateMachineIndex,
    SM1: StateMachineIndex,
    SM2: StateMachineIndex,
> {
    pio: PIO<P>,
    cs_tx: rp2040_hal::pio::Tx<(P, SM0)>,
    data_tx: rp2040_hal::pio::Tx<(P, SM1)>,
    data_rx: rp2040_hal::pio::Rx<(P, SM1)>,
    pub(crate) slave_tx: rp2040_hal::pio::Tx<(P, SM2)>,
    pub(crate) slave_rx: rp2040_hal::pio::Rx<(P, SM2)>,
}

impl<P: PIOExt, SM0: StateMachineIndex, SM1: StateMachineIndex, SM2: StateMachineIndex>
    PioSpiDownstream<P, SM0, SM1, SM2>
{
    pub fn new(
        mut pio: PIO<P>,
        sm0: UninitStateMachine<(P, SM0)>,
        sm1: UninitStateMachine<(P, SM1)>,
        sm2: UninitStateMachine<(P, SM2)>,
    ) -> Self {
        let sck_pin_id = 18;

        let miso_pin_id = 20;
        let mosi_pin_id = 19;
        let cs_pins_base = 21;

        let slave_cs = 29;
        let slave_sck = 26;
        let slave_mosi = 28;
        let slave_miso = 27;

        let (program, wrap_source, wrap_target) = spi_program();

        let mut program = program.assemble_with_wrap(wrap_source, wrap_target);
        program.side_set = SideSet::new(true, 1, false);
        let mut program = pio.install(&program).unwrap();

        let (mut sm, mut data_rx, mut data_tx) = rp2040_hal::pio::PIOBuilder::from_program(program)
            .buffers(Buffers::RxTx)
            .in_pin_base(miso_pin_id)
            .out_pins(mosi_pin_id, 1)
            .out_shift_direction(ShiftDirection::Left)
            .side_set_pin_base(sck_pin_id)
            .clock_divisor_fixed_point(84, 0)
            /*  .autopull(true)
            .pull_threshold(32)*/
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
            .out_shift_direction(ShiftDirection::Right)
            .build(sm0);

        sm.set_pindirs([
            (cs_pins_base, PinDir::Output),
            (cs_pins_base + 1, PinDir::Output),
            (cs_pins_base + 2, PinDir::Output),
            (cs_pins_base + 3, PinDir::Output),
            (cs_pins_base + 4, PinDir::Output),
        ]);

        sm.start();

        let (program, wrap_source, wrap_target) = spi_slave_program(slave_cs, slave_sck);
        let program = program.assemble_with_wrap(wrap_source, wrap_target);
        let program = pio.install(&program).unwrap();
        let (mut sm, slave_rx, slave_tx) = rp2040_hal::pio::PIOBuilder::from_program(program)
            .buffers(Buffers::RxTx)
            .jmp_pin(slave_cs)
            .out_pins(slave_miso, 1)
            .in_pin_base(slave_mosi)
            .autopush(true)
            .push_threshold(32)
            .build(sm2);

        sm.set_pindirs([
            (slave_cs, PinDir::Input),
            (slave_sck, PinDir::Input),
            (slave_mosi, PinDir::Input),
            (slave_miso, PinDir::Output),
        ]);

        sm.start();

        Self {
            pio,
            cs_tx,
            data_tx,
            data_rx,
            slave_tx,
            slave_rx,
        }
    }
}

impl<P: PIOExt, SM0: StateMachineIndex, SM1: StateMachineIndex, SM2: StateMachineIndex>
    DownstreamInterface for PioSpiDownstream<P, SM0, SM1, SM2>
{
    fn transfer(&mut self, cs: u8, packet: &mut [u8; 8]) -> Result<[u8; 8], DownstreamError> {
        let first = make_u32(packet[0], packet[1], packet[2], packet[3]);
        let second = make_u32(packet[4], packet[5], packet[6], packet[7]);
        self.cs_tx.write_u8_replicated(cs);
        self.data_tx.write(first);
        self.data_tx.write(second);
        let mut res = [0u8; 8];
        loop {
            match self.data_rx.read() {
                Some(word) => {
                    res[0] = (word >> 24) as u8;
                    res[1] = (word >> 16) as u8;
                    res[2] = (word >> 8) as u8;
                    res[3] = word as u8;
                    break;
                }
                None => {}
            }
        }
        loop {
            match self.data_rx.read() {
                Some(word) => {
                    res[4] = (word >> 24) as u8;
                    res[5] = (word >> 16) as u8;
                    res[6] = (word >> 8) as u8;
                    res[7] = word as u8;
                    break;
                }
                None => {}
            }
        }
        Ok(res)
    }
}

pub(crate) struct DownstreamDevice {
    cs: u8,
    tx_buffer: RingBuffer<NegiconEvent, 4>,
    rx_buffer: RingBuffer<NegiconEvent, 4>,
}

impl DownstreamDevice {
    pub(crate) fn new(cs: u8) -> Self {
        Self {
            cs,
            tx_buffer: RingBuffer::new(),
            rx_buffer: RingBuffer::new(),
        }
    }

    pub fn poll(
        &mut self,
        _delay: &mut Delay,
        interface: &mut dyn DownstreamInterface,
    ) -> Result<(), DownstreamError> {
        let event = self.tx_buffer.pop().unwrap_or(NegiconEvent::new(
            NegiconEventType::Output,
            0,
            u7::new(0),
            0x39,
            39,
            0,
        ));

        let mut packet = event.serialize();
        let reply = interface.transfer(self.cs, &mut packet)?;
        let deserialized = NegiconEvent::deserialize(&reply);
        match deserialized {
            Ok(event) => {
                self.rx_buffer.push(event);
                Ok(())
            }
            Err(e) => Err(DownstreamError::InvalidMessage),
        }
    }

    pub fn send(&mut self, event: NegiconEvent) -> Result<(), DownstreamError> {
        self.tx_buffer
            .push(event)
            .map_err(|_| DownstreamError::TxOverflow)
    }

    pub fn receive(&mut self) -> Result<Option<NegiconEvent>, DownstreamError> {
        self.rx_buffer
            .pop()
            .map_or(Ok(None), |event| Ok(Some(event)))
    }
}

fn spi_program() -> (pio::Assembler<32>, Label, Label) {
    let mut program = pio::Assembler::<32>::new_with_side_set(SideSet::new(true, 1, false));
    //let mut program = pio::Assembler::<32>::new();
    let mut wrap_target = program.label();
    let mut wrap_source = program.label();
    let mut next_bit = program.label();
    let mut next_word = program.label();
    program.bind(&mut wrap_target);

    //number of words to transfer
    program.set(pio::SetDestination::Y, 1);
    program.bind(&mut next_word);
    program.pull(false, true);
    program.bind(&mut next_bit);
    // cs delay
    program.out_with_side_set(pio::OutDestination::PINS, 1, 1);
    program.in_with_side_set(pio::InSource::PINS, 1, 0);
    program.jmp(
        pio::JmpCondition::OutputShiftRegisterNotEmpty,
        &mut next_bit,
    );
    program.push(false, false);
    program.jmp(pio::JmpCondition::YDecNonZero, &mut next_word);
    program.irq(false, false, 4, false);
    program.bind(&mut wrap_source);
    (program, wrap_source, wrap_target)
}

fn cs_program() -> (pio::Assembler<32>, Label, Label) {
    let mut program = pio::Assembler::<32>::new();
    let mut wrap_target = program.label();
    let mut wrap_source = program.label();
    program.bind(&mut wrap_target);
    program.pull(false, true);
    program.out(pio::OutDestination::PINS, 5);
    //program.set(pio::SetDestination::PINS, 0x0);
    //wait for transfer to finish
    //program.nop_with_delay(31);
    program.wait(1, pio::WaitSource::IRQ, 4, false);
    program.set(pio::SetDestination::PINS, 0x1f);
    program.bind(&mut wrap_source);

    (program, wrap_source, wrap_target)
}

fn spi_slave_program(cs_index: u8, sck_index: u8) -> (pio::Assembler<32>, Label, Label) {
    let mut program = pio::Assembler::<32>::new();
    let mut wrap_target = program.label();
    let mut wrap_source = program.label();
    let mut wait_for_transfer = program.label();
    program.bind(&mut wait_for_transfer);
    //Wait for idle
    program.wait(1, pio::WaitSource::GPIO, cs_index, false);
    //Wait for CS
    program.wait(0, pio::WaitSource::GPIO, cs_index, false);
    program.bind(&mut wrap_target);
    program.pull(true, false);
    program.wait(1, pio::WaitSource::GPIO, sck_index, false);
    program.jmp(pio::JmpCondition::PinHigh, &mut wait_for_transfer);
    program.out(pio::OutDestination::PINS, 1);
    program.wait(0, pio::WaitSource::GPIO, sck_index, false);
    program.r#in(pio::InSource::PINS, 1);

    program.bind(&mut wrap_source);
    (program, wrap_source, wrap_target)
}
