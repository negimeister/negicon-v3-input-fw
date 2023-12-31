use defmt::Format;
use ux::u7;

use crate::downstream::util::{make_i16, make_u16};
use core::ops::Shr;
#[derive(Clone, Copy)]
pub(crate) struct NegiconEvent {
    pub(crate) event_type: NegiconEventType,
    pub(crate) id: u16,
    pub(crate) sub_id: u7,
    pub(crate) value: i16,
    pub(crate) controller_id: u8,
    pub(crate) sequence: u8,
}

#[derive(PartialEq, Clone, Copy, Format)]
pub(crate) enum NegiconEventType {
    Input,
    Output,
    MemWrite,
    Reboot,
    SetControllerId,
    Log,
}

impl NegiconEvent {
    pub(crate) fn new(
        event_type: NegiconEventType,
        id: u16,
        sub_id: u7,
        value: i16,
        controller_id: u8,
        sequence: u8,
    ) -> Self {
        NegiconEvent {
            event_type,
            id,
            sub_id,
            value,
            controller_id,
            sequence,
        }
    }

    pub(crate) fn serialize(&self) -> [u8; 8] {
        let byte0: u8 = if self.event_type == NegiconEventType::Input {
            let sub: u8 = self.sub_id.into();
            0b1000_0000u8 | sub
        } else {
            self.event_type as u8
        };
        [
            byte0,
            self.id.shr(8) as u8,
            self.id as u8,
            self.value.shr(8) as u8,
            self.value as u8,
            self.controller_id,
            self.sequence,
            0u8,
        ]
    }

    pub(crate) fn deserialize(data: [u8; 8]) -> Self {
        let event_type = if data[0] & 0b1000_0000 != 0 {
            NegiconEventType::Input
        } else {
            match data[0] {
                1 => NegiconEventType::Output,
                2 => NegiconEventType::MemWrite,
                3 => NegiconEventType::Reboot,
                4 => NegiconEventType::SetControllerId,
                5 => NegiconEventType::Log,
                _ => NegiconEventType::Input,
            }
        };
        let sub_id: u7 = u7::new(data[0] & 0b0111_1111);
        let id = make_u16(data[1], data[2]);
        let value = make_i16(data[3], data[4]);
        let controller_id = data[5];
        let sequence = data[6];

        NegiconEvent {
            event_type,
            id,
            sub_id,
            value,
            controller_id,
            sequence,
        }
    }
}
